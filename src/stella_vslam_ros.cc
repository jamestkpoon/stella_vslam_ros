#include <stella_vslam_ros.h>
#include <stella_vslam/publish/map_publisher.h>

#include <chrono>
#include <filesystem>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <Eigen/Geometry>


#define VOCAB_PARAM "vocab"
#define MASK_PARAM "mask"
#define RECTIFY_STEREO_PARAM "rectify_stereo"

#define SLAM_FRAME_LOOKUP_TIMEOUT 5.0
#define MESSAGEPACK_EXT ".msg"


namespace stella_vslam_ros {
system::system(const std::shared_ptr<stella_vslam::config>& cfg, const std::string& vocab_file_path, const std::string& mask_img_path)
    : node_(std::make_shared<rclcpp::Node>("run_slam"))
{
    init(cfg, vocab_file_path, mask_img_path);
}

system::system(const std::shared_ptr<stella_vslam::config>& cfg, std::shared_ptr<rclcpp::Node>& node) : node_(node)
{
    init(cfg,
        node_->declare_parameter<std::string>(VOCAB_PARAM, ""),
        node_->declare_parameter<std::string>(MASK_PARAM, "")
    );
}

void system::init(const std::shared_ptr<stella_vslam::config>& cfg, const std::string& vocab_file_path, const std::string& mask_img_path)
{
    cfg_ = cfg;
    SLAM_ = std::make_shared<stella_vslam::system>(cfg_, vocab_file_path);
    mask_ = mask_img_path.empty() ? cv::Mat{} : cv::imread(mask_img_path, cv::IMREAD_GRAYSCALE);

    pose_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("~/camera_pose", 1);
    map_to_odom_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
    tf_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

    loadmap_svc_ = node_->create_service<nav2_msgs::srv::LoadMap>(
        "~/load_map",
        std::bind(&system::load_map_svc, this, std::placeholders::_1, std::placeholders::_2)
    );
    savemap_svc_ = node_->create_service<nav2_msgs::srv::SaveMap>(
        "~/save_map",
        std::bind(&system::save_map_svc, this, std::placeholders::_1, std::placeholders::_2)
    );
    togglemap_svc_ = node_->create_service<std_srvs::srv::SetBool>(
        "~/enable_mapping",
        std::bind(&system::toggle_mapping_svc, this, std::placeholders::_1, std::placeholders::_2)
    );

    custom_qos_ = rmw_qos_profile_default;
    custom_qos_.depth = 1;
    exec_.add_node(node_);
    init_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 1,
        std::bind(&system::init_pose_callback, this, std::placeholders::_1));
    setParams();
}

void system::publish_pose(const Eigen::Matrix4d& cam_pose_wc, const rclcpp::Time& stamp) {
    // Extract rotation matrix and translation vector from
    Eigen::Matrix3d rot(cam_pose_wc.block<3, 3>(0, 0));
    Eigen::Translation3d trans(cam_pose_wc.block<3, 1>(0, 3));
    Eigen::Affine3d map_to_camera_affine(trans * rot);
    Eigen::AngleAxisd rot_ros_to_cv_map_frame(
        (Eigen::Matrix3d() << 0, 0, 1,
         -1, 0, 0,
         0, -1, 0)
            .finished());

    // Transform map frame from CV coordinate system to ROS coordinate system
    map_to_camera_affine.prerotate(rot_ros_to_cv_map_frame);

    // apply slam_frame_transform_ for the "real" map frame
    map_to_camera_affine = slam_frame_transform_ * (map_to_camera_affine * rot_ros_to_cv_map_frame.inverse());

    // Create odometry message and update it with current camera pose
    nav_msgs::msg::Odometry pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = map_frame_;
    pose_msg.child_frame_id = camera_frame_;
    pose_msg.pose.pose = tf2::toMsg(map_to_camera_affine);
    pose_pub_->publish(pose_msg);

    // Send map->odom transform. Set publish_tf to false if not using TF
    if (publish_tf_) {
        try {
            auto camera_to_odom = tf_->lookupTransform(
                camera_optical_frame_, odom_frame_, tf2_ros::fromMsg(builtin_interfaces::msg::Time(stamp)),
                tf2::durationFromSec(0.0));
            Eigen::Affine3d camera_to_odom_affine = tf2::transformToEigen(camera_to_odom.transform);

            auto map_to_odom_msg = tf2::eigenToTransform(map_to_camera_affine * camera_to_odom_affine);
            tf2::TimePoint transform_timestamp = tf2_ros::fromMsg(stamp) + tf2::durationFromSec(transform_tolerance_);
            map_to_odom_msg.header.stamp = tf2_ros::toMsg(transform_timestamp);
            map_to_odom_msg.header.frame_id = map_frame_;
            map_to_odom_msg.child_frame_id = odom_frame_;
            map_to_odom_broadcaster_->sendTransform(map_to_odom_msg);
        }
        catch (tf2::TransformException& ex) {
            RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Transform failed: " << ex.what());
        }
    }
}

void system::setParams() {
    odom_frame_ = std::string("odom");
    odom_frame_ = node_->declare_parameter("odom_frame", odom_frame_);

    map_frame_ = std::string("map");
    map_frame_ = node_->declare_parameter("map_frame", map_frame_);

    base_link_ = std::string("base_footprint");
    base_link_ = node_->declare_parameter("base_link", base_link_);

    camera_frame_ = std::string("camera_frame");
    camera_frame_ = node_->declare_parameter("camera_frame", camera_frame_);

    // Set publish_tf to false if not using TF
    publish_tf_ = true;
    publish_tf_ = node_->declare_parameter("publish_tf", publish_tf_);

    // Publish pose's timestamp in the future
    transform_tolerance_ = 0.5;
    transform_tolerance_ = node_->declare_parameter("transform_tolerance", transform_tolerance_);
}

void system::load_map_and_disable_mapping_on_restart(const std::string& filepath)
{
    auto yaml = YAML::LoadFile(filepath);

    auto map_db = std::filesystem::path(filepath).parent_path() / yaml["map_db"].as<std::string>();

    geometry_msgs::msg::Transform transform;
    transform.translation.x = yaml["transform"]["translation"]["x"].as<double>();
    transform.translation.y = yaml["transform"]["translation"]["y"].as<double>();
    transform.translation.z = yaml["transform"]["translation"]["z"].as<double>();
    transform.rotation.x = yaml["translation"]["rotation"]["x"].as<double>();
    transform.rotation.y = yaml["translation"]["rotation"]["y"].as<double>();
    transform.rotation.z = yaml["translation"]["rotation"]["z"].as<double>();
    transform.rotation.w = yaml["translation"]["rotation"]["w"].as<double>();

    SLAM_->load_map_database(map_db.string());
    set_slam_frame_transform(transform);
    SLAM_->startup(false);
    SLAM_->disable_mapping_module();
}

void system::init_pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    if (camera_optical_frame_.empty()) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Camera link is not set: no images were received yet");
        return;
    }

    Eigen::Translation3d trans(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z);
    Eigen::Quaterniond rot_q(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z);
    Eigen::Affine3d initialpose_affine(trans * rot_q);

    Eigen::Affine3d map_to_initialpose_frame_affine;
    try {
        auto map_to_initialpose_frame = tf_->lookupTransform(
            map_frame_, msg->header.frame_id, tf2_ros::fromMsg(msg->header.stamp),
            tf2::durationFromSec(0.0));
        map_to_initialpose_frame_affine = tf2::transformToEigen(
            map_to_initialpose_frame.transform);
    }
    catch (tf2::TransformException& ex) {
        RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Transform failed: " << ex.what());
        return;
    }

    Eigen::Affine3d base_link_to_camera_affine;
    try {
        auto base_link_to_camera = tf_->lookupTransform(
            base_link_, camera_optical_frame_, tf2_ros::fromMsg(msg->header.stamp),
            tf2::durationFromSec(0.0));
        base_link_to_camera_affine = tf2::transformToEigen(base_link_to_camera.transform);
    }
    catch (tf2::TransformException& ex) {
        RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Transform failed: " << ex.what());
        return;
    }

    // Target transform is map_cv -> camera_link and known parameters are following:
    //   rot_cv_to_ros_map_frame: T(map_cv -> map)
    //   map_to_initialpose_frame_affine: T(map -> `msg->header.frame_id`)
    //   initialpose_affine: T(`msg->header.frame_id` -> base_link)
    //   base_link_to_camera_affine: T(base_link -> camera_link)
    // The flow of the transformation is as follows:
    //   map_cv -> map -> `msg->header.frame_id` -> base_link -> camera_link

    init_pose(
        map_to_initialpose_frame_affine
            * initialpose_affine * base_link_to_camera_affine
    );
}

bool system::init_pose(const Eigen::Affine3d& cam_pose) {
    Eigen::Matrix3d rot_cv_to_ros_map_frame;
    rot_cv_to_ros_map_frame << 0, -1, 0,
        0, 0, -1,
        1, 0, 0;

    Eigen::Matrix4d cam_pose_cv = (rot_cv_to_ros_map_frame * cam_pose).matrix();
    const Eigen::Vector3d normal_vector = (Eigen::Vector3d() << 0., 1., 0.).finished();

    bool res = SLAM_->relocalize_by_pose_2d(cam_pose_cv, normal_vector);

    if(res) {
        RCLCPP_INFO(node_->get_logger(), "Initial pose set");
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Failed to set initial pose");
    }

    return res;
}

void system::save_map_svc(const std::shared_ptr<nav2_msgs::srv::SaveMap::Request> request,
    std::shared_ptr<nav2_msgs::srv::SaveMap::Response> response)
{
    try {
        SLAM_->save_map_database(request->map_url + MESSAGEPACK_EXT);

        std::ofstream yaml; yaml.open(request->map_url + ".yaml");
        auto stem = std::filesystem::path(request->map_url).stem().string();
        auto map_db_local_filepath = stem + MESSAGEPACK_EXT;
        auto transform = tf2::eigenToTransform(slam_frame_transform_).transform;
        yaml << "map_db: " << map_db_local_filepath << "\n";
        yaml << "transform:\n";
        yaml << "  translation:\n";
        yaml << "    x: " << std::to_string(transform.translation.x) << "\n";
        yaml << "    y: " << std::to_string(transform.translation.y) << "\n";
        yaml << "    z: " << std::to_string(transform.translation.z) << "\n";
        yaml << "  rotation:\n";
        yaml << "    x: " << std::to_string(transform.rotation.x) << "\n";
        yaml << "    y: " << std::to_string(transform.rotation.y) << "\n";
        yaml << "    z: " << std::to_string(transform.rotation.z) << "\n";
        yaml << "    w: " << std::to_string(transform.rotation.w) << "\n";
        yaml.close();        

        response->result = true;
    } catch(...) {
        response->result = false;
    }
}

void system::load_map_svc(const std::shared_ptr<nav2_msgs::srv::LoadMap::Request> request,
    std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response)
{
    try {
        SLAM_->shutdown();
        load_map_and_disable_mapping_on_restart(request->map_url);
        response->result = true;
    } catch(...) {
        response->result = false;
    }
}

void system::toggle_mapping_svc(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    try {
        if (request->data) {
            SLAM_->enable_mapping_module();
            response->message = "Mapping module enabled";
        }
        else {
            SLAM_->disable_mapping_module();
            response->message = "Mapping module disabled";
        }

        response->success = true;
    } catch(...) {
        response->success = false;
    }
}

void system::set_slam_frame_transform(const geometry_msgs::msg::Transform& transform) {
    slam_frame_transform_ = tf2::transformToEigen(transform).inverse();
}

void system::set_slam_frame_transform_from_lookup() {
    geometry_msgs::msg::Transform transform;

    try {
        auto map_to_camera_stf = tf_->lookupTransform(
            map_frame_, camera_optical_frame_, tf2::TimePointZero,
            tf2::durationFromSec(SLAM_FRAME_LOOKUP_TIMEOUT));
        transform = map_to_camera_stf.transform;
    }
    catch (tf2::TransformException& ex) {
        RCLCPP_ERROR(node_->get_logger(), ex.what());

        transform.translation.x = transform.translation.y = transform.translation.z = 0.0;
        transform.rotation.x = transform.rotation.y = transform.rotation.z = 0.0;
        transform.rotation.w = 1.0;
        RCLCPP_WARN(node_->get_logger(), "SLAM frame is identity");
    }

    set_slam_frame_transform(transform);
}


mono::mono(const std::shared_ptr<stella_vslam::config>& cfg, const std::string& vocab_file_path, const std::string& mask_img_path)
    : system(cfg, vocab_file_path, mask_img_path) {
    initialize_subs();
}

mono::mono(const std::shared_ptr<stella_vslam::config>& cfg, std::shared_ptr<rclcpp::Node>& node)
    : system(cfg, node)
{
    initialize_subs();
}

void mono::initialize_subs()
{
    sub_ = image_transport::create_subscription(
        node_.get(), "camera/image_raw", [this](const sensor_msgs::msg::Image::ConstSharedPtr& msg) { callback(msg); }, "raw", custom_qos_);
}

void mono::callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    if (camera_optical_frame_.empty()) {
        camera_optical_frame_ = msg->header.frame_id;
    }
    const rclcpp::Time tp_1 = node_->now();
    const double timestamp = tp_1.seconds();

    // input the current frame and estimate the camera pose
    auto cam_pose_wc = SLAM_->feed_monocular_frame(cv_bridge::toCvShare(msg)->image, timestamp, mask_);

    const rclcpp::Time tp_2 = node_->now();
    const double track_time = (tp_2 - tp_1).seconds();

    //track times in seconds
    track_times_.push_back(track_time);

    if (cam_pose_wc) {
        publish_pose(*cam_pose_wc, msg->header.stamp);
    }
}


stereo::stereo(const std::shared_ptr<stella_vslam::config>& cfg, const std::string& vocab_file_path, const std::string& mask_img_path,
               const bool rectify)
    : system(cfg, vocab_file_path, mask_img_path)
{
    rectify_ = rectify;
    initialize_subs();
}

stereo::stereo(const std::shared_ptr<stella_vslam::config>& cfg, std::shared_ptr<rclcpp::Node>& node)
    : system(cfg, node)
{
    rectify_ = node_->declare_parameter<bool>(RECTIFY_STEREO_PARAM, false);
    initialize_subs();
}

void stereo::initialize_subs()
{
    rectifier_ = rectify_ ? std::make_shared<stella_vslam::util::stereo_rectifier>(cfg_) : nullptr;
    left_sf_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(node_, "camera/left/image_raw");
    right_sf_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(node_, "camera/right/image_raw");

    use_exact_time_ = false;
    use_exact_time_ = node_->declare_parameter("use_exact_time", use_exact_time_);
    if (use_exact_time_) {
        exact_time_sync_ = std::make_shared<ExactTimeSyncPolicy::Sync>(2, *left_sf_, *right_sf_);
        exact_time_sync_->registerCallback(&stereo::callback, this);
    }
    else {
        approx_time_sync_ = std::make_shared<ApproximateTimeSyncPolicy::Sync>(10, *left_sf_, *right_sf_);
        approx_time_sync_->registerCallback(&stereo::callback, this);
    }
}

void stereo::callback(const sensor_msgs::msg::Image::ConstSharedPtr& left, const sensor_msgs::msg::Image::ConstSharedPtr& right) {
    if (camera_optical_frame_.empty()) {
        camera_optical_frame_ = left->header.frame_id;
    }
    auto leftcv = cv_bridge::toCvShare(left)->image;
    auto rightcv = cv_bridge::toCvShare(right)->image;
    if (leftcv.empty() || rightcv.empty()) {
        return;
    }

    if (rectifier_) {
        rectifier_->rectify(leftcv, rightcv, leftcv, rightcv);
    }

    const rclcpp::Time tp_1 = node_->now();
    const double timestamp = tp_1.seconds();

    // input the current frame and estimate the camera pose
    auto cam_pose_wc = SLAM_->feed_stereo_frame(leftcv, rightcv, timestamp, mask_);

    const rclcpp::Time tp_2 = node_->now();
    const double track_time = (tp_2 - tp_1).seconds();

    //track times in seconds
    track_times_.push_back(track_time);

    if (cam_pose_wc) {
        publish_pose(*cam_pose_wc, left->header.stamp);
    }
}


rgbd::rgbd(const std::shared_ptr<stella_vslam::config>& cfg, const std::string& vocab_file_path, const std::string& mask_img_path)
    : system(cfg, vocab_file_path, mask_img_path)
{
    initialize_subs();
}

rgbd::rgbd(const std::shared_ptr<stella_vslam::config>& cfg, std::shared_ptr<rclcpp::Node>& node)
    : system(cfg, node)
{
    initialize_subs();
}

void rgbd::initialize_subs()
{
    color_sf_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(node_, "camera/color/image_raw");
    depth_sf_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(node_, "camera/depth/image_raw");

    use_exact_time_ = false;
    use_exact_time_ = node_->declare_parameter("use_exact_time", use_exact_time_);
    if (use_exact_time_) {
        exact_time_sync_ = std::make_shared<ExactTimeSyncPolicy::Sync>(2, *color_sf_, *depth_sf_);
        exact_time_sync_->registerCallback(&rgbd::callback, this);
    }
    else {
        approx_time_sync_ = std::make_shared<ApproximateTimeSyncPolicy::Sync>(10, *color_sf_, *depth_sf_);
        approx_time_sync_->registerCallback(&rgbd::callback, this);
    }
}

void rgbd::callback(const sensor_msgs::msg::Image::ConstSharedPtr& color, const sensor_msgs::msg::Image::ConstSharedPtr& depth) {
    if (camera_optical_frame_.empty()) {
        camera_optical_frame_ = color->header.frame_id;
    }
    auto colorcv = cv_bridge::toCvShare(color)->image;
    auto depthcv = cv_bridge::toCvShare(depth)->image;
    if (colorcv.empty() || depthcv.empty()) {
        return;
    }
    if (depthcv.type() == CV_32FC1) {
        cv::patchNaNs(depthcv);
    }

    const rclcpp::Time tp_1 = node_->now();
    const double timestamp = tp_1.seconds();

    // input the current frame and estimate the camera pose
    auto cam_pose_wc = SLAM_->feed_RGBD_frame(colorcv, depthcv, timestamp, mask_);

    const rclcpp::Time tp_2 = node_->now();
    const double track_time = (tp_2 - tp_1).seconds();

    // track time in seconds
    track_times_.push_back(track_time);

    if (cam_pose_wc) {
        publish_pose(*cam_pose_wc, color->header.stamp);
    }
}

} // namespace stella_vslam_ros
