#include <stella_vslam_ros.h>
#include <stella_vslam/publish/map_publisher.h>

#include <chrono>
#include <filesystem>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <Eigen/Geometry>


#define VOCAB_PARAM "vocab"
#define MASK_PARAM "mask"
#define RECTIFY_STEREO_PARAM "rectify_stereo"

#define MESSAGEPACK_EXT ".msg"


std::vector<std::string> split_string(const std::string& s, const char sep)
{
    std::vector<std::string> out_;
    std::string substring_ = "";
    for(const char& c : s + sep)
    {
        if(c != sep) substring_ += c;
        else if(!substring_.empty())
        {
            out_.push_back(substring_);
            substring_.clear();
        }
    }

    return out_;
}

geometry_msgs::msg::TransformStamped from_tum(const std::string& line, const char sep = ' ')
{
    std::vector<double> doubles;
    for(const auto& s : split_string(line, sep)) {
        doubles.push_back(std::atof(s.c_str()));
    }

    geometry_msgs::msg::TransformStamped stf;

    stf.header.stamp.sec = doubles[0];
    stf.header.stamp.nanosec = (doubles[0] - stf.header.stamp.sec) * 1e9;

    stf.transform.translation.x = doubles[1];
    stf.transform.translation.y = doubles[2];
    stf.transform.translation.z = doubles[3];
    stf.transform.rotation.x = doubles[4];
    stf.transform.rotation.y = doubles[5];
    stf.transform.rotation.z = doubles[6];
    stf.transform.rotation.w = doubles[7];

    return stf;
}


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
    init_pose_sub_ = node_->create_subscription<geometry_msgs::msg::TransformStamped>(
        "~/initial_pose", 1,
        std::bind(&system::init_pose_callback, this, std::placeholders::_1));
    setParams();

    slam_frame_transform_.setIdentity();
}

void system::to_ros(const Eigen::Matrix4d& cam_pose_wc, Eigen::Affine3d& map_to_camera_affine, Eigen::Affine3d& map_to_camera_ros) {
    Eigen::AngleAxisd rot_ros_to_cv_map_frame(
        (Eigen::Matrix3d() << 0, 0, 1,
         -1, 0, 0,
         0, -1, 0)
            .finished());

    // Extract rotation matrix and translation vector from
    Eigen::Matrix3d rot(cam_pose_wc.block<3, 3>(0, 0));
    Eigen::Translation3d trans(cam_pose_wc.block<3, 1>(0, 3));
    map_to_camera_affine = trans * rot;

    // Transform map frame from CV coordinate system to ROS coordinate system
    map_to_camera_affine.prerotate(rot_ros_to_cv_map_frame);

    // apply slam_frame_transform_ for the "real" map frame
    map_to_camera_affine = slam_frame_transform_ * map_to_camera_affine;

    map_to_camera_ros = map_to_camera_affine * rot_ros_to_cv_map_frame.inverse();
}

void system::publish_pose(const Eigen::Matrix4d& cam_pose_wc, const rclcpp::Time& stamp) {
    Eigen::Affine3d map_to_camera_affine, map_to_camera_ros;
    to_ros(cam_pose_wc, map_to_camera_affine, map_to_camera_ros);

    // Create odometry message and update it with current camera pose
    nav_msgs::msg::Odometry pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = map_frame_;
    pose_msg.child_frame_id = camera_frame_;
    pose_msg.pose.pose = tf2::toMsg(map_to_camera_ros);
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

    auto map_db = std::filesystem::path(filepath).parent_path() / yaml["map"].as<std::string>();
    SLAM_->load_map_database(map_db.string());
    SLAM_->startup(false);
    SLAM_->disable_mapping_module();

    geometry_msgs::msg::Transform transform;
    transform.translation.x = yaml["transform"]["translation"]["x"].as<double>();
    transform.translation.y = yaml["transform"]["translation"]["y"].as<double>();
    transform.translation.z = yaml["transform"]["translation"]["z"].as<double>();
    transform.rotation.x = yaml["transform"]["rotation"]["x"].as<double>();
    transform.rotation.y = yaml["transform"]["rotation"]["y"].as<double>();
    transform.rotation.z = yaml["transform"]["rotation"]["z"].as<double>();
    transform.rotation.w = yaml["transform"]["rotation"]["w"].as<double>();
    set_slam_frame_transform(transform);
}

void system::set_slam_frame_transform(const geometry_msgs::msg::Transform& transform) {
    slam_frame_transform_ = tf2::transformToEigen(transform);
}

void system::init_pose_callback(
    const geometry_msgs::msg::TransformStamped::SharedPtr msg) {
    if (camera_optical_frame_.empty()) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Camera link is not set: no images were received yet");
        return;
    }

    Eigen::Affine3d initialpose_affine = tf2::transformToEigen(msg->transform);

    Eigen::Affine3d map_to_initialpose_frame_affine;
    try {
        auto map_to_initialpose_frame = tf_->lookupTransform(
            map_frame_, msg->header.frame_id, tf2_ros::fromMsg(msg->header.stamp),
            tf2::durationFromSec(0.0));
        map_to_initialpose_frame_affine = tf2::transformToEigen(
            map_to_initialpose_frame.transform);
        map_to_initialpose_frame_affine = slam_frame_transform_.inverse() * map_to_initialpose_frame_affine;
    }
    catch (tf2::TransformException& ex) {
        RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Transform failed: " << ex.what());
        return;
    }

    Eigen::Affine3d initialpose_child_frame_to_camera_affine;
    try {
        auto initialpose_child_frame_to_camera = tf_->lookupTransform(
            msg->child_frame_id, camera_optical_frame_, tf2_ros::fromMsg(msg->header.stamp),
            tf2::durationFromSec(0.0));
        initialpose_child_frame_to_camera_affine = tf2::transformToEigen(initialpose_child_frame_to_camera.transform);
    }
    catch (tf2::TransformException& ex) {
        RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Transform failed: " << ex.what());
        return;
    }

    init_pose(
        // map -> initialpose frame_id -> initialpose child_frame_id -> camera
        map_to_initialpose_frame_affine * initialpose_affine * initialpose_child_frame_to_camera_affine
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
        // map database
        SLAM_->save_map_database(request->map_url + MESSAGEPACK_EXT);

        // metadata
        std::ofstream yaml; yaml.open(request->map_url + ".yaml");
        auto stem = std::filesystem::path(request->map_url).stem().string();
        auto map_db_local_filepath = stem + MESSAGEPACK_EXT;
        auto transform = tf2::eigenToTransform(slam_frame_transform_).transform;
        yaml << "map: " << map_db_local_filepath << "\n";
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

        // trajectory
        std::string tum_fp = request->map_url + "_trajectory_tum.csv",
            tum_ros_fp = request->map_url + "_trajectory_ros.csv";
        SLAM_->save_frame_trajectory(tum_fp, "TUM");
        tum_to_ros(tum_fp, tum_ros_fp);

        response->result = true;
    } catch(...) {
        response->result = false;
    }
}

void system::tum_to_ros(const std::string& tum_fp, const std::string& ros_fp) {
    std::ifstream reader (tum_fp);
    if(!reader.is_open()) { return; }

    std::ofstream writer; writer.open(ros_fp);
    std::string tum_line;
    while(std::getline(reader, tum_line)) {
        auto stf = from_tum(tum_line);
        auto cam_pose_wc = tf2::transformToEigen(stf).matrix();
        Eigen::Affine3d map_to_camera_affine, map_to_camera_ros;
        to_ros(cam_pose_wc, map_to_camera_affine, map_to_camera_ros);
        auto cam_pose = tf2::toMsg(map_to_camera_ros);

        writer << std::to_string(rclcpp::Time(stf.header.stamp).seconds()) << " ";
        writer << std::to_string(cam_pose.position.x) << " ";
        writer << std::to_string(cam_pose.position.y) << " ";
        writer << std::to_string(cam_pose.position.z) << " ";
        writer << std::to_string(cam_pose.orientation.x) << " ";
        writer << std::to_string(cam_pose.orientation.y) << " ";
        writer << std::to_string(cam_pose.orientation.z) << " ";
        writer << std::to_string(cam_pose.orientation.w) << "\n";
    }

    writer.close();
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
    const auto timestamp = rclcpp::Time(msg->header.stamp).seconds();

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
    const auto timestamp = rclcpp::Time(left->header.stamp).seconds();

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
    const auto timestamp = rclcpp::Time(color->header.stamp).seconds();

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
