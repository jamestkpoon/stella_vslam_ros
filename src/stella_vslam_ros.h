#ifndef STELLA_SLAM_ROS_H
#define STELLA_SLAM_ROS_H

#include <stella_vslam/system.h>
#include <stella_vslam/config.h>
#include <stella_vslam/util/stereo_rectifier.h>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>

#include <opencv2/core/core.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <nav2_msgs/srv/load_map.hpp>
#include <nav2_msgs/srv/save_map.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace stella_vslam_ros {
class system {
public:
    system(const std::shared_ptr<stella_vslam::config>& cfg, const std::string& vocab_file_path, const std::string& mask_img_path);
    system(const std::shared_ptr<stella_vslam::config>& cfg, std::shared_ptr<rclcpp::Node>& node);
    void publish_pose(const Eigen::Matrix4d& cam_pose_wc, const rclcpp::Time& stamp);
    void setParams();
    void load_map_and_disable_mapping_on_restart(const std::string& path);
    std::shared_ptr<stella_vslam::system> SLAM_;
    std::shared_ptr<stella_vslam::config> cfg_;
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::executors::SingleThreadedExecutor exec_;
    rmw_qos_profile_t custom_qos_;
    cv::Mat mask_;
    std::vector<double> track_times_;
    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> pose_pub_;
    std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>>
        init_pose_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> map_to_odom_broadcaster_;
    std::string odom_frame_, map_frame_, base_link_;
    std::string camera_frame_, camera_optical_frame_;
    std::unique_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
    bool publish_tf_;
    double transform_tolerance_;

private:
    void init_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void save_map_svc(const std::shared_ptr<nav2_msgs::srv::SaveMap::Request> request,
        std::shared_ptr<nav2_msgs::srv::SaveMap::Response> response);
    void load_map_svc(const std::shared_ptr<nav2_msgs::srv::LoadMap::Request> request,
        std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response);
    void enable_mapping_svc(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    void init(const std::shared_ptr<stella_vslam::config>& cfg, const std::string& vocab_file_path, const std::string& mask_img_path);

protected:
    virtual void initialize_subs() { RCLCPP_ERROR(node_->get_logger(), "No subs to initialize"); }
};

class mono : public system {
public:
    mono(const std::shared_ptr<stella_vslam::config>& cfg, const std::string& vocab_file_path, const std::string& mask_img_path);
    mono(const std::shared_ptr<stella_vslam::config>& cfg, std::shared_ptr<rclcpp::Node>& node);
    void callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

    image_transport::Subscriber sub_;

private:
    void initialize_subs() override;
};

class stereo : public system {
public:
    stereo(const std::shared_ptr<stella_vslam::config>& cfg, const std::string& vocab_file_path, const std::string& mask_img_path,
           const bool rectify);
    stereo(const std::shared_ptr<stella_vslam::config>& cfg, std::shared_ptr<rclcpp::Node>& node);
    void callback(const sensor_msgs::msg::Image::ConstSharedPtr& left, const sensor_msgs::msg::Image::ConstSharedPtr& right);

    std::shared_ptr<stella_vslam::util::stereo_rectifier> rectifier_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> left_sf_, right_sf_;
    using ApproximateTimeSyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    std::shared_ptr<ApproximateTimeSyncPolicy::Sync> approx_time_sync_;
    using ExactTimeSyncPolicy = message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    std::shared_ptr<ExactTimeSyncPolicy::Sync> exact_time_sync_;
    bool use_exact_time_;

private:
    void initialize_subs() override;
    bool rectify_;
};

class rgbd : public system {
public:
    rgbd(const std::shared_ptr<stella_vslam::config>& cfg, const std::string& vocab_file_path, const std::string& mask_img_path);
    rgbd(const std::shared_ptr<stella_vslam::config>& cfg, std::shared_ptr<rclcpp::Node>& node);
    void callback(const sensor_msgs::msg::Image::ConstSharedPtr& color, const sensor_msgs::msg::Image::ConstSharedPtr& depth);

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> color_sf_, depth_sf_;
    using ApproximateTimeSyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    std::shared_ptr<ApproximateTimeSyncPolicy::Sync> approx_time_sync_;
    using ExactTimeSyncPolicy = message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    std::shared_ptr<ExactTimeSyncPolicy::Sync> exact_time_sync_;
    bool use_exact_time_;

private:
    void initialize_subs() override;
};

} // namespace stella_vslam_ros

#endif // STELLA_SLAM_ROS_H
