#include <stella_vslam/config.h>
#include <stella_vslam/system.h>
#include <stella_vslam_ros.h>


#define CONFIG_PARAM "config"
#define INITIAL_MAP_PARAM "map"


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("vslam");
    auto cfg = std::make_shared<stella_vslam::config>(
        node->declare_parameter<std::string>(CONFIG_PARAM, "")
    );

    std::shared_ptr<stella_vslam_ros::system> ros;    
    if (cfg->camera_->setup_type_ == stella_vslam::camera::setup_type_t::Monocular) {
        ros = std::make_shared<stella_vslam_ros::mono>(cfg, node);
    }
    else if (cfg->camera_->setup_type_ == stella_vslam::camera::setup_type_t::Stereo) {
        ros = std::make_shared<stella_vslam_ros::stereo>(cfg, node);
    }
    else if (cfg->camera_->setup_type_ == stella_vslam::camera::setup_type_t::RGBD) {
        ros = std::make_shared<stella_vslam_ros::rgbd>(cfg, node);
    }
    else {
        auto error_str = "Invalid input type: " + cfg->camera_->get_setup_type_string();
        RCLCPP_ERROR(node->get_logger(), error_str.c_str());
        return EXIT_FAILURE;
    }

    auto initial_map = node->declare_parameter<std::string>(INITIAL_MAP_PARAM, "");
    if(initial_map != "") {
        ros->load_map_and_disable_mapping_on_restart(initial_map);
    } else {
        ros->set_slam_frame_transform_from_lookup();
        ros->SLAM_->startup();
        ros->SLAM_->enable_mapping_module();
    }

    rclcpp::Rate rate(50);
    while (rclcpp::ok()) {
        ros->exec_.spin_some();
        rate.sleep();
    }

    ros->SLAM_->shutdown();
    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
