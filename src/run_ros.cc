#include <stella_vslam/config.h>
#include <stella_vslam/system.h>
#include <stella_vslam_ros.h>


#define CONFIG_PARAM "config"
#define INITIAL_MAP_PARAM "map"


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node_ = std::make_shared<rclcpp::Node>("vslam");
    auto cfg = std::make_shared<stella_vslam::config>(
        node_->declare_parameter<std::string>(CONFIG_PARAM, "")
    );

    std::shared_ptr<stella_vslam_ros::system> ros;
    if (cfg->camera_->setup_type_ == stella_vslam::camera::setup_type_t::Monocular) {
        ros = std::make_shared<stella_vslam_ros::mono>(cfg, node_);
    }
    else if (cfg->camera_->setup_type_ == stella_vslam::camera::setup_type_t::Stereo) {
        ros = std::make_shared<stella_vslam_ros::stereo>(cfg, node_);
    }
    else if (cfg->camera_->setup_type_ == stella_vslam::camera::setup_type_t::RGBD) {
        ros = std::make_shared<stella_vslam_ros::rgbd>(cfg, node_);
    }
    else {
        auto error_str = "Invalid input type: " + cfg->camera_->get_setup_type_string();
        RCLCPP_ERROR(node_->get_logger(), error_str.c_str());
        return EXIT_FAILURE;
    }

    auto slam = ros->SLAM_;
    auto initial_map = node_->declare_parameter<std::string>(INITIAL_MAP_PARAM, "");
    if(initial_map != "") {
        ros->load_map_and_disable_mapping_on_restart(initial_map);
    } else {
        slam->startup();
        slam->enable_mapping_module();
    }

    rclcpp::Rate rate(50);
    while (rclcpp::ok()) {
        ros->exec_.spin_some();
        rate.sleep();
    }

    slam->shutdown();
    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
