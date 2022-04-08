#include <stella_vslam/config.h>
#include <stella_vslam/system.h>
#include <stella_vslam_ros.h>
#include <yaml-cpp/yaml.h>


#define CONFIG_PARAM "config"
#define INITIAL_MAP_PARAM "map"
#define INITIAL_POSE_PARAM "initial_cam_pose"


geometry_msgs::msg::Transform transform_msg_from_yaml(const std::string& filepath) {
    auto yaml = YAML::LoadFile(filepath);

    geometry_msgs::msg::Transform msg;
    msg.translation.x = yaml["translation"]["x"].as<double>();
    msg.translation.y = yaml["translation"]["y"].as<double>();
    msg.translation.z = yaml["translation"]["z"].as<double>();
    msg.rotation.x = yaml["rotation"]["x"].as<double>();
    msg.rotation.y = yaml["rotation"]["y"].as<double>();
    msg.rotation.z = yaml["rotation"]["z"].as<double>();
    msg.rotation.w = yaml["rotation"]["w"].as<double>();

    return msg;
}


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("vslam");
    auto logger = node->get_logger();
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
        RCLCPP_ERROR(logger, error_str.c_str());
        return EXIT_FAILURE;
    }

    auto slam = ros->SLAM_;
    auto initial_map = node->declare_parameter<std::string>(INITIAL_MAP_PARAM, "");
    if(initial_map != "") {
        ros->load_map_and_disable_mapping_on_restart(initial_map);
    } else {
        slam->startup();
        slam->enable_mapping_module();
    }

    auto initial_cam_pose_fp = node->declare_parameter<std::string>(INITIAL_POSE_PARAM, "");
    if(initial_cam_pose_fp != "") {
        ros->init_pose_direct_callback(
            std::make_shared<geometry_msgs::msg::Transform>(
                transform_msg_from_yaml(initial_cam_pose_fp)
            )
        );
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
