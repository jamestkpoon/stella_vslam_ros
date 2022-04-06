#ifdef USE_PANGOLIN_VIEWER
#include <pangolin_viewer/viewer.h>
#elif USE_SOCKET_PUBLISHER
#include <socket_publisher/publisher.h>
#endif

#include <stella_vslam/system.h>
#include <stella_vslam/config.h>
#include <stella_vslam/util/yaml.h>
#include <stella_vslam_ros.h>

#include <iostream>
#include <chrono>
#include <fstream>
#include <numeric>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <spdlog/spdlog.h>

#ifdef USE_STACK_TRACE_LOGGER
#include <glog/logging.h>
#endif

#ifdef USE_GOOGLE_PERFTOOLS
#include <gperftools/profiler.h>
#endif


#define CONFIG_PARAM "config"
#define DEBUG_PARAM "debug"
#define EVAL_LOG_PARAM "eval_log"


int main(int argc, char* argv[]) {
#ifdef USE_STACK_TRACE_LOGGER
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
#endif

    rclcpp::init(argc, argv);

    std::shared_ptr<stella_vslam_ros::system> ros;

    /*
    if (cfg->camera_->setup_type_ == stella_vslam::camera::setup_type_t::Monocular) {
        ros = std::make_shared<stella_vslam_ros::mono>(cfg, vocab_file_path, mask_img_path);
    }
    else if (cfg->camera_->setup_type_ == stella_vslam::camera::setup_type_t::Stereo) {
        ros = std::make_shared<stella_vslam_ros::stereo>(cfg, vocab_file_path, mask_img_path, rectify);
    }
    else if (cfg->camera_->setup_type_ == stella_vslam::camera::setup_type_t::RGBD) {
        ros = std::make_shared<stella_vslam_ros::rgbd>(cfg, vocab_file_path, mask_img_path);
    }
    else {
        throw std::runtime_error("Invalid setup type: " + cfg->camera_->get_setup_type_string());
    }*/

    

#ifdef USE_GOOGLE_PERFTOOLS
    ProfilerStop();
#endif

    return EXIT_SUCCESS;
}
