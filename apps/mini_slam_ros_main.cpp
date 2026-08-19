#include "vio_plugin.hpp"
#include "config.hpp"

#include <common/image_data.hpp>
#include <common/imu_data.hpp>
#include <common/posedata.hpp>
#include <krisea_log/logger.hpp>
#include <mini_slam/adapter/ros_adapter.hpp>

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <utility>

int main(int argc, char** argv)
{
    KR_LOG_INIT("mini_slam_ros", "logs/mini_slam_ros.log", 20 * 1024 * 1024, 5);
    KR_SET_LOG_LEVEL("info");

    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("mini_slam");
    const auto config_path = node->declare_parameter<std::string>(
        "config_path", MINI_SLAM_DEFAULT_CONFIG_PATH);
    const auto world_frame = node->declare_parameter<std::string>(
        "world_frame", "map");

    if (!loadConfig(config_path))
    {
        KR_CRITICAL("Failed to load config: {}", config_path);
        rclcpp::shutdown();
        return 1;
    }

    mini_slam::RosAdapter adapter(*node);
    VioPlugin vio;

    vio.setPoseCallback(
        [&adapter, &world_frame](const PoseData& estimated_pose) {
          PoseData output_pose = estimated_pose;
          output_pose.frame_id = world_frame;
          adapter.publishPose(output_pose);
        });
    
    adapter.setRgbCallback([&vio](ImageData rgb) {
        vio.inputImage(std::move(rgb));
        vio.process();
    });
    adapter.setDepthCallback([&vio](DepthImageData depth) {
        vio.inputDepth(std::move(depth));
        vio.process();
    });
    adapter.setImuCallback([&vio](ImuData imu) {
        vio.inputImu(std::move(imu));
        vio.process();
    });

    KR_INFO("miniSLAM ROS node ready");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
