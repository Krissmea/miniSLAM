#ifndef MINI_SLAM_ADAPTER_ROS_CONVERTER_HPP
#define MINI_SLAM_ADAPTER_ROS_CONVERTER_HPP

#include <common/image_data.hpp>
#include <common/imu_data.hpp>
#include <common/posedata.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <optional>

namespace mini_slam::converter
{

// ROS -> project-owned data types.
std::optional<ImageData> fromRosImage(const sensor_msgs::msg::Image& message);

std::optional<DepthImageData> fromRosDepth(const sensor_msgs::msg::Image& message);

ImuData fromRos(const sensor_msgs::msg::Imu& message);

// Project-owned result -> ROS.
geometry_msgs::msg::PoseStamped toRos(const PoseData& data);

}  // namespace mini_slam::converter

#endif
