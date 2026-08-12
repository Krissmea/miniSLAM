#include <mini_slam/adapter/ros_adapter.hpp>

#include <mini_slam/adapter/ros_converter.hpp>
#include <krisea_log/logger.hpp>

#include <utility>

namespace mini_slam
{

RosAdapter::RosAdapter(rclcpp::Node& node)
    : node_(node),
      rgb_sub_(&node_, node_.declare_parameter(
          "rgb_topic", "/camera/color/image_raw"),
          rmw_qos_profile_sensor_data),
      depth_sub_(&node_, node_.declare_parameter(
          "depth_topic", "/camera/depth/image_raw"),
          rmw_qos_profile_sensor_data),
      sync_(SyncPolicy(10), rgb_sub_, depth_sub_)
{
    const auto imu_topic = node_.declare_parameter<std::string>(
        "imu_topic", "/camera/gyro_accel/sample");
    pose_pub_ = node_.create_publisher<geometry_msgs::msg::PoseStamped>(
        "~/pose", 10);
    path_pub_ = node_.create_publisher<nav_msgs::msg::Path>("~/path", 10);
    imu_sub_ = node_.create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, rclcpp::SensorDataQoS(),
        std::bind(&RosAdapter::imuCallback, this, std::placeholders::_1));
    sync_.setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.03));
    sync_.registerCallback(std::bind(
        &RosAdapter::imageCallback, this,
        std::placeholders::_1, std::placeholders::_2));

    KR_INFO(
        "ROS adapter ready: rgb={} depth={} imu={}",
        rgb_sub_.getTopic(), depth_sub_.getTopic(), imu_topic);
}

void RosAdapter::setImageCallback(ImageCallback callback)
{
    image_callback_ = std::move(callback);
}

void RosAdapter::setImuCallback(ImuCallback callback)
{
    imu_callback_ = std::move(callback);
}

void RosAdapter::publishPose(const PoseData& pose_data)
{
    auto pose = converter::toRos(pose_data);
    pose_pub_->publish(pose);
    path_.header = pose.header;
    path_.poses.push_back(pose);
    path_pub_->publish(path_);
}

void RosAdapter::imageCallback(
    const ImageMessage::ConstSharedPtr& rgb,
    const ImageMessage::ConstSharedPtr& depth)
{
    auto rgb_data = converter::fromRosImage(*rgb);
    auto depth_data = converter::fromRosDepth(*depth);
    if (!rgb_data || !depth_data)
    {
        KR_ERROR(
            "ROS image conversion failed: rgb_valid={} depth_valid={} rgb_encoding={} depth_encoding={}",
            rgb_data.has_value(), depth_data.has_value(),
            rgb->encoding, depth->encoding);
        return;
    }

    if (image_callback_)
    {
        image_callback_(std::move(*rgb_data), std::move(*depth_data));
    }
}

void RosAdapter::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& imu)
{
    if (imu_callback_)
    {
        imu_callback_(converter::fromRos(*imu));
    }
}

}  // namespace mini_slam
