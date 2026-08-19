#include <mini_slam/adapter/ros_adapter.hpp>

#include <mini_slam/adapter/ros_converter.hpp>
#include <krisea_log/logger.hpp>

#include <utility>

namespace mini_slam
{

RosAdapter::RosAdapter(rclcpp::Node& node)
    : node_(node)
{
    const auto rgb_topic = node_.declare_parameter<std::string>(
        "rgb_topic", "/camera/color/image_raw");
    const auto depth_topic = node_.declare_parameter<std::string>(
        "depth_topic", "/camera/depth/image_raw");
    const auto imu_topic = node_.declare_parameter<std::string>(
        "imu_topic", "/camera/gyro_accel/sample");

    pose_pub_ = node_.create_publisher<geometry_msgs::msg::PoseStamped>(
        "~/pose", 10);
    path_pub_ = node_.create_publisher<nav_msgs::msg::Path>("~/path", 10);

    rgb_sub_ = node_.create_subscription<ImageMessage>(
        rgb_topic, rclcpp::SensorDataQoS(),
        std::bind(&RosAdapter::rgbCallback, this, std::placeholders::_1));
    depth_sub_ = node_.create_subscription<ImageMessage>(
        depth_topic, rclcpp::SensorDataQoS(),
        std::bind(&RosAdapter::depthCallback, this, std::placeholders::_1));
    imu_sub_ = node_.create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, rclcpp::SensorDataQoS(),
        std::bind(&RosAdapter::imuCallback, this, std::placeholders::_1));

    KR_INFO(
        "ROS adapter ready: rgb={} depth={} imu={}",
        rgb_topic, depth_topic, imu_topic);
}

void RosAdapter::setRgbCallback(RgbCallback callback)
{
    rgb_callback_ = std::move(callback);
}

void RosAdapter::setDepthCallback(DepthCallback callback)
{
    depth_callback_ = std::move(callback);
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



void RosAdapter::rgbCallback(const ImageMessage::ConstSharedPtr& rgb)
{
    auto rgb_data = converter::fromRosImage(*rgb);
    if (!rgb_data)
    {
        KR_ERROR(
            "ROS RGB conversion failed: encoding={}",
            rgb->encoding);
        return;
    }

    if (rgb_callback_)
    {
        rgb_callback_(std::move(*rgb_data));
    }
}

void RosAdapter::depthCallback(const ImageMessage::ConstSharedPtr& depth)
{
    auto depth_data = converter::fromRosDepth(*depth);
    if (!depth_data)
    {
        KR_ERROR(
            "ROS depth conversion failed: encoding={}", depth->encoding);
        return;
    }

    if (depth_callback_)
    {
        depth_callback_(std::move(*depth_data));
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
