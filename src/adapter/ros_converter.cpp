#include <mini_slam/adapter/ros_converter.hpp>

#include <opencv2/imgproc.hpp>

#include <algorithm>

namespace mini_slam::converter
{
namespace
{

double timestamp(const builtin_interfaces::msg::Time& stamp)
{
    return static_cast<double>(stamp.sec) +
           static_cast<double>(stamp.nanosec) * 1e-9;
}

std::optional<int> imageType(const std::string& encoding)
{
    if (encoding == "mono8") return CV_8UC1;
    if (encoding == "bgr8" || encoding == "rgb8") return CV_8UC3;
    if (encoding == "bgra8" || encoding == "rgba8") return CV_8UC4;
    if (encoding == "mono16" || encoding == "16UC1") return CV_16UC1;
    if (encoding == "32FC1") return CV_32FC1;
    return std::nullopt;
}

}  // namespace

std::optional<ImageData> fromRosImage(
    const sensor_msgs::msg::Image& message)
{
    const auto type = imageType(message.encoding);
    if (!type)
    {
        return std::nullopt;
    }

    ImageData data;
    data.timestamp = timestamp(message.header.stamp);
    data.frame_id = message.header.frame_id;
    const cv::Mat view(
        static_cast<int>(message.height), static_cast<int>(message.width), *type,
        const_cast<unsigned char*>(message.data.data()), message.step);

    if (message.encoding == "rgb8")
    {
        cv::cvtColor(view, data.image, cv::COLOR_RGB2BGR);
    }
    else if (message.encoding == "rgba8")
    {
        cv::cvtColor(view, data.image, cv::COLOR_RGBA2BGR);
    }
    else if (message.encoding == "bgra8")
    {
        cv::cvtColor(view, data.image, cv::COLOR_BGRA2BGR);
    }
    else
    {
        data.image = view.clone();
    }
    if (!data.valid())
    {
        return std::nullopt;
    }
    return data;
}

std::optional<DepthImageData> fromRosDepth(
    const sensor_msgs::msg::Image& message)
{
    if (message.encoding != "16UC1" && message.encoding != "mono16" &&
        message.encoding != "32FC1")
    {
        return std::nullopt;
    }
    const int type = message.encoding == "32FC1" ? CV_32FC1 : CV_16UC1;

    DepthImageData data;
    data.timestamp = timestamp(message.header.stamp);
    data.frame_id = message.header.frame_id;
    const cv::Mat view(
        static_cast<int>(message.height), static_cast<int>(message.width), type,
        const_cast<unsigned char*>(message.data.data()), message.step);
    data.image = view.clone();
    if (!data.valid())
    {
        return std::nullopt;
    }
    return data;
}

ImuData fromRos(const sensor_msgs::msg::Imu& message)
{
    ImuData data;
    data.timestamp = timestamp(message.header.stamp);
    data.frame_id = message.header.frame_id;
    data.angular_velocity = {
        message.angular_velocity.x,
        message.angular_velocity.y,
        message.angular_velocity.z};
    data.linear_acceleration = {
        message.linear_acceleration.x,
        message.linear_acceleration.y,
        message.linear_acceleration.z};
    std::copy(message.orientation_covariance.begin(),
              message.orientation_covariance.end(),
              data.orientation_covariance.begin());
    std::copy(message.angular_velocity_covariance.begin(),
              message.angular_velocity_covariance.end(),
              data.angular_velocity_covariance.begin());
    std::copy(message.linear_acceleration_covariance.begin(),
              message.linear_acceleration_covariance.end(),
              data.linear_acceleration_covariance.begin());

    if (message.orientation_covariance[0] >= 0.0)
    {
        data.orientation = Eigen::Quaterniond(
            message.orientation.w,
            message.orientation.x,
            message.orientation.y,
            message.orientation.z).normalized();
        data.has_orientation = true;
    }
    return data;
}

geometry_msgs::msg::PoseStamped toRos(const PoseData& data)
{
    geometry_msgs::msg::PoseStamped message;
    const auto seconds = static_cast<std::int32_t>(data.timestamp);
    message.header.stamp.sec = seconds;
    message.header.stamp.nanosec = static_cast<std::uint32_t>(
        (data.timestamp - static_cast<double>(seconds)) * 1e9);
    message.header.frame_id = data.frame_id;
    message.pose.position.x = data.position.x();
    message.pose.position.y = data.position.y();
    message.pose.position.z = data.position.z();
    message.pose.orientation.x = data.orientation.x();
    message.pose.orientation.y = data.orientation.y();
    message.pose.orientation.z = data.orientation.z();
    message.pose.orientation.w = data.orientation.w();
    return message;
}

}  // namespace mini_slam::converter
