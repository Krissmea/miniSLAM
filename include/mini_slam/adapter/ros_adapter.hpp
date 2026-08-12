#ifndef MINI_SLAM_ADAPTER_ROS_ADAPTER_HPP
#define MINI_SLAM_ADAPTER_ROS_ADAPTER_HPP

#include <common/image_data.hpp>
#include <common/imu_data.hpp>
#include <common/posedata.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <functional>
#include <string>

namespace mini_slam
{

class RosAdapter
{
public:
    using ImageCallback =
        std::function<void(ImageData, DepthImageData)>;
    using ImuCallback = std::function<void(ImuData)>;

    explicit RosAdapter(rclcpp::Node& node);

    void setImageCallback(ImageCallback callback);
    void setImuCallback(ImuCallback callback);
    void publishPose(const PoseData& pose_data);

private:
    using ImageMessage = sensor_msgs::msg::Image;
    using SyncPolicy =
        message_filters::sync_policies::ApproximateTime<ImageMessage, ImageMessage>;

    void imageCallback(
        const ImageMessage::ConstSharedPtr& rgb,
        const ImageMessage::ConstSharedPtr& depth);
    void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& imu);

    rclcpp::Node& node_;
    message_filters::Subscriber<ImageMessage> rgb_sub_;
    message_filters::Subscriber<ImageMessage> depth_sub_;
    message_filters::Synchronizer<SyncPolicy> sync_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    nav_msgs::msg::Path path_;
    ImageCallback image_callback_;
    ImuCallback imu_callback_;
};

}  // namespace mini_slam

#endif
