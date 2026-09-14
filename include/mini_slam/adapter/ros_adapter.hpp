#ifndef MINI_SLAM_ADAPTER_ROS_ADAPTER_HPP
#define MINI_SLAM_ADAPTER_ROS_ADAPTER_HPP

#include <common/image_data.hpp>
#include <common/imu_data.hpp>
#include <common/posedata.hpp>


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
    using ImageCallback = std::function<void(ImageData)>;
    using RgbCallback = ImageCallback;
    using DepthCallback = std::function<void(DepthImageData)>;
    using ImuCallback = std::function<void(ImuData)>;

    explicit RosAdapter(
        rclcpp::Node& node,
        bool stereo_mode,
        const std::string& rgb_topic,
        const std::string& depth_topic,
        const std::string& left_topic,
        const std::string& right_topic,
        const std::string& imu_topic);

    void setRgbCallback(RgbCallback callback);
    void setDepthCallback(DepthCallback callback);
    void setLeftCallback(ImageCallback callback);
    void setRightCallback(ImageCallback callback);
    void setImuCallback(ImuCallback callback);
    void publishPose(const PoseData& pose_data);

private:
    using ImageMessage = sensor_msgs::msg::Image;
    
    void rgbCallback(const ImageMessage::ConstSharedPtr& rgb);
    void depthCallback(const ImageMessage::ConstSharedPtr& depth);
    void leftCallback(const ImageMessage::ConstSharedPtr& left);
    void rightCallback(const ImageMessage::ConstSharedPtr& right);
    void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& imu);

    rclcpp::Node& node_;
    rclcpp::Subscription<ImageMessage>::SharedPtr rgb_sub_;
    rclcpp::Subscription<ImageMessage>::SharedPtr depth_sub_;
    rclcpp::Subscription<ImageMessage>::SharedPtr left_sub_;
    rclcpp::Subscription<ImageMessage>::SharedPtr right_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    nav_msgs::msg::Path path_;

    RgbCallback rgb_callback_;
    DepthCallback depth_callback_;
    ImageCallback left_callback_;
    ImageCallback right_callback_;
    ImuCallback imu_callback_;
};

}  // namespace mini_slam

#endif
