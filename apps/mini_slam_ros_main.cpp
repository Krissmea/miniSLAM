#include "VO.hpp"
#include "config.hpp"

#include <common/frame.hpp>
#include <common/image_data.hpp>
#include <common/imu_data.hpp>
#include <common/posedata.hpp>
#include <krisea_log/logger.hpp>
#include <mini_slam/adapter/ros_adapter.hpp>

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <stdexcept>
#include <string>

class MiniSlamRosNode final : public rclcpp::Node
{
public:
    MiniSlamRosNode()
        : Node("mini_slam"), adapter_(*this)
    {
        const auto config_path = declare_parameter<std::string>(
            "config_path", MINI_SLAM_DEFAULT_CONFIG_PATH);
        world_frame_ = declare_parameter<std::string>("world_frame", "map");
        if (!loadConfig(config_path))
        {
            throw std::runtime_error("Failed to load config: " + config_path);
        }

        adapter_.setImageCallback(
            [this](ImageData rgb, DepthImageData depth) {
                onImages(std::move(rgb), std::move(depth));
            });

        // Deliberately empty for now. ROS Imu has already become ImuData here.
        adapter_.setImuCallback([](ImuData /* imu */) {
            // TODO: forward to the future VIO algorithm interface.
        });

        KR_INFO("miniSLAM ROS node ready; algorithm only receives internal data");
    }

private:
    void onImages(ImageData rgb, DepthImageData depth)
    {
        if (depth.image.type() != CV_16UC1)
        {
            KR_ERROR("Unsupported depth type: {}", depth.image.type());
            return;
        }

        Frame frame;
        frame.id = next_frame_id_++;
        frame.timestamp = rgb.timestamp;
        frame.rgb = std::move(rgb.image);
        frame.depth = std::move(depth.image);
        if (!vo_.processFrame(frame))
        {
            return;
        }

        PoseData pose;
        pose.timestamp = frame.timestamp;
        pose.frame_id = world_frame_;
        pose.position = vo_.pose().translation();
        pose.orientation =
            Eigen::Quaterniond(vo_.pose().rotation()).normalized();
        adapter_.publishPose(pose);
    }

    mini_slam::RosAdapter adapter_;
    VO vo_;
    std::string world_frame_;
    int next_frame_id_{0};
};

int main(int argc, char** argv)
{
    KR_LOG_INIT("mini_slam_ros", "logs/mini_slam_ros.log", 20 * 1024 * 1024, 5);
    KR_SET_LOG_LEVEL("info");
    rclcpp::init(argc, argv);
    try
    {
        rclcpp::spin(std::make_shared<MiniSlamRosNode>());
    }
    catch (const std::exception& error)
    {
        KR_CRITICAL("ROS node failed: {}", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
