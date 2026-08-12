#include "dataset.hpp"
#include "VO.hpp"
#include "pose.hpp"

#include <common/posedata.hpp>
#include <krisea_log/logger.hpp>

#include <fstream>
#include <iomanip>
#include <string>

int main(int argc, char** argv)
{
    KR_LOG_INIT("mini_slam", "logs/mini_slam.log", 20 * 1024 * 1024, 5);
    KR_SET_LOG_LEVEL("debug");

    std::string config_path = MINI_SLAM_DEFAULT_CONFIG_PATH;
    if (argc > 1)
    {
        config_path = argv[1];
    }

    if (!loadConfig(config_path))
    {
        return 1;
    }

    Dataset dataset(g_dataset_path);
    const std::string trajectory_path = "estimated_trajectory.txt";
    std::ofstream trajectory_file(trajectory_path);
    if (!trajectory_file.is_open())
    {
        KR_ERROR("Failed to create trajectory file: {}", trajectory_path);
        return 1;
    }

    trajectory_file << std::fixed << std::setprecision(9);
    KR_INFO("Estimated trajectory will be saved to: {}", trajectory_path);

    VO vo;
    Frame frame;
    Pose pose_viewer;

    while (dataset.next(frame))
    {
        if (!vo.processFrame(frame))
        {
            continue;
        }

        const Eigen::Isometry3d& pose = vo.pose();
        PoseData pose_data;
        pose_data.timestamp = frame.timestamp;
        pose_data.position = pose.translation();
        pose_data.orientation = Eigen::Quaterniond(pose.rotation()).normalized();
        pose_viewer.posePath(pose_data);

        trajectory_file
            << pose_data.timestamp << " "
            << pose_data.position.x() << " "
            << pose_data.position.y() << " "
            << pose_data.position.z() << " "
            << pose_data.orientation.x() << " "
            << pose_data.orientation.y() << " "
            << pose_data.orientation.z() << " "
            << pose_data.orientation.w() << '\n';

        KR_INFO(
            "frame={} position=[{:.6f}, {:.6f}, {:.6f}] "
            "quaternion=[{:.6f}, {:.6f}, {:.6f}, {:.6f}]",
            frame.id,
            pose_data.position.x(), pose_data.position.y(), pose_data.position.z(),
            pose_data.orientation.x(), pose_data.orientation.y(),
            pose_data.orientation.z(), pose_data.orientation.w());
    }

    pose_viewer.pathShow();
    return 0;
}
