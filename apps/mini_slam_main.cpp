#include "dataset.hpp"
#include "vio_plugin.hpp"
#include "pose.hpp"
#include <common/image_data.hpp>
#include <common/posedata.hpp>
#include <krisea_log/logger.hpp>
#include <utility>

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

    VioPlugin vio;
    Frame frame;
    Pose pose_viewer;

    vio.setPoseCallback(
        [&](const PoseData& pose_data) {
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
              pose_data.position.x(),
              pose_data.position.y(),
              pose_data.position.z(),
              pose_data.orientation.x(),
              pose_data.orientation.y(),
              pose_data.orientation.z(),
              pose_data.orientation.w());
        });

   while (dataset.next(frame))
   {
       ImageData rgb;
       rgb.timestamp = frame.timestamp;
       rgb.image = std::move(frame.rgb);

       DepthImageData depth;
       depth.timestamp = frame.timestamp;
       depth.image = std::move(frame.depth);

       vio.inputImage(std::move(rgb));
       vio.inputDepth(std::move(depth));
       vio.process();
   }

    pose_viewer.pathShow();
    return 0;
}
