#include "dataset.hpp"
#include "VO.hpp"
#include "pose.hpp"
#include <common/posedata.hpp>
#include <iostream>
#include <string>
#include <krisea_log/logger.hpp>
#include <fstream>
#include <iomanip>


int main(int argc, char** argv)
{
    KR_LOG_INIT(
    "mini_slam",                // 日志器名称
    "logs/mini_slam.log",       // 日志文件
    20 * 1024 * 1024,           // 每个文件最大 20 MB
    5);                         // 最多保留 5 个轮转文件);
    KR_SET_LOG_LEVEL("debug");      // 输出 debug 及以上级别

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
        KR_ERROR(
          "Failed to create trajectory file: {}",
          trajectory_path);

        return 1;
    }

    trajectory_file << std::fixed << std::setprecision(9);

    KR_INFO("Estimated trajectory will be saved to: {}", trajectory_path);

    VO vo;

    Frame frame;
    Pose poseviewer;

    while(dataset.next(frame))
    {
        bool success = vo.processFrame(frame);
        // KR_INFO("processframe:{}",success);
        if(success)
        {
            const Eigen::Isometry3d& pose = vo.pose();
            PoseData pD;
            pD.timestamp = frame.timestamp;
            pD.position = pose.translation();
            pD.orientation = Eigen::Quaterniond(pose.rotation()).normalized();
            
            poseviewer.posePath(pD);

            trajectory_file
                << pD.timestamp << " "
                << pD.position.x() << " "
                << pD.position.y() << " "
                << pD.position.z() << " "
                << pD.orientation.x() << " "
                << pD.orientation.y() << " "
                << pD.orientation.z() << " "
                << pD.orientation.w()
                << '\n';

            
            
            KR_INFO("frame={} position=[{:.6f}, {:.6f}, {:.6f}] " "quaternion=[{:.6f}, {:.6f}, {:.6f}, {:.6f}]",
                frame.id,
                pD.position.x(),
                pD.position.y(),
                pD.position.z(),
                pD.orientation.x(),
                pD.orientation.y(),
                pD.orientation.z(),
                pD.orientation.w());




            
        }
    }

    poseviewer.pathShow();

    return 0;
}
