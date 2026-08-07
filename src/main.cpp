#include "dataset.hpp"
#include "VO.hpp"
#include "pose.hpp"
#include <common/posedata.hpp>
#include <iostream>
#include <string>
#include <krisea_log/logger.hpp>


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

    VO vo;

    Frame frame;
    Pose poseviewer;

    while(dataset.next(frame))
    {
        bool success = vo.processFrame(frame);
        KR_INFO("processframe:{}",success);
        if(success)
        {
            PoseData pD;
            Eigen::Isometry3d pose = vo.pose();
            pD.timestamp = frame.timestamp;
            pD.position = pose.translation();
            
            poseviewer.posePath(pD);
            
            
            KR_INFO("frame:{}",frame.id);
            KR_INFO("position:{}",pose.translation().x());



            std::cout << "frame:" << frame.id << std::endl;
            std::cout << "position:" << pose.translation() << std::endl;
        }
    }

    poseviewer.pathShow();

    return 0;
}
