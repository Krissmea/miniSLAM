#include "dataset.hpp"
#include "VO.hpp"
#include "pose.hpp"
#include "posedata.hpp"
#include <iostream>
#include <string>



int main(int argc, char** argv)
{
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
        if(success)
        {
            PoseData pD;
            Eigen::Isometry3d pose = vo.pose();
            pD.timestamp = frame.timestamp;
            pD.position = pose.translation();
            
            poseviewer.posePath(pD);
            poseviewer.pathShow(pD);
            

            std::cout << "frame:" << frame.id << std::endl;
            std::cout << "position:" << pose.translation() << std::endl;
        }
    }

    


    return 0;
}
