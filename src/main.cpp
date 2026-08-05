#include "dataset.hpp"
#include "VO.hpp"
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

    while(dataset.next(frame))
    {
        bool success = vo.processFrame(frame);
        if(success)
        {
            Eigen::Isometry3d pose = vo.pose();

            std::cout << "frame:" << frame.id << std::endl;
            std::cout << "position:" << pose.translation() << std::endl;
        }
    }

    


    return 0;
}
