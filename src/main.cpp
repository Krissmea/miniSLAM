#include "dataset.hpp"
#include "camera.hpp"
#include "config.hpp"
#include "point_cloud.hpp"
#include "VO.hpp"
#include <iostream>





int main()
{
    Config config;
    config.load("/home/liuxz/workspace/CppProjects/miniSLAM/config/camera.yaml");
    
    Dataset dataset(config.datasetPath());

    Camera camera(config.fx(), config.fy(), config.cx(), config.cy());

    VO vo(camera, config);

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