#include "point_cloud.hpp"

void PointCloud::generate(Frame& frame, Camera& camera, double depth_scale)
{
    frame.points3d.clear();

    for (int v = 0; v < frame.depth.rows; v++)
    {
        for (int u = 0; u < frame.depth.cols; u++)
        {
            uint16_t d = frame.depth.at<uint16_t>(v,u);
            if (d == 0)
                continue;
            
            double depth_m = d / depth_scale;

            Eigen::Vector3d p = camera.pixel2camera(u, v, depth_m);
            frame.points3d.push_back(p);
        }
    }
}

