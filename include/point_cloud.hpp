#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include <common/frame.hpp>
#include "config.hpp"

class PointCloud
{
public:
    // explicit PointCloud(Camera& camera);
    void generate(Frame& frame, double depth_scale);
    Eigen::Vector3d pixel2camera(double u, double v, double depth);

private:
    
};



#endif
