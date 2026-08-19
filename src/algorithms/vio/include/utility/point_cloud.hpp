#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include <common/image_data.hpp>
#include "config.hpp"
#include <Eigen/Core>
#include <vector>

class PointCloud
{
public:
    // explicit PointCloud(Camera& camera);
    std::vector<Eigen::Vector3d> generate(DepthImageData& depth, double depth_scale);
    Eigen::Vector3d pixel2camera(double u, double v, double depth);

private:
    
};



#endif
