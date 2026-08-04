#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include "frame.hpp"
#include "camera.hpp"

class PointCloud
{
public:
    void generate(Frame& frame, Camera& camera, double depth_scale);
};



#endif