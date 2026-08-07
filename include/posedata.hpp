#ifndef POSEDATA_H
#define POSEDATA_H

#include <Eigen/Core>
#include <Eigen/Geometry>


struct PoseData
{
    double timestamp;
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
};

#endif // POSE_Ha   