#ifndef POSE_H
#define POSE_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "posedata.hpp"

class Pose
{
public:
    Pose() : timestamp(0.0), position(Eigen::Vector3d::Zero()), orientation(Eigen::Quaterniond::Identity()) {}
    Pose(double ts, const Eigen::Vector3d& pos, const Eigen::Quaterniond& ori)
        : timestamp(ts), position(pos), orientation(ori) {}

    double timestamp;
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;

    bool posePath(const PoseData& poseData);
    void pathShow();

private:
    std::vector<Eigen::Vector3d> position_;
    std::vector<Eigen::Quaterniond> orientation_;
};




#endif // POSE_H