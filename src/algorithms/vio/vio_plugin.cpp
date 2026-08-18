#include <iostream>
#include "vio_plugin.hpp"
#include <krisea_log/logger.hpp>
 #include <utility>



void VioPlugin::inputFrame(Frame& frame)
{
    if (!estimator_.processFrame(frame))
    {
        return;
    }

    const Eigen::Isometry3d& estimated_pose = estimator_.pose();

    PoseData pose_data;
    pose_data.timestamp = frame.timestamp;
    pose_data.position = estimated_pose.translation();
    pose_data.orientation =
        Eigen::Quaterniond(estimated_pose.rotation()).normalized();

    if (pose_callback_)
    {
        pose_callback_(pose_data);
    }

}

void VioPlugin::inputImu(const ImuData& imu)
{
    estimator_.processImu(imu);

}

void VioPlugin::setPoseCallback(PoseCallback callback)
{
    pose_callback_ = std::move(callback);
}

void VioPlugin::reset()
{
    estimator_.reset();
}