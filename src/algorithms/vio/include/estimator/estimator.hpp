#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include <common/frame.hpp>
#include <common/imu_data.hpp>
#include "feature_tracker.hpp"
#include <Eigen/Geometry>



class Estimator
{
public:
    bool processFrame(Frame& frame);
    const Eigen::Isometry3d& pose() const;

    void processImu(const ImuData& imu);
    void reset();

private:
    FeatureTracker tracker_;
    Frame frame_;
    Eigen::Isometry3d pose_ = Eigen::Isometry3d::Identity();

    bool first_frame_{true};

    std::vector<cv::Point3f> pts3d_last_;
    std::vector<cv::Point2f> pts2d_curr_;


    // 后续加入 IMU 融合时再增加
    // Eigen::Vector3d velocity_;
    // Eigen::Vector3d accel_bias_;
    // Eigen::Vector3d gyro_bias_;
    // Eigen::Vector3d gravity_;

    // ImuPreintegration preintegration_;

};


#endif
