#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include <common/frame.hpp>
#include <common/imu_data.hpp>

#include "feature_tracker.hpp"

#include <Eigen/Geometry>

#include <deque>
#include <optional>
#include <vector>
 
struct EstimatorResult
{
    double timestamp{0.0};
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
};
 


class Estimator
{
public:
    // Input functions only enqueue measurements.
    void inputFrame(Frame frame);
    void inputImu(ImuData imu);

    // Process one valid output. Returns nullopt if more data is required.
    std::optional<EstimatorResult> process();
 
    const Eigen::Isometry3d& pose() const;
    void reset();

private:
    bool solveFrame(Frame& frame, const std::vector<ImuData>& imu_samples);
    bool imuCoversFrame(double frame_timestamp) const;
    std::vector<ImuData> collectImuForFrame(double frame_timestamp) const;
    void pruneImuBefore(double frame_timestamp);

    FeatureTracker tracker_;
    Frame frame_;
    Eigen::Isometry3d pose_ = Eigen::Isometry3d::Identity();

    bool first_frame_{true};
    bool imu_stream_seen_{false};
    double last_processed_frame_timestamp_{-1.0};

    std::deque<Frame> frame_buf_;
    std::deque<ImuData> imu_buf_;

    std::vector<cv::Point3f> pts3d_last_;
    std::vector<cv::Point2f> pts2d_curr_;


    // 后续加入 IMU 融合时再增加
    // Eigen::Vector3d velocity_;
    // Eigen::Vector3d accel_bias_;
    // Eigen::Vector3d gyro_bias_;
    // Eigen::Vector3d gravity_;


};


#endif
