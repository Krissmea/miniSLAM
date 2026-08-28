#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include <common/frame.hpp>
#include <common/imu_data.hpp>

#include "feature_tracker.hpp"

#include <Eigen/Geometry>

#include <deque>
#include <optional>
#include <unordered_map>
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
    //参考帧 reference参考
    struct RefFrame
    {
        Frame frame;
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        std::unordered_map<int, cv::Point3f> points3d;
        std::unordered_map<int, cv::Point2f> pixels;
        bool valid{false};
    };

    struct PnPResult
    {
        bool success{false};
        std::string failure_reason{"not_run"};
        Eigen::Isometry3d T_ref_curr = Eigen::Isometry3d::Identity();
        int matches{0};
        int inliers{0};
        double inlier_ratio{0.0};
        double med_reprojection_error{0.0};
    };

    bool solveFrame(Frame& frame, const std::vector<ImuData>& imu_samples);
    bool imuCoversFrame(double frame_timestamp) const;
    std::vector<ImuData> collectImuForFrame(double frame_timestamp) const;
    void pruneImuBefore(double frame_timestamp);

    void updateReference(RefFrame& reference, Frame& frame, const Eigen::Isometry3d& pose);
    bool referenceUsable(const RefFrame& reference) const;
    PnPResult solvePnPFromReference(const RefFrame& reference);
    bool validCandidatePose(const Eigen::Isometry3d& candidate_pose,  double current_timestamp) const;
    double calculateParallax(std::size_t& common_feature_count) const;
    bool isKeyFrame(double median_parallax, std::size_t common_feature_count, bool keyframe_pnp_success) const;

    FeatureTracker tracker_;
    // Layer 1: previous input frame. Used only for continuous KLT.
    Frame last_frame_;
    // Layer 2: most recent frame with a trusted visual pose.
    RefFrame last_pose_reference_;
    // Layer 3: long-lived keyframe used as primary PnP reference.
    RefFrame keyframe_reference_;
    Eigen::Isometry3d pose_ = Eigen::Isometry3d::Identity();

    bool first_frame_{true};
    bool imu_stream_seen_{false};
    double last_processed_frame_timestamp_{-1.0};

    int frames_since_keyframe_{0};
    int keyframe_id_{0};
    int consecutive_pnp_failures_{0};

    std::deque<Frame> frame_buf_;
    std::deque<ImuData> imu_buf_;

    std::vector<cv::Point3f> pts3d_reference_;
    std::vector<cv::Point2f> pts2d_current_;


    // 后续加入 IMU 融合时再增加
    // Eigen::Vector3d velocity_;
    // Eigen::Vector3d accel_bias_;
    // Eigen::Vector3d gyro_bias_;
    // Eigen::Vector3d gravity_;


};


#endif
