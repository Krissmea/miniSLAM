#ifndef VIO_PLUGIN_H
#define VIO_PLUGIN_H

#include <Eigen/Geometry>
#include <common/frame.hpp>
#include "feature_tracker.hpp"


class VioPlugin
{
public:
    VioPlugin();
    ~VioPlugin();

    using PoseCallback = std::function<void(const PoseData&)>;

    bool initialize(const std::string& comfig_path);
    void inputFrame(Fram& frame);
    void inputImu(ImuData& imu);
    void setPoseCallback(PoseCallback callback);
    void reset();

private:
    FeatureTracker tracker_;
    Estimator estimator_;
    PoseCallback pose_callback_;
    
}


class VO
{
public:
    bool processFrame(Frame& frame);
    const Eigen::Isometry3d& pose() const;

private:
    FeatureTracker tracker_;
    Frame frame_;
    Eigen::Isometry3d pose_ = Eigen::Isometry3d::Identity();
    bool first_frame = true;

    std::vector<cv::Point3f> pts3d_last_;
    std::vector<cv::Point2f> pts2d_curr_;

    cv::Mat K_;
    cv::Mat dist_;

    
};


#endif
