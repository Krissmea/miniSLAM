#ifndef MINI_SLAM_STEREO_DEPTH_H
#define MINI_SLAM_STEREO_DEPTH_H

#include "config.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

class StereoDepth
{
public:
    bool initialize();
    bool compute(
        const cv::Mat& left,
        const cv::Mat& right,
        cv::Mat& rectified_left,
        cv::Mat& depth_mm);

    bool ready() const { return ready_; }
    double baseline() const { return baseline_; }
    const cv::Mat& leftProjection() const { return left_projection_; }
    const cv::Mat& rightProjection() const { return right_projection_; }
    const cv::Mat& reprojectionMatrix() const { return reprojection_matrix_; }
    const cv::Ptr<cv::StereoSGBM>& matcher() const { return matcher_; }

private:
    bool ready_{false};
    double baseline_{0.0};
    cv::Size image_size_;
    double min_depth_{0.2};
    double max_depth_{8.0};

    cv::Mat left_map_x_;
    cv::Mat left_map_y_;
    cv::Mat right_map_x_;
    cv::Mat right_map_y_;
    cv::Mat left_projection_;
    cv::Mat right_projection_;
    cv::Mat reprojection_matrix_;
    cv::Ptr<cv::StereoSGBM> matcher_;
};

#endif
