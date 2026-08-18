#include "estimator.hpp"
#include <iostream>
#include <krisea_log/logger.hpp>
#include "config.hpp"


bool Estimator::processFrame(Frame& frame)
{
    if (first_frame_)
    {
        frame_ = frame;
        first_frame_ = false;
        return true;
    }

    tracker_.detect_klt(frame_, frame);

    //3d-2d
    tracker_.get3d2d(frame_, pts3d_last_, pts2d_curr_);
    if(pts3d_last_.size()<10)
        {

            KR_WARN("Not enough 3D-2D correspondences: {}", pts3d_last_.size());
            frame_=frame;
            return false;
        }

    //pnp
    // cv::Mat K = g_K;
    cv::Mat dist = cv::Mat::zeros(5,1,CV_64F);
    cv::Mat rvec;
    cv::Mat tvec;
    cv::Mat inliers;

    bool success = cv::solvePnPRansac(
        pts3d_last_, 
        pts2d_curr_, 
        g_K, 
        dist, 
        rvec, 
        tvec, 
        false,
        100,      // iterationsCount
        3.0,      // reprojectionError
        0.99,     // confidence
        inliers,
        cv::SOLVEPNP_ITERATIVE);
    
    const int match_count =
    static_cast<int>(pts3d_last_.size());
    const int inlier_count = inliers.rows;
    const double inlier_ratio = match_count > 0 ? static_cast<double>(inlier_count) / match_count : 0.0;

    if(success)
    {
        cv::Mat R_cv;
        cv::Rodrigues(rvec, R_cv);

        Eigen::Matrix3d R;
        for (int row = 0; row < 3; ++row)
        {
            for (int col = 0; col < 3; ++col)
            {
                R(row, col) = R_cv.at<double>(row, col);
            }
        }

        Eigen::Vector3d t(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
        // solvePnP 给出 T_curr_last：上一帧坐标到当前帧坐标
        // 轨迹需要其逆变换 T_last_curr
        Eigen::Isometry3d T_last_curr = Eigen::Isometry3d::Identity();
        T_last_curr.linear() = R.transpose();
        T_last_curr.translation() = -R.transpose() * t;

        // 累计得到当前帧在世界坐标系中的位姿
        pose_ = pose_ * T_last_curr;


    }
    else
    {
        frame_ = frame;
        return false;
    }

    //更新pose


    frame_ = frame;

    return true;
}

void Estimator::processImu(const ImuData& imu)
{
    // IMU preintegration will be implemented later.
    (void)imu;
}

const Eigen::Isometry3d& Estimator::pose() const
{
    return pose_;
}

void Estimator::reset()
{
    tracker_ = FeatureTracker{};
    frame_ = Frame{};

    pose_ = Eigen::Isometry3d::Identity();
    first_frame_ = true;

    pts3d_last_.clear();
    pts2d_curr_.clear();

    
}
