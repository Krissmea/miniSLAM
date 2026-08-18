#include "estimator.hpp"
#include <iostream>
#include <krisea_log/logger.hpp>
#include "config.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace
{

constexpr std::size_t kMinCorrespondences = 30;
constexpr int kMinInliers = 20;
constexpr double kMinInlierRatio = 0.5;
constexpr double kMaxMedianReprojectionError = 2.5;
constexpr double kMaxTranslationPerFrame = 0.5;
constexpr double kMaxRotationRadians = 20.0 * CV_PI / 180.0;

double medianReprojectionError(
    const std::vector<cv::Point3f>& object_points,
    const std::vector<cv::Point2f>& image_points,
    const cv::Mat& inliers,
    const cv::Mat& rvec,
    const cv::Mat& tvec,
    const cv::Mat& camera_matrix,
    const cv::Mat& distortion)
{
    std::vector<cv::Point3f> inlier_object_points;
    std::vector<cv::Point2f> inlier_image_points;
    inlier_object_points.reserve(inliers.rows);
    inlier_image_points.reserve(inliers.rows);
    for (int row = 0; row < inliers.rows; ++row)
    {
        const int index = inliers.at<int>(row, 0);
        if (index < 0 ||
            index >= static_cast<int>(object_points.size()))
        {
            continue;
        }
        inlier_object_points.push_back(object_points[index]);
        inlier_image_points.push_back(image_points[index]);
    }

    if (inlier_object_points.empty())
    {
        return std::numeric_limits<double>::infinity();
    }

    std::vector<cv::Point2f> projected_points;
    cv::projectPoints(
        inlier_object_points, rvec, tvec, camera_matrix,
        distortion, projected_points);

    std::vector<double> errors;
    errors.reserve(projected_points.size());
    for (std::size_t i = 0; i < projected_points.size(); ++i)
    {
        errors.push_back(cv::norm(
            projected_points[i] - inlier_image_points[i]));
    }
    const auto middle = errors.begin() + errors.size() / 2;
    std::nth_element(errors.begin(), middle, errors.end());
    return *middle;
}

}  // namespace


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
    if (pts3d_last_.size() < kMinCorrespondences)
    {
        KR_WARN(
            "Not enough 3D-2D correspondences: {} < {}",
            pts3d_last_.size(), kMinCorrespondences);
        frame_ = frame;
        return false;
    }

    //pnp
    // cv::Mat K = g_K;
    cv::Mat dist = cv::Mat::zeros(5,1,CV_64F);
    cv::Mat rvec;
    cv::Mat tvec;
    cv::Mat inliers;

    const bool success = cv::solvePnPRansac(
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
    
    const int match_count = static_cast<int>(pts3d_last_.size());
    const int inlier_count = inliers.rows;
    const double inlier_ratio = match_count > 0
        ? static_cast<double>(inlier_count) / match_count
        : 0.0;

    if (!success || inlier_count < kMinInliers ||
        inlier_ratio < kMinInlierRatio)
    {
        KR_WARN(
            "Rejecting PnP: success={} matches={} inliers={} ratio={:.3f}",
            success, match_count, inlier_count, inlier_ratio);
        frame_ = frame;
        return false;
    }

    const double median_reprojection_error = medianReprojectionError(
        pts3d_last_, pts2d_curr_, inliers, rvec, tvec,
        g_K, dist);
    if (!std::isfinite(median_reprojection_error) ||
        median_reprojection_error > kMaxMedianReprojectionError)
    {
        KR_WARN(
            "Rejecting PnP: median reprojection error={:.3f}px",
            median_reprojection_error);
        frame_ = frame;
        return false;
    }

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

    const Eigen::Vector3d t(
        tvec.at<double>(0), tvec.at<double>(1),
        tvec.at<double>(2));
    Eigen::Isometry3d T_last_curr = Eigen::Isometry3d::Identity();
    T_last_curr.linear() = R.transpose();
    T_last_curr.translation() = -R.transpose() * t;

    const double translation_norm = T_last_curr.translation().norm();
    const double rotation_angle = Eigen::AngleAxisd(
        T_last_curr.rotation()).angle();
    if (!T_last_curr.matrix().allFinite() ||
        translation_norm > kMaxTranslationPerFrame ||
        rotation_angle > kMaxRotationRadians)
    {
        KR_WARN(
            "Rejecting PnP motion: translation={:.3f}m rotation={:.3f}deg",
            translation_norm, rotation_angle * 180.0 / CV_PI);
        frame_ = frame;
        return false;
    }

    pose_ = pose_ * T_last_curr;
    KR_INFO(
        "Visual update: matches={} inliers={} ratio={:.3f} "
        "reprojection={:.3f}px translation={:.3f}m rotation={:.3f}deg",
        match_count, inlier_count, inlier_ratio,
        median_reprojection_error, translation_norm,
        rotation_angle * 180.0 / CV_PI);

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
