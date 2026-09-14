#include "stereo/stereo_depth.hpp"

#include <krisea_log/logger.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>
#include <cstdint>
#include <string>

namespace
{

cv::Mat asDouble(const cv::Mat& source)
{
    cv::Mat result;
    source.convertTo(result, CV_64F);
    return result;
}

bool toMono8(const cv::Mat& source, cv::Mat& result)
{
    if (source.empty())
    {
        return false;
    }
    if (source.type() == CV_8UC1)
    {
        result = source;
        return true;
    }
    if (source.type() == CV_16UC1)
    {
        source.convertTo(result, CV_8UC1, 1.0 / 256.0);
        return true;
    }
    if (source.type() == CV_8UC3)
    {
        cv::cvtColor(source, result, cv::COLOR_BGR2GRAY);
        return true;
    }
    if (source.type() == CV_8UC4)
    {
        cv::cvtColor(source, result, cv::COLOR_BGRA2GRAY);
        return true;
    }
    return false;
}

}  // namespace

bool StereoDepth::initialize()
{
    ready_ = false;
    baseline_ = 0.0;
    matcher_.release();
    image_size_ = cv::Size{};

    std::string reason;
    if (!validateStereoConfig(reason))
    {
        KR_ERROR("Cannot initialize stereo depth: {}", reason);
        return false;
    }

    try
    {
        const cv::Size image_size(g_stereo_image_width, g_stereo_image_height);
        const cv::Mat left_K = asDouble(g_stereo_left_K);
        const cv::Mat left_D = asDouble(g_stereo_left_D);
        const cv::Mat right_K = asDouble(g_stereo_right_K);
        const cv::Mat right_D = asDouble(g_stereo_right_D);
        const cv::Mat rotation = asDouble(g_R_right_left);
        const cv::Mat translation = asDouble(g_t_right_left).reshape(1, 3);

        cv::Mat left_rectification;
        cv::Mat right_rectification;
        cv::stereoRectify(
            left_K, left_D,
            right_K, right_D,
            image_size,
            rotation, translation,
            left_rectification, right_rectification,
            left_projection_, right_projection_, reprojection_matrix_,
            cv::CALIB_ZERO_DISPARITY, 0.0, image_size);

        cv::initUndistortRectifyMap(
            left_K, left_D, left_rectification, left_projection_,
            image_size, CV_32FC1, left_map_x_, left_map_y_);
        cv::initUndistortRectifyMap(
            right_K, right_D, right_rectification, right_projection_,
            image_size, CV_32FC1, right_map_x_, right_map_y_);

        const double rectified_fx = left_projection_.at<double>(0, 0);
        const double right_tx = right_projection_.at<double>(0, 3);
        if (!std::isfinite(rectified_fx) || rectified_fx <= 0.0 ||
            !std::isfinite(right_tx))
        {
            KR_ERROR("Stereo rectification produced an invalid projection matrix");
            return false;
        }

        baseline_ = std::abs(right_tx / right_projection_.at<double>(0, 0));
        if (!std::isfinite(baseline_) || baseline_ <= 1e-6)
        {
            KR_ERROR("Stereo rectification produced an invalid baseline");
            return false;
        }

        matcher_ = cv::StereoSGBM::create(
            g_sgbm_min_disparity,
            g_sgbm_num_disparities,
            g_sgbm_block_size);

        const int window_area =
            g_sgbm_block_size * g_sgbm_block_size;
        matcher_->setP1(8 * window_area);
        matcher_->setP2(32 * window_area);
        matcher_->setUniquenessRatio(g_sgbm_uniqueness_ratio);
        matcher_->setSpeckleWindowSize(g_sgbm_speckle_window_size);
        matcher_->setSpeckleRange(g_sgbm_speckle_range);
        matcher_->setDisp12MaxDiff(g_sgbm_disp12_max_diff);
        matcher_->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);

        image_size_ = image_size;
        min_depth_ = g_min_depth_meters;
        max_depth_ = g_max_depth_meters;
        ready_ = true;
        KR_INFO(
            "Stereo depth initialized: size={}x{} fx={:.3f} baseline={:.6f}m "
            "disparities={} block_size={}",
            image_size.width, image_size.height,
            rectified_fx, baseline_, g_sgbm_num_disparities,
            g_sgbm_block_size);
        return true;
    }
    catch (const cv::Exception& exception)
    {
        KR_ERROR("OpenCV stereo initialization failed: {}", exception.what());
        return false;
    }
}

bool StereoDepth::compute(
    const cv::Mat& left,
    const cv::Mat& right,
    cv::Mat& rectified_left,
    cv::Mat& depth_mm)
{
    rectified_left.release();
    depth_mm.release();

    if (!ready_ || left.size() != image_size_ || right.size() != image_size_)
    {
        KR_WARN(
            "Stereo input rejected: ready={} left={}x{} right={}x{} expected={}x{}",
            ready_, left.cols, left.rows, right.cols, right.rows,
            image_size_.width, image_size_.height);
        return false;
    }

    cv::Mat left_mono;
    cv::Mat right_mono;
    if (!toMono8(left, left_mono) || !toMono8(right, right_mono))
    {
        KR_WARN(
            "Unsupported stereo image types: left={} right={}",
            left.type(), right.type());
        return false;
    }

    cv::Mat right_rectified;
    cv::remap(
        left_mono, rectified_left, left_map_x_, left_map_y_,
        cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    cv::remap(
        right_mono, right_rectified, right_map_x_, right_map_y_,
        cv::INTER_LINEAR, cv::BORDER_CONSTANT);

    cv::Mat disparity_fixed;
    matcher_->compute(rectified_left, right_rectified, disparity_fixed);

    depth_mm = cv::Mat::zeros(image_size_, CV_16UC1);
    const double rectified_fx = left_projection_.at<double>(0, 0);
    std::size_t valid_depth_count = 0;
    for (int row = 0; row < disparity_fixed.rows; ++row)
    {
        const auto* disparities = disparity_fixed.ptr<std::int16_t>(row);
        auto* depths = depth_mm.ptr<std::uint16_t>(row);
        for (int col = 0; col < disparity_fixed.cols; ++col)
        {
            const double disparity =
                static_cast<double>(disparities[col]) / 16.0;
            if (!std::isfinite(disparity) || disparity <= 0.0)
            {
                continue;
            }

            const double depth = rectified_fx * baseline_ / disparity;
            if (!std::isfinite(depth) || depth < min_depth_ ||
                depth > max_depth_)
            {
                continue;
            }

            depths[col] = cv::saturate_cast<std::uint16_t>(
                std::lround(depth * 1000.0));
            ++valid_depth_count;
        }
    }

    const double valid_ratio = static_cast<double>(valid_depth_count) /
        static_cast<double>(depth_mm.total());
    KR_DEBUG(
        "[Stereo] valid_depth={} ratio={:.3f}",
        valid_depth_count, valid_ratio);
    return valid_depth_count > 0;
}
