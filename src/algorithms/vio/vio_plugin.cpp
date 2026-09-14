#include "vio_plugin.hpp"



#include <krisea_log/logger.hpp>

#include <algorithm>
#include <cmath>
#include <utility>

namespace
{

template <typename T>
void insertByTimestamp(std::deque<T>& buffer, T data)
{
    const auto position = std::upper_bound(
        buffer.begin(), buffer.end(), data.timestamp,
        [](double timestamp, const T& buffered) {
            return timestamp < buffered.timestamp;
        });
    buffer.insert(position, std::move(data));
}

template <typename T>
void trimBuffer(std::deque<T>& buffer, std::size_t max_size)
{
    while (buffer.size() > max_size)
    {
        buffer.pop_front();
    }
}
 
}  // namespace

VioPlugin::VioPlugin()
    : stereo_mode_(stereoInputEnabled()),
      max_time_diff_(g_input_max_time_diff)
{
    if (stereo_mode_)
    {
        ready_ = stereo_depth_.initialize();
        if (ready_)
        {
            const cv::Mat& projection = stereo_depth_.leftProjection();
            g_fx = projection.at<double>(0, 0);
            g_fy = projection.at<double>(1, 1);
            g_cx = projection.at<double>(0, 2);
            g_cy = projection.at<double>(1, 2);
            g_K = (cv::Mat_<double>(3, 3) <<
                g_fx, 0.0, g_cx,
                0.0, g_fy, g_cy,
                0.0, 0.0, 1.0);
            g_depth_scale = 1000.0;
        }
    }

    KR_INFO(
        "VioPlugin input mode={} ready={} max_time_diff={:.3f}s",
        g_input_mode, ready_, max_time_diff_);
}

void VioPlugin::inputImage(ImageData image)
{
    if (image.image.empty())
    {
        KR_WARN("Dropping empty RGB image at {:.9f}", image.timestamp);
        return;
    }

    std::lock_guard<std::mutex> lock(buffer_mutex_);
    insertByTimestamp(rgb_buf_, std::move(image));
    trimBuffer(rgb_buf_, kMaxImageBufferSize);
}

void VioPlugin::inputDepth(DepthImageData depth)
{
    if (depth.image.empty())
    {
        KR_WARN("Dropping empty depth image at {:.9f}", depth.timestamp);
        return;
    }

    if (depth.image.type() != CV_16UC1)
    {
        KR_WARN("Dropping unsupported depth image type={} at {:.9f}",
            depth.image.type(), depth.timestamp);
        return;
    }

    std::lock_guard<std::mutex> lock(buffer_mutex_);
    insertByTimestamp(depth_buf_, std::move(depth));
    trimBuffer(depth_buf_, kMaxImageBufferSize);
}

void VioPlugin::inputLeftImage(ImageData image)
{
    if (image.image.empty())
    {
        KR_WARN("Dropping empty left image at {:.9f}", image.timestamp);
        return;
    }

    std::lock_guard<std::mutex> lock(buffer_mutex_);
    insertByTimestamp(left_buf_, std::move(image));
    trimBuffer(left_buf_, kMaxImageBufferSize);
}

void VioPlugin::inputRightImage(ImageData image)
{
    if (image.image.empty())
    {
        KR_WARN("Dropping empty right image at {:.9f}", image.timestamp);
        return;
    }

    std::lock_guard<std::mutex> lock(buffer_mutex_);
    insertByTimestamp(right_buf_, std::move(image));
    trimBuffer(right_buf_, kMaxImageBufferSize);
}

void VioPlugin::inputImu(ImuData imu)
{
    estimator_.inputImu(std::move(imu));
}

bool VioPlugin::popMatchedRgbdFrame(Frame& frame)
{
    std::lock_guard<std::mutex> lock(buffer_mutex_);

    while (!rgb_buf_.empty() && !depth_buf_.empty())
    {
        // Consume the oldest RGB image and find the closest depth timestamp.
        const double rgb_timestamp = rgb_buf_.front().timestamp;
        const auto best_depth = std::min_element(
            depth_buf_.begin(), depth_buf_.end(),
            [rgb_timestamp](const DepthImageData& lhs, const DepthImageData& rhs) {
                return std::abs(lhs.timestamp - rgb_timestamp) <
                       std::abs(rhs.timestamp - rgb_timestamp);
            });

        const double time_diff = std::abs(best_depth->timestamp - rgb_timestamp);
        if (time_diff <= max_time_diff_)
        {
            ImageData rgb = std::move(rgb_buf_.front());
            DepthImageData depth = std::move(*best_depth);
            rgb_buf_.pop_front();
            depth_buf_.erase(best_depth);

            frame = Frame{};
            frame.id = next_frame_id_++;
            frame.timestamp = rgb.timestamp;
            frame.rgb = std::move(rgb.image);
            frame.depth = std::move(depth.image);
            return true;
        }

        // Only discard measurements that are already outside the other
        // stream's current time range. No synchronizer/state machine here.
        if (depth_buf_.front().timestamp >
            rgb_timestamp + max_time_diff_)
        {
            KR_DEBUG(
                "Dropping stale RGB: t={:.9f}, first depth={:.9f}",
                rgb_timestamp, depth_buf_.front().timestamp);
            rgb_buf_.pop_front();
            continue;
        }

        if (depth_buf_.back().timestamp <
            rgb_timestamp - max_time_diff_)
        {
            KR_DEBUG(
                "Dropping stale depth: t={:.9f}, rgb={:.9f}",
                depth_buf_.front().timestamp, rgb_timestamp);
            depth_buf_.pop_front();
            continue;
        }

        // The matching depth may still arrive later. Keep both buffers as-is.
        return false;
    }
 
    return false;
}

bool VioPlugin::popMatchedStereoFrame(Frame& frame)
{
    ImageData left;
    ImageData right;
    double time_diff = 0.0;

    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        while (!left_buf_.empty() && !right_buf_.empty())
        {
            const double left_timestamp = left_buf_.front().timestamp;
            const auto best_right = std::min_element(
                right_buf_.begin(), right_buf_.end(),
                [left_timestamp](const ImageData& lhs, const ImageData& rhs) {
                    return std::abs(lhs.timestamp - left_timestamp) <
                           std::abs(rhs.timestamp - left_timestamp);
                });

            time_diff = std::abs(best_right->timestamp - left_timestamp);
            if (time_diff <= max_time_diff_)
            {
                left = std::move(left_buf_.front());
                right = std::move(*best_right);
                left_buf_.pop_front();
                right_buf_.erase(best_right);
                break;
            }

            if (right_buf_.front().timestamp > left_timestamp + max_time_diff_)
            {
                KR_DEBUG(
                    "Dropping stale left image: t={:.9f}, first right={:.9f}",
                    left_timestamp, right_buf_.front().timestamp);
                left_buf_.pop_front();
                continue;
            }
            if (right_buf_.back().timestamp < left_timestamp - max_time_diff_)
            {
                KR_DEBUG(
                    "Dropping stale right image: t={:.9f}, left={:.9f}",
                    right_buf_.front().timestamp, left_timestamp);
                right_buf_.pop_front();
                continue;
            }
            return false;
        }
    }

    if (left.image.empty() || right.image.empty())
    {
        return false;
    }

    cv::Mat rectified_left;
    cv::Mat depth_mm;
    if (!stereo_depth_.compute(
            left.image, right.image, rectified_left, depth_mm))
    {
        KR_WARN(
            "[Stereo] Frame generation failed: left_t={:.9f} right_t={:.9f}",
            left.timestamp, right.timestamp);
        return false;
    }

    frame = Frame{};
    frame.id = next_frame_id_++;
    frame.timestamp = left.timestamp;
    frame.rgb = std::move(rectified_left);
    frame.depth = std::move(depth_mm);
    KR_DEBUG(
        "[Stereo] Matched frame={} dt={:.6f}s",
        frame.id, time_diff);
    return true;
}

void VioPlugin::process()
 {
    if (!ready_)
    {
        return;
    }

    Frame frame;
    const auto pop_frame = [this, &frame]() {
        return stereo_mode_
            ? popMatchedStereoFrame(frame)
            : popMatchedRgbdFrame(frame);
    };
    while (pop_frame())
    {
        estimator_.inputFrame(std::move(frame));
        frame = Frame{};
    }
 
    // Estimator owns frame/IMU timestamp alignment and solving.
    while (const auto result = estimator_.process())
    {
        publishPose(*result);
    }
}

void VioPlugin::publishPose(const EstimatorResult& result)
{
    PoseData pose_data;
    pose_data.timestamp = result.timestamp;
    pose_data.position = result.pose.translation();
    pose_data.orientation =
        Eigen::Quaterniond(result.pose.rotation()).normalized();

    if (pose_callback_)
    {
        pose_callback_(pose_data);
    }
}

void VioPlugin::setPoseCallback(PoseCallback callback)
{
    pose_callback_ = std::move(callback);
}

void VioPlugin::reset()
{
    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        rgb_buf_.clear();
        depth_buf_.clear();
        left_buf_.clear();
        right_buf_.clear();
        next_frame_id_ = 0;
    }
    estimator_.reset();
}
