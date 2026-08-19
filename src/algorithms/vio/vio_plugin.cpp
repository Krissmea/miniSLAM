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

void VioPlugin::inputImu(ImuData imu)
{
    estimator_.inputImu(std::move(imu));
}

bool VioPlugin::popMatchedFrame(Frame& frame)
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
        if (time_diff <= kRgbDepthMaxTimeDiff)
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
            rgb_timestamp + kRgbDepthMaxTimeDiff)
        {
            KR_DEBUG(
                "Dropping stale RGB: t={:.9f}, first depth={:.9f}",
                rgb_timestamp, depth_buf_.front().timestamp);
            rgb_buf_.pop_front();
            continue;
        }

        if (depth_buf_.back().timestamp <
            rgb_timestamp - kRgbDepthMaxTimeDiff)
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

void VioPlugin::process()
 {
    // RGB/depth matching happens only here, never in the input callbacks.
    Frame frame;
    while (popMatchedFrame(frame))
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
        next_frame_id_ = 0;
    }
    estimator_.reset();
}