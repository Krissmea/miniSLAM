#include "feature_tracker.hpp"

#include "config.hpp"

#include <krisea_log/logger.hpp>

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <optional>
#include <string>
#include <utility>

namespace
{
constexpr int kDepthWindowRadius = 1;               //深度取值，某点周围深度的中位数
constexpr double kMaxDepthDeviationMeters = 0.2;    //邻域内最大距离与最小距离差值的阈值


//这里保证的是去除未跟踪上的点后，去除这些点，并且保证剩下点的下标连续，注意不是ID连续
/***
    index:          0     1     2
    point:          P0    P2    P4
    ID:             10    12    14
    track_count:    8     3     2
 */
template <typename T>
void reduceVector(std::vector<T>& values, const std::vector<uchar>& status)
{
    std::size_t write_index = 0;
    for (std::size_t i = 0; i < values.size(); ++i)
    {
        if (status[i])
        {
            // 移动赋值，节省空间
            values[write_index++] = std::move(values[i]);
        }
    }
    values.resize(write_index);
}

std::optional<double> queryRobustDepth(
    const cv::Mat& depth, const cv::Point2f& pixel)
{
    if (depth.empty() || depth.type() != CV_16UC1 ||
        !std::isfinite(pixel.x) || !std::isfinite(pixel.y) ||
        g_depth_scale <= 0.0)
    {
        return std::nullopt;
    }

    const int center_x = cvRound(pixel.x);
    const int center_y = cvRound(pixel.y);
    std::vector<double> samples;
    samples.reserve(
        (2 * kDepthWindowRadius + 1) *
        (2 * kDepthWindowRadius + 1));

    for (int offset_y = -kDepthWindowRadius;
         offset_y <= kDepthWindowRadius; ++offset_y)
    {
        for (int offset_x = -kDepthWindowRadius;
             offset_x <= kDepthWindowRadius; ++offset_x)
        {
            const int x = center_x + offset_x;
            const int y = center_y + offset_y;
            if (x < 0 || y < 0 || x >= depth.cols || y >= depth.rows)
            {
                continue;
            }

            const std::uint16_t raw = depth.at<std::uint16_t>(y, x);
            const double meters = static_cast<double>(raw) / g_depth_scale;
            if (raw != 0 && meters >= g_min_depth_meters &&
                meters <= g_max_depth_meters)
            {
                samples.push_back(meters);
            }
        }
    }

    if (samples.size() < 3)
    {
        return std::nullopt;
    }

    const auto [minimum, maximum] = std::minmax_element(
        samples.begin(), samples.end());
    if (*maximum - *minimum > kMaxDepthDeviationMeters)
    {
        return std::nullopt;
    }

    const auto middle = samples.begin() + samples.size() / 2;
    std::nth_element(samples.begin(), middle, samples.end());
    return *middle;
}

}  // namespace

bool FeatureTracker::initialize(Frame& frame)
{
    cv::Mat gray;
    if (!toGray(frame.rgb, gray))
    {
        KR_WARN("Cannot initialize tracker: invalid RGB frame");
        return false;
    }

    pts_last_.clear();
    pts_curr_.clear();
    tracked_features_.clear();
    initializeTracks(gray);

    KR_INFO("Feature tracker initialized: active:{}",active_points_.size());
    return !active_points_.empty();
}

bool FeatureTracker::toGray(const cv::Mat& image, cv::Mat& gray)
{
    if (image.empty())
    {
        return false;
    }

    if (image.channels() == 1)
    {
        gray = image;
        return true;
    }
    if (image.channels() == 3)
    {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        return true;
    }
    if (image.channels() == 4)
    {
        cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
        return true;
    }

    KR_WARN("Unsupported image channel count: {}", image.channels());
    return false;
}

bool FeatureTracker::inBorder(
    const cv::Point2f& point, int cols, int rows)
{
    return std::isfinite(point.x) && std::isfinite(point.y) &&
           point.x >= kBorderSize && point.y >= kBorderSize &&
           point.x < cols - kBorderSize && point.y < rows - kBorderSize;
}

// 初始化步骤，在灰度图上查找特征点分配空间，id等，为后续光流等做准备
void FeatureTracker::initializeTracks(const cv::Mat& gray)
{
    active_points_.clear();
    active_ids_.clear();
    active_track_counts_.clear();

    // 使用 shi-tomasi 角点检测
    cv::goodFeaturesToTrack(
        gray, active_points_, kMaxFeatures, kQualityLevel,
        kMinFeatureDistance, cv::Mat(), 7, false, 0.04);

    active_ids_.reserve(active_points_.size());
    active_track_counts_.reserve(active_points_.size());
    for (std::size_t i = 0; i < active_points_.size(); ++i)
    {
        active_ids_.push_back(next_feature_id_++);
        active_track_counts_.push_back(1);
    }
}

void FeatureTracker::rejectWithFundamental(
    std::vector<cv::Point2f>& previous_points,
    std::vector<cv::Point2f>& current_points,
    std::vector<int>& ids,
    std::vector<int>& track_counts) const
{
    if (previous_points.size() < 8)
    {
        return;
    }

    std::vector<uchar> status;
    const cv::Mat fundamental = cv::findFundamentalMat(
        previous_points, current_points, cv::FM_RANSAC,
        kFundamentalThreshold, kFundamentalConfidence, status);

    if (fundamental.empty() || status.size() != previous_points.size())
    {
        KR_WARN("Fundamental matrix estimation failed; retaining LK tracks");
        return;
    }

    reduceVector(previous_points, status);
    reduceVector(current_points, status);
    reduceVector(ids, status);
    reduceVector(track_counts, status);
}


/***
 * 在当前所有已经跟踪到的特征点里，优先保留“跟踪时间长”的老点，
 * 同时让保留下来的点在图像上尽量分散，避免很多特征点挤在同一个局部区域。
 */
cv::Mat FeatureTracker::selectSpatiallyDistributedTracks(
    const cv::Size& image_size,
    std::vector<cv::Point2f>& previous_points,
    std::vector<cv::Point2f>& current_points,
    std::vector<int>& ids,
    std::vector<int>& track_counts) const
{
    cv::Mat mask(image_size, CV_8UC1, cv::Scalar(255));

    std::vector<std::size_t> order(current_points.size());
    std::iota(order.begin(), order.end(), 0);
    std::stable_sort(
        order.begin(), order.end(),
        [&track_counts](std::size_t lhs, std::size_t rhs) {
            return track_counts[lhs] > track_counts[rhs];
        });

    std::vector<cv::Point2f> selected_previous;
    std::vector<cv::Point2f> selected_current;
    std::vector<int> selected_ids;
    std::vector<int> selected_track_counts;
    selected_previous.reserve(previous_points.size());
    selected_current.reserve(current_points.size());
    selected_ids.reserve(ids.size());
    selected_track_counts.reserve(track_counts.size());

    for (const std::size_t index : order)
    {
        const cv::Point2f& point = current_points[index];
        const int x = cvRound(point.x);
        const int y = cvRound(point.y);
        if (!inBorder(point, image_size.width, image_size.height) ||
            mask.at<uchar>(y, x) == 0)
        {
            continue;
        }

        selected_previous.push_back(previous_points[index]);
        selected_current.push_back(point);
        selected_ids.push_back(ids[index]);
        selected_track_counts.push_back(track_counts[index]);
        cv::circle(mask, point, kMinFeatureDistance, 0, -1);
    }

    previous_points = std::move(selected_previous);
    current_points = std::move(selected_current);
    ids = std::move(selected_ids);
    track_counts = std::move(selected_track_counts);
    return mask;
}


// 针对去除外点异常点后进行补点
void FeatureTracker::addNewFeatures(
    const cv::Mat& gray, const cv::Mat& mask)
{
    const int required =
        kMaxFeatures - static_cast<int>(active_points_.size());
    if (required <= 0)
    {
        return;
    }

    std::vector<cv::Point2f> new_points;
    cv::goodFeaturesToTrack(
        gray, new_points, required, kQualityLevel,
        kMinFeatureDistance, mask, 7, false, 0.04);

    for (const cv::Point2f& point : new_points)
    {
        active_points_.push_back(point);
        active_ids_.push_back(next_feature_id_++);
        active_track_counts_.push_back(1);
    }
}


// 特征点匹配可视化
void FeatureTracker::visualizeTracks(
    const cv::Mat& previous_gray, const cv::Mat& current_gray) const
{
    if (!SHOW_KLT_TRACKING)
    {
        return;
    }

    cv::Mat previous_vis;
    cv::Mat current_vis;
    cv::cvtColor(previous_gray, previous_vis, cv::COLOR_GRAY2BGR);
    cv::cvtColor(current_gray, current_vis, cv::COLOR_GRAY2BGR);

    for (const TrackedFeature& feature : tracked_features_)
    {
        cv::circle(
            previous_vis, feature.previous_pixel, 3,
            cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
        cv::circle(
            current_vis, feature.current_pixel, 3,
            cv::Scalar(0, 255, 0), -1, cv::LINE_AA);
        cv::putText(
            current_vis, std::to_string(feature.id),
            feature.current_pixel + cv::Point2f(3.0F, -3.0F),
            cv::FONT_HERSHEY_PLAIN, 0.7, cv::Scalar(0, 255, 255), 1);
    }

    cv::Mat combined;
    cv::hconcat(previous_vis, current_vis, combined);
    const int offset = previous_vis.cols;
    for (const TrackedFeature& feature : tracked_features_)
    {
        cv::line(
            combined, feature.previous_pixel,
            feature.current_pixel + cv::Point2f(
                static_cast<float>(offset), 0.0F),
            cv::Scalar(255, 0, 0), 1, cv::LINE_AA);
    }

    cv::putText(
        combined,
        "Tracked: " + std::to_string(tracked_features_.size()) +
            " Active: " + std::to_string(active_points_.size()),
        cv::Point(20, combined.rows - 20), cv::FONT_HERSHEY_SIMPLEX,
        0.7, cv::Scalar(0, 255, 255), 2);
    cv::imshow("Persistent feature tracks", combined);
    cv::waitKey(1);
}

void FeatureTracker::detect_klt(Frame& frame_last, Frame& frame_curr)
{
    pts_last_.clear();
    pts_curr_.clear();
    tracked_features_.clear();

    cv::Mat previous_gray;
    cv::Mat current_gray;
    if (!toGray(frame_last.rgb, previous_gray) ||
        !toGray(frame_curr.rgb, current_gray))
    {
        return;
    }
    if (previous_gray.size() != current_gray.size())
    {
        KR_WARN(
            "Image size changed from {}x{} to {}x{}; resetting tracks",
            previous_gray.cols, previous_gray.rows,
            current_gray.cols, current_gray.rows);
        reset();
        return;
    }

    if (active_points_.empty() ||
        active_points_.size() != active_ids_.size() ||
        active_points_.size() != active_track_counts_.size())
    {
        //此步初始化之后，给被追踪到点分配ID，以及统计被追踪的次数
        initializeTracks(previous_gray);
    }
    if (active_points_.empty())
    {
        KR_WARN("No features detected in previous frame");
        return;
    }

    std::vector<cv::Point2f> current_points;
    std::vector<uchar> forward_status;
    std::vector<float> forward_error;
    cv::calcOpticalFlowPyrLK(
        previous_gray, current_gray, active_points_, current_points,
        forward_status, forward_error, cv::Size(21, 21), 3,
        cv::TermCriteria(
            cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01));

    std::vector<cv::Point2f> backward_points;
    std::vector<uchar> backward_status;
    std::vector<float> backward_error;
    cv::calcOpticalFlowPyrLK(
        current_gray, previous_gray, current_points, backward_points,
        backward_status, backward_error, cv::Size(21, 21), 3,
        cv::TermCriteria(
            cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01));

    std::vector<cv::Point2f> valid_previous;
    std::vector<cv::Point2f> valid_current;
    std::vector<int> valid_ids;
    std::vector<int> valid_track_counts;
    valid_previous.reserve(active_points_.size());
    valid_current.reserve(active_points_.size());
    valid_ids.reserve(active_points_.size());
    valid_track_counts.reserve(active_points_.size());

    for (std::size_t i = 0; i < active_points_.size(); ++i)
    {
        if (!forward_status[i] || !backward_status[i] ||
            !inBorder(current_points[i], current_gray.cols, current_gray.rows) ||
            cv::norm(active_points_[i] - backward_points[i]) >
                kForwardBackwardThreshold)
        {
            continue;
        }

        valid_previous.push_back(active_points_[i]);
        valid_current.push_back(current_points[i]);
        valid_ids.push_back(active_ids_[i]);
        valid_track_counts.push_back(active_track_counts_[i] + 1);
    }

    const std::size_t before_geometry = valid_current.size();
    rejectWithFundamental(
        valid_previous, valid_current, valid_ids, valid_track_counts);
    const cv::Mat detection_mask = selectSpatiallyDistributedTracks(
        current_gray.size(), valid_previous, valid_current,
        valid_ids, valid_track_counts);

    pts_last_ = valid_previous;
    pts_curr_ = valid_current;
    tracked_features_.reserve(valid_current.size());
    for (std::size_t i = 0; i < valid_current.size(); ++i)
    {
        tracked_features_.push_back({
            valid_ids[i], valid_track_counts[i],
            valid_previous[i], valid_current[i]});
    }

    active_points_ = valid_current;
    active_ids_ = valid_ids;
    active_track_counts_ = valid_track_counts;
    addNewFeatures(current_gray, detection_mask);

    

    KR_INFO(
        "Feature tracking: input={} fb_valid={} geometry_valid={} active={}",
        forward_status.size(), before_geometry, tracked_features_.size(),
        active_points_.size());
    visualizeTracks(previous_gray, current_gray);
}

void FeatureTracker::get3d2d(
    Frame& frame_last,
    std::vector<cv::Point3f>& pts3d_last,
    std::vector<cv::Point2f>& pts2d_curr)
{
    pts3d_last.clear();
    pts2d_curr.clear();

    for (std::size_t i = 0; i < pts_last_.size(); ++i)
    {
        const std::optional<double> depth = queryRobustDepth(
            frame_last.depth, pts_last_[i]);
        if (!depth)
        {
            continue;
        }

        const int u = cvRound(pts_last_[i].x);
        const int v = cvRound(pts_last_[i].y);
        const Eigen::Vector3d point = pointcloud_.pixel2camera(
            u, v, *depth);
        pts3d_last.emplace_back(
            static_cast<float>(point.x()),
            static_cast<float>(point.y()),
            static_cast<float>(point.z()));
        pts2d_curr.push_back(pts_curr_[i]);
    }
}

/**
 * @brief 构建当前帧的特征点参考数据
 *
 * 根据当前正在跟踪的特征点 active_points_ 及其对应 ID active_ids_，
 * 构建特征点的二维像素坐标和三维空间坐标，用于后续参考帧匹配、
 * PnP 位姿估计等处理。
 *
 * 对每个有效跟踪特征点：
 * 1. 将其像素坐标保存到 pixels，建立 id -> 2D 像素坐标映射；
 * 2. 在当前帧深度图中查询该像素位置的鲁棒深度；
 * 3. 若深度有效，则通过相机模型将像素点反投影到相机坐标系，
 *    并保存到 points3d，建立 id -> 3D 坐标映射。
 *
 * 若某个特征点无法获得有效深度，该特征点仍会保留在 pixels 中，
 * 但不会加入 points3d。
 *
 * @param frame     当前帧数据，使用其中的深度图 frame.depth 获取特征点深度
 * @param points3d  输出的特征点三维坐标，key 为特征点 ID，坐标位于相机坐标系
 * @param pixels    输出的特征点二维像素坐标，key 为特征点 ID
 */
void FeatureTracker::buildReferenceData(
    Frame& frame,
    std::unordered_map<int, cv::Point3f>& points3d,
    std::unordered_map<int, cv::Point2f>& pixels)
{
    points3d.clear();
    pixels.clear();

    if (active_points_.size() != active_ids_.size())
    {
        KR_WARN("Cannot build reference data: points={} ids={}", active_points_.size(), active_ids_.size());
        return ;
    }

    for (std::size_t i = 0; i < active_points_.size(); ++i)
    {
        const int id = active_ids_[i];
        const cv::Point2f& pixel = active_points_[i];

        const std::optional<double> depth = queryRobustDepth(frame.depth, pixel);
        if (!depth)
        {
            continue;
        }

        pixels[id] = pixel;
        
        const Eigen::Vector3d point = pointcloud_.pixel2camera(pixel.x, pixel.y, *depth);
        points3d[id] = cv::Point3f(
            static_cast<float>(point.x()),
            static_cast<float>(point.y()),
            static_cast<float>(point.z()));
    }
}

void FeatureTracker::get3d2dFromReference(
    const std::unordered_map<int, cv::Point3f>& reference_points3d, 
    std::vector<cv::Point3f>& pts3d_reference,
    std::vector<cv::Point2f>& pts2d_current) const
{
    pts3d_reference.clear();
    pts2d_current.clear();

    if (active_points_.size() != active_ids_.size())
    {
        return;
    }

    for (std::size_t i = 0; i < active_points_.size(); ++i)
    {
        const auto it = reference_points3d.find(active_ids_[i]);
        if (it == reference_points3d.end())
        {
            continue;
        }

        pts3d_reference.push_back(it->second);
        pts2d_current.push_back(active_points_[i]);
    }
}

void FeatureTracker::reset()
{
    active_points_.clear();
    active_ids_.clear();
    active_track_counts_.clear();
    pts_last_.clear();
    pts_curr_.clear();
    tracked_features_.clear();
    next_feature_id_ = 0;
}
