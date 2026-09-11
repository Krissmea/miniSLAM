#include "estimator.hpp"

#include "config.hpp"

#include <krisea_log/logger.hpp>
#include <opencv2/calib3d.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <iterator>
#include <utility>
#include <vector>

namespace
{

constexpr std::size_t kMinCorrespondences = 30;
constexpr int kMinInliers = 20;
constexpr double kMinInlierRatio = 0.5;
constexpr double kMaxMedianReprojectionError = 2.5;
constexpr double kMaxTranslationPerFrame = 0.5;
constexpr double kMaxRotationRadians = 20.0 * CV_PI / 180.0;

constexpr int kMaxFramesBetweenKeyframes = 10;
constexpr double kKeyframeParallaxPixels = 15.0;
constexpr std::size_t kMinKeyframeCommonFeatures = 60;
constexpr int kMaxFramesBetweenKeyFrames = 10;

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
        if (index < 0 || index >= static_cast<int>(object_points.size()))
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

void Estimator::inputFrame(Frame frame)
{
    insertByTimestamp(frame_buf_, std::move(frame));
}

void Estimator::inputImu(ImuData imu)
{
    imu_stream_seen_ = true;
    insertByTimestamp(imu_buf_, std::move(imu));
}

bool Estimator::imuCoversFrame(double frame_timestamp) const
{
    if (!imu_stream_seen_)
    {
        return true;
    }

    if (imu_buf_.empty())
    {
        return false;
    }

    return imu_buf_.back().timestamp >= frame_timestamp;
}

std::vector<ImuData> Estimator::collectImuForFrame(double frame_timestamp) const
{
    std::vector<ImuData> samples;
    if (!imu_stream_seen_)
    {
        return samples;
    }

    for (const auto& imu : imu_buf_)
    {
        if (imu.timestamp > frame_timestamp)
        {
            break;
        }
        samples.push_back(imu);
    }

    return samples;
}


void Estimator::pruneImuBefore(double frame_timestamp)
{
    if (imu_buf_.empty())
    {
        return;
    }

    const auto first_after = std::upper_bound(
        imu_buf_.begin(), imu_buf_.end(), frame_timestamp,
        [](double timestamp, const ImuData& imu) {
            return timestamp < imu.timestamp;
        });

    if (first_after == imu_buf_.begin())
    {
        return;
    }

    const auto keep = std::prev(first_after);
    imu_buf_.erase(imu_buf_.begin(), keep);
}

std::optional<EstimatorResult> Estimator::process()
{
    while (!frame_buf_.empty())
    {
        const double frame_timestamp = frame_buf_.front().timestamp;
        if (!imuCoversFrame(frame_timestamp))
        {
            return std::nullopt;
        }

        Frame frame = std::move(frame_buf_.front());
        frame_buf_.pop_front();

        const auto imu_samples = collectImuForFrame(frame_timestamp);
        const bool solved = solveFrame(frame, imu_samples);

        last_processed_frame_timestamp_ = frame_timestamp;
        pruneImuBefore(frame_timestamp);

        if (!solved)
        {
            continue;
        }

        EstimatorResult result;
        result.timestamp = frame_timestamp;
        result.pose = pose_;
        return result;
    }

    return std::nullopt;
}

void Estimator::updateReference(RefFrame& reference, Frame& frame, const Eigen::Isometry3d& pose)
{
    reference.frame = frame;
    reference.pose = pose;
    tracker_.buildReferenceData(frame, reference.points3d, reference.pixels);
    reference.valid = true;
}

bool Estimator::referenceUsable(const RefFrame& reference) const
{
    return reference.valid && reference.points3d.size() >= kMinCorrespondences;
}


//求当前相机相对于 reference 的位姿，这个reference可能是上一有效位姿帧，也可能是关键帧
Estimator::PnPResult Estimator::solvePnPFromReference(const RefFrame& reference)
{
    PnPResult result;
    if (!referenceUsable(reference))
    {
        result.failure_reason = "reference_unusable";
        return result;
    }

    tracker_.get3d2dFromReference(reference.points3d, pts3d_reference_, pts2d_current_);
    result.matches = static_cast<int>(pts3d_reference_.size());
    if (pts3d_reference_.size() < kMinCorrespondences)
    {
        return result;
    }

    cv::Mat dist = cv::Mat::zeros(5, 1, CV_64F);
    cv::Mat rvec;
    cv::Mat tvec;
    cv::Mat inliers;
    
    const bool success = cv::solvePnPRansac(
        pts3d_reference_,
        pts2d_current_,
        g_K,
        dist,
        rvec,
        tvec,
        false,
        100,
        3.0,
        0.99,
        inliers,
        cv::SOLVEPNP_ITERATIVE);
    
    result.inliers = inliers.rows;
    result.inlier_ratio = result.matches > 0 ? static_cast<double>(result.inliers) / result.matches : 0.0;

    if (!success || result.inliers < kMinInliers || result.inlier_ratio < kMinInlierRatio)
    {
        return result;
    }

    result.med_reprojection_error = medianReprojectionError(pts3d_reference_, pts2d_current_, inliers, rvec, tvec, g_K, dist);
    if (!std::isfinite(result.med_reprojection_error) || result.med_reprojection_error > kMaxMedianReprojectionError)
    {
        return result;
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
        tvec.at<double>(0),
        tvec.at<double>(1),
        tvec.at<double>(2));

    result.T_ref_curr = Eigen::Isometry3d::Identity();
    result.T_ref_curr.linear() = R.transpose();
    result.T_ref_curr.translation() = -R.transpose() * t;

    if (!result.T_ref_curr.matrix().allFinite())
    {
        return result;
    }

    result.success = true;
    return result;
}

/**
 * @brief 判断当前候选位姿是否可信
 *
 * 将当前候选位姿与最近一次可信位姿进行比较，计算两者之间的相对平移和旋转。
 * 如果平移距离或旋转角度超过允许阈值，则认为当前候选位姿存在异常跳变，
 * 拒绝该位姿更新。
 *
 * 考虑到中间可能存在视觉位姿更新失败的情况，阈值会根据当前帧与最近一次
 * 可信位姿之间的时间间隔进行动态放宽，避免多帧间隔导致正常运动被误判。
 *
 * @param candidate_pose     当前待验证的候选全局位姿
 * @param current_timestamp  当前帧时间戳，单位为秒
 *
 * @return true  候选位姿满足运动约束，可以接受
 * @return false 候选位姿存在非法值或运动跳变，不应更新当前位姿
 */
bool Estimator::validCandidatePose(const Eigen::Isometry3d& candidate_pose, double current_timestamp) const
{
    if (!candidate_pose.matrix().allFinite())
    {
        return false;
    }

    if (!last_pose_reference_.valid)
    {
        return true;
    }

    const Eigen::Isometry3d delta = last_pose_reference_.pose.inverse() * candidate_pose;
    const double translation = delta.translation().norm();
    const double rotation = Eigen::AngleAxisd(delta.rotation()).angle();

    // 当前阈值按照 10 Hz 的逐帧处理频率设置
    // 当中间跳过了一帧或多帧视觉更新时，需要按时间间隔对阈值进行放宽。
    constexpr double kNominalFramePeriodSeconds = 0.1;
    const double dt = std::max(current_timestamp - last_pose_reference_.frame.timestamp, kNominalFramePeriodSeconds);
    const double scale = std::max(1.0, dt / kNominalFramePeriodSeconds);

    const double max_translation = kMaxTranslationPerFrame * scale;
    const double max_rotation = kMaxRotationRadians * scale;

    if (translation > max_translation || rotation > max_rotation)
    {
        KR_WARN("Rejecting candidate pose: dt={:.3f}s " "translation={:.3f}/{:.3f}m " "rotation={:.3f}/{:.3f}deg",
            dt, translation, max_translation, rotation * 180.0 / CV_PI, max_rotation * 180.0 / CV_PI);
        return false;
    }
    return true;

}

/**
 * @brief 计算当前帧相对于关键帧的特征点中值视差
 *
 * 根据持续跟踪的特征点 ID，在当前帧和关键帧之间查找公共特征，
 * 计算每个公共特征在两帧中的像素位移，并取所有有效视差的中值，
 * 用于判断当前帧与关键帧之间的视觉变化程度。
 *
 * 同时通过 common_feature_count 返回当前帧与关键帧之间的公共特征数量。
 *
 * @param common_feature_count 当前帧与关键帧之间的公共有效特征数量
 * @return 中值视差，单位为像素；无有效公共特征时返回 0.0
 */
double Estimator::calculateParallax(std::size_t& common_feature_count) const
{
    common_feature_count = 0;
    if (!keyframe_reference_.valid || keyframe_reference_.pixels.empty())
    {
        return 0.0;
    }
    //当前帧中仍处于有效跟踪状态的特征点 ID
    const auto& ids = tracker_.activeIds();
    //与上述特征点 ID 一一对应的当前帧像素坐标
    const auto& points = tracker_.activePoints();
    if (ids.size() != points.size())
    {
        return 0.0;
    }

    //通过特征点的视差中位数来判断
    std::vector<double> parallaxes;
    parallaxes.reserve(points.size());

    for (std::size_t i = 0; i < ids.size(); ++i)
    {
        const auto it = keyframe_reference_.pixels.find(ids[i]);
        if (it == keyframe_reference_.pixels.end())
        {
            continue;
        }

        const double parallax = cv::norm(points[i] - it->second);
        if (!std::isfinite(parallax))
        {
            continue;
        }

        parallaxes.push_back(parallax);
    } 

    common_feature_count = parallaxes.size();
    if (parallaxes.empty())
    {
        return 0.0;
    }

    const auto middle = parallaxes.begin() + parallaxes.size() / 2;
    std::nth_element(parallaxes.begin(), middle, parallaxes.end());
    
    return *middle;
}

/**
 * @brief 判断当前帧是否需要创建为新的关键帧
 *
 * 根据当前关键帧状态、关键帧 PnP 是否成功、当前帧相对关键帧的中值视差、
 * 公共特征数量以及距离上一关键帧的帧数，综合判断是否需要更新关键帧。
 *
 * 触发新关键帧的条件包括：
 * 1. 当前还没有有效关键帧；
 * 2. 当前关键帧 PnP 失败，但 last-pose fallback PnP 成功；
 * 3. 当前帧相对关键帧的中值视差超过阈值；
 * 4. 当前帧与关键帧之间的公共特征数量过少；
 * 5. 距离上一关键帧的帧数达到最大允许间隔。
 *
 * @param median_parallax 当前帧相对于关键帧的中值视差，单位为像素
 * @param common_feature_count 当前帧与关键帧之间的公共特征数量
 * @param keyframe_pnp_success 当前帧使用关键帧作为参考进行 PnP 是否成功
 *
 * @return true  当前帧应该创建为新的关键帧
 * @return false 继续使用当前关键帧
 */
bool Estimator::isKeyFrame(double median_parallax, std::size_t common_feature_count, bool keyframe_pnp_success) const
{
    // 当前还没有有效关键帧时，当前帧直接作为新的关键帧
    if (!keyframe_reference_.valid)
    {
        return true;
    }

    // 当前关键帧已经无法稳定完成 PnP，
    // 但 last-pose fallback 能够恢复当前位姿时，
    // 说明原关键帧已经不适合作为后续参考，需要及时刷新关键帧。
    if (!keyframe_pnp_success)
    {
        return true;
    }

    // 当前帧相对于关键帧的视差已经足够大，
    // 说明两帧之间已经产生明显运动，需要建立新的关键帧。
    if (median_parallax >= kKeyframeParallaxPixels)
    {
        return true;
    }

    // 当前帧与关键帧之间仍然能够持续跟踪的公共特征过少，
    // 说明两帧的共视关系正在变差，需要更新关键帧。
    if (common_feature_count < kMinKeyframeCommonFeatures)
    {
        return true;
    }

    // 即使运动较小、公共特征仍然充足，
    // 也不能长期不更新关键帧，因此达到最大帧间隔后强制创建新关键帧。
    return frames_since_keyframe_ >= kMaxFramesBetweenKeyframes;
}

bool Estimator::solveFrame(
    Frame& frame,
    const std::vector<ImuData>& imu_samples)
{
    // The timestamp-aligned IMU segment is intentionally owned by Estimator.
    // Preintegration/fusion can be added here without changing Plugin I/O.
    (void)imu_samples;

    if (first_frame_)
    {
        if (!tracker_.initialize(frame))
        {
            KR_WARN("Failed to initialize visual tracker");
            return false;
        }
        pose_ = Eigen::Isometry3d::Identity();
        last_frame_ = frame;
        
        updateReference(last_pose_reference_, frame, pose_);
        updateReference(keyframe_reference_, frame, pose_);

        frames_since_keyframe_ = 0;
        consecutive_pnp_failures_ = 0;
        first_frame_ = false;

        KR_INFO("[Keyframe] Initialize id={} timestamp={:.6f} " "features={} depth_points={}",
            keyframe_id_, frame.timestamp,
            keyframe_reference_.pixels.size(), keyframe_reference_.points3d.size());
        ++keyframe_id_;

        return true; 
    }

    // 1. 状态按帧推进，保证KLT特征跟踪连续
    tracker_.detect_klt(last_frame_, frame);
    last_frame_ = frame;

    if (tracker_.trackedFeatures().size() < kMinCorrespondences)
    {
        ++consecutive_pnp_failures_;
        KR_WARN(
            "[Tracking] Too few persistent tracks: {} < {}, "
            "PnP failures={}",
            tracker_.trackedFeatures().size(),
            kMinCorrespondences,
            consecutive_pnp_failures_);
        return false;
    }

    bool pose_valid = false;
    bool keyframe_pnp_success = false;
    const char* pnp_source = "none";
    PnPResult accepted_result;
    Eigen::Isometry3d candidate_pose = pose_;

    // Primary: keyframe -> current.
    KR_WARN("referenceUsable(keyframe_reference_){}",referenceUsable(keyframe_reference_));
    if (referenceUsable(keyframe_reference_))
    {
        PnPResult result = solvePnPFromReference(keyframe_reference_);
        if (result.success)
        {
            const Eigen::Isometry3d candidate =
                keyframe_reference_.pose * result.T_ref_curr;

            if (validCandidatePose(candidate, frame.timestamp))
            {
                candidate_pose = candidate;
                accepted_result = result;
                pose_valid = true;
                keyframe_pnp_success = true;
                pnp_source = "keyframe";
            }
        }
    }

    // Fallback: latest trusted pose frame -> current.
    const bool same_reference =
        keyframe_reference_.valid && last_pose_reference_.valid &&
        std::abs(
            keyframe_reference_.frame.timestamp -
            last_pose_reference_.frame.timestamp) < 1e-9;

    if (!pose_valid &&
        referenceUsable(last_pose_reference_) &&
        !same_reference)
    {
        PnPResult result = solvePnPFromReference(last_pose_reference_);
        if (result.success)
        {
            const Eigen::Isometry3d candidate =
                last_pose_reference_.pose * result.T_ref_curr;

            if (validCandidatePose(candidate, frame.timestamp))
            {
                candidate_pose = candidate;
                accepted_result = result;
                pose_valid = true;
                pnp_source = "last_pose";
            }
        }
    }

    if (!pose_valid)
    {
        ++consecutive_pnp_failures_;
        KR_WARN("[PnP] Failed: keyframe_ref={} last_pose_ref={} " "tracked={} consecutive={}",
            keyframe_reference_.points3d.size(), last_pose_reference_.points3d.size(),
            tracker_.trackedFeatures().size(), consecutive_pnp_failures_);
        return false;
    }

    // Only a trusted PnP result may update the pose layer.
    pose_ = candidate_pose;
    consecutive_pnp_failures_ = 0;

    updateReference(last_pose_reference_, frame, pose_);

    ++frames_since_keyframe_;

    std::size_t common_feature_count = 0;
    const double median_parallax = calculateParallax(common_feature_count);

    const bool is_keyframe = isKeyFrame(median_parallax, common_feature_count, keyframe_pnp_success);

    KR_INFO(
        "[Visual] source={} matches={} inliers={} ratio={:.3f} "
        "reprojection={:.3f}px parallax={:.2f}px "
        "keyframe_common={} keyframe={} position=[{:.3f},{:.3f},{:.3f}]",
        pnp_source,
        accepted_result.matches,
        accepted_result.inliers,
        accepted_result.inlier_ratio,
        accepted_result.med_reprojection_error,
        median_parallax,
        common_feature_count,
        is_keyframe,
        pose_.translation().x(),
        pose_.translation().y(),
        pose_.translation().z());

    if (is_keyframe)
    {
        RefFrame new_keyframe;
        updateReference(new_keyframe, frame, pose_);
 
        if (referenceUsable(new_keyframe))
        {
            keyframe_reference_ = std::move(new_keyframe);
            frames_since_keyframe_ = 0;

            KR_INFO(
                "[Keyframe] Create id={} timestamp={:.6f} "
                "features={} depth_points={} source={}",
                keyframe_id_,
                frame.timestamp,
                keyframe_reference_.pixels.size(),
                keyframe_reference_.points3d.size(),
                pnp_source);
            ++keyframe_id_;
        }
        else
        {
            KR_WARN(
                "[Keyframe] Candidate rejected: only {} valid depth points",
                new_keyframe.points3d.size());
        }
    }
    return true;
}

const Eigen::Isometry3d& Estimator::pose() const
{
    return pose_;
}

void Estimator::reset()
{
    tracker_.reset();

    last_frame_ = Frame{};
    last_pose_reference_ = RefFrame{};
    keyframe_reference_ = RefFrame{};

    pose_ = Eigen::Isometry3d::Identity();
    first_frame_ = true;
    imu_stream_seen_ = false;
    last_processed_frame_timestamp_ = -1.0;

    frames_since_keyframe_ = 0;
    keyframe_id_ = 0;
    consecutive_pnp_failures_ = 0;
 
    frame_buf_.clear();
    imu_buf_.clear();

    pts3d_reference_.clear();
    pts2d_current_.clear();

    
}
