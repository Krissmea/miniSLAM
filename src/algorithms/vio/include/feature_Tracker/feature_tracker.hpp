#ifndef FEATURE_TRACKER_H
#define FEATURE_TRACKER_H

#include "point_cloud.hpp"
#include <common/frame.hpp>
#include <opencv2/core.hpp>
#include <unordered_map>
#include <vector>

struct TrackedFeature
{
    int id{-1};
    int track_count{0};
    cv::Point2f previous_pixel;
    cv::Point2f current_pixel;
};

class FeatureTracker
{
public:
    bool initialize(Frame& frame);
    void detect_klt(Frame& frame_last, Frame& frame_curr);
    void get3d2d(
        Frame& frame_last,
        std::vector<cv::Point3f>& pts3d_last,
        std::vector<cv::Point2f>& pts2d_curr);
    void reset();

    void buildReferenceData(
        Frame& frame,
        std::unordered_map<int, cv::Point3f>& points3d,
        std::unordered_map<int, cv::Point2f>& pixels);
    void get3d2dFromReference(
        const std::unordered_map<int, cv::Point3f>& reference_points3d,
        std::vector<cv::Point3f>& pts3d_reference,
        std::vector<cv::Point2f>& pts2d_current) const;

    const std::vector<cv::Point2f>& activePoints() const
    {
        return active_points_;
    }
    const std::vector<int>& activeIds() const
    {
        return active_ids_;
    }

    const std::vector<cv::Point2f>& ptsLast() const { return pts_last_; }
    const std::vector<cv::Point2f>& ptsCurr() const { return pts_curr_; }
    const std::vector<TrackedFeature>& trackedFeatures() const { return tracked_features_; }

private:
    static constexpr int kMaxFeatures = 200;                        //最多追踪点数
    static constexpr int kMinFeatureDistance = 30;                  //点之间最小距离
    static constexpr int kBorderSize = 2;                           //边界判断阈值，单位：像素
    static constexpr double kQualityLevel = 0.01;                   //质量水平？
    static constexpr double kForwardBackwardThreshold = 0.8;        //正反向光流阈值
    static constexpr double kFundamentalThreshold = 1.0;            //F矩阵剔除外点 单位像素
    static constexpr double kFundamentalConfidence = 0.99;          //F矩阵剔除外点置信度

    static bool toGray(const cv::Mat& image, cv::Mat& gray);
    static bool inBorder(const cv::Point2f& point, int cols, int rows);

    void initializeTracks(const cv::Mat& gray);
    void rejectWithFundamental(
        std::vector<cv::Point2f>& previous_points,
        std::vector<cv::Point2f>& current_points,
        std::vector<int>& ids,
        std::vector<int>& track_counts) const;
    cv::Mat selectSpatiallyDistributedTracks(
        const cv::Size& image_size,
        std::vector<cv::Point2f>& previous_points,
        std::vector<cv::Point2f>& current_points,
        std::vector<int>& ids,
        std::vector<int>& track_counts) const;
    void addNewFeatures(const cv::Mat& gray, const cv::Mat& mask);
    void visualizeTracks(
        const cv::Mat& previous_gray,
        const cv::Mat& current_gray) const;

    std::vector<cv::Point2f> active_points_;
    std::vector<int> active_ids_;
    std::vector<int> active_track_counts_;
    int next_feature_id_{0};

    std::vector<cv::Point2f> pts_last_;
    std::vector<cv::Point2f> pts_curr_;
    std::vector<TrackedFeature> tracked_features_;

    PointCloud pointcloud_;
};

#endif
