#include "feature_tracker.hpp"
#include <krisea_log/logger.hpp>
#include "config.hpp"


// FeatureTracker::FeatureTracker(
    
// )
// :
// pointcloud_;
// {

// }

//光流追踪
void FeatureTracker::detect_klt(Frame& frame_last, Frame& frame_curr)
{
    pts_last_.clear();
    pts_curr_.clear();

    if (frame_last.rgb.empty() || frame_curr.rgb.empty())
    {
        // KR_WARN("Empty frame images, skipping KLT tracking.");
        return;
    }

    cv::Mat gray_last, gray_curr;
    if (frame_last.rgb.channels() == 3)
    {
        cv::cvtColor(frame_last.rgb, gray_last, cv::COLOR_BGR2GRAY);
        cv::cvtColor(frame_curr.rgb, gray_curr, cv::COLOR_BGR2GRAY);
    }
    else
    {
        gray_last = frame_last.rgb.clone();
        gray_curr = frame_curr.rgb.clone();
    }

    std::vector<cv::Point2f> corners_last;
    cv::goodFeaturesToTrack(gray_last, corners_last, 150, 0.01, 10.0, cv::Mat(), 7, false, 0.04);
    if (corners_last.size() <= 20)
    {
        // KR_ERROR("Too few features detected in the last frame: {}", corners_last.size());
        return;
    }

    std::vector<cv::Point2f> corners_curr;
    std::vector<uchar> status_forward;
    std::vector<float> error_forward;

    cv::calcOpticalFlowPyrLK(gray_last, 
        gray_curr, 
        corners_last, 
        corners_curr, 
        status_forward, 
        error_forward, 
        cv::Size(21, 21), 
        3, 
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01));
    
    std::vector<cv::Point2f> corners_back;
    std::vector<uchar> status_back;
    std::vector<float> error_back;

    cv::calcOpticalFlowPyrLK(gray_curr, 
        gray_last, 
        corners_curr, 
        corners_back, 
        status_back, 
        error_back,
        cv::Size(21, 21),
        3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01));

    //==================== 光流结果可视化 ====================
    if (SHOW_KLT_TRACKING)
    {
        cv::Mat last_vis;
        cv::Mat curr_vis;

        if (gray_last.channels() == 1)
            cv::cvtColor(gray_last, last_vis, cv::COLOR_GRAY2BGR);
        else
            last_vis = gray_last.clone();

        if (gray_curr.channels() == 1)
            cv::cvtColor(gray_curr, curr_vis, cv::COLOR_GRAY2BGR);
        else
            curr_vis = gray_curr.clone();

        int valid_count = 0;

        for (size_t i = 0; i < corners_last.size(); ++i)
        {
            if (!status_forward[i])
            continue;

        const cv::Point2f& pt_last = corners_last[i];
        const cv::Point2f& pt_curr = corners_curr[i];

        // 可选：过滤明显异常的光流
        float dx = pt_curr.x - pt_last.x;
        float dy = pt_curr.y - pt_last.y;
        float distance = std::sqrt(dx * dx + dy * dy);

        if (distance > 100.0f)
            continue;

        ++valid_count;

        // 上一帧特征点：红色
        cv::circle(
            last_vis,
            pt_last,
            3,
            cv::Scalar(0, 0, 255),
            -1,
            cv::LINE_AA);

        // 当前帧跟踪点：绿色
        cv::circle(
            curr_vis,
            pt_curr,
            3,
            cv::Scalar(0, 255, 0),
            -1,
            cv::LINE_AA);
        }


        // 标题
        cv::putText(
            last_vis,
            "Last frame",
            cv::Point(20, 30),
            cv::FONT_HERSHEY_SIMPLEX,
            0.7,
            cv::Scalar(0, 255, 255),
            2);

        cv::putText(
            curr_vis,
            "Current frame",
            cv::Point(20, 30),
            cv::FONT_HERSHEY_SIMPLEX,
            0.7,
            cv::Scalar(0, 255, 255),
            2);


        // 并排拼接
        cv::Mat combined;
        cv::hconcat(last_vis, curr_vis, combined);


        // ==================== 在并排图上画匹配连线 ====================

        const int x_offset = last_vis.cols;

        for (size_t i = 0; i < corners_last.size(); ++i)
        {
            if (!status_forward[i])
                continue;

            const cv::Point2f& pt_last = corners_last[i];
            const cv::Point2f& pt_curr = corners_curr[i];

            float dx = pt_curr.x - pt_last.x;
            float dy = pt_curr.y - pt_last.y;
            float distance = std::sqrt(dx * dx + dy * dy);

            if (distance > 100.0f)
            continue;

            // 当前帧在 combined 里面的坐标需要增加 x 偏移
            cv::Point2f pt_curr_offset(
                pt_curr.x + x_offset,
                pt_curr.y);

            // 蓝色连线表示匹配关系
            cv::line(
                combined,
                pt_last,
                pt_curr_offset,
                cv::Scalar(255, 0, 0),
                1,
                cv::LINE_AA);
        }


        // 显示有效跟踪数量
        cv::putText(
            combined,
            "Tracked: " + std::to_string(valid_count) +
            "/" + std::to_string(corners_last.size()),
            cv::Point(20, combined.rows - 20),
            cv::FONT_HERSHEY_SIMPLEX,
            0.7,
            cv::Scalar(0, 255, 255),
            2);

        cv::imshow("Optical Flow: Last | Current", combined);

        cv::waitKey(1);
    }
    //==================== 光流可视化结束 ====================

    double total_motion = 0.0;

    for (size_t i = 0; i < corners_last.size(); ++i)
    {
        if (!status_forward[i] )
        {
            // KR_WARN("Feature {} lost during tracking.", i);
            continue;
        }

        const cv::Point2f &p_last = corners_last[i];
        const cv::Point2f &p_curr = corners_curr[i];
        const cv::Point2f &p_back = corners_back[i];

        if (p_curr.x < 0 || p_curr.y < 0 || p_curr.x >= gray_curr.cols || p_curr.y >= gray_curr.rows)
        {
            continue;
        }

        const double fb_error = cv::norm(p_last - p_back);
        if (fb_error > 1.0)
        {
            continue;
        }

        // KR_INFO("Feature {} tracked successfully: Last ({:.2f}, {:.2f}) -> Current ({:.2f}, {:.2f})", i, p_last.x, p_last.y, p_curr.x, p_curr.y);
        const double motion = cv::norm(p_last - p_curr);
        // if (motion < 0.01)
        // {
        //     continue;
        // }
        // KR_WARN("Feature {} tracked successfully: Last ({:.2f}, {:.2f}) -> Current ({:.2f}, {:.2f})", i, p_last.x, p_last.y, p_curr.x, p_curr.y);


        pts_last_.push_back(p_last);
        pts_curr_.push_back(p_curr);
        total_motion += motion;
    }
    double avg_motion = pts_curr_.empty() ? 0.0 : total_motion / pts_curr_.size();
    // KR_INFO("Tracked {} features, Average motion: {:.4f}", pts_curr_.size(), avg_motion);

    if (pts_last_.empty())
    {
        KR_WARN("Tracked {} features from last frame to current frame.", pts_last_.size());
    }

}

void FeatureTracker::get3d2d(Frame& frame_last, std::vector<cv::Point3f>& pts3d_last, std::vector<cv::Point2f>& pts2d_curr)

{
    pts3d_last.clear();
    pts2d_curr.clear();

    for (size_t i = 0; i < pts_last_.size(); i++)
    {
        int u = static_cast<int>(pts_last_[i].x);
        int v = static_cast<int>(pts_last_[i].y);

        if (frame_last.depth.empty() || frame_last.depth.type() != CV_16UC1 ||
            u < 0 || v < 0 || u >= frame_last.depth.cols || v >= frame_last.depth.rows)
        {
            continue;
        }

        float depth = frame_last.depth.at<uint16_t>(v, u);
        if (depth == 0)
            continue;
        
        Eigen::Vector3d p = pointcloud_.pixel2camera(u,v, depth / g_depth_scale);
        // KR_ERROR("3D point: ({}, {}, {})", p.x(), p.y(), p.z());

        pts3d_last.push_back(cv::Point3f(p.x(),p.y(),p.z()));
        pts2d_curr.push_back(pts_curr_[i]);

    }
}
