#include "feature_tracker.hpp"


FeatureTracker::FeatureTracker(
    Camera& camera,
    Config& config
)
:
camera_(camera),
config_(config)
{

}

//光流追踪
void FeatureTracker::detect_klt(Frame& frame_last, Frame& frame_curr)
{
    pts_last_.clear();
    pts_curr_.clear();

    if (frame_last.rgb.empty() || frame_curr.rgb.empty())
    {
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
    cv::goodFeaturesToTrack(gray_last, corners_last, 300, 0.01, 10.0, cv::Mat(), 7, false, 0.04);
    if (corners_last.size() <= 20)
    {
        std::cerr << "Too few features: " << corners_last.size() << std::endl;
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

    double total_motion = 0.0;

    for (size_t i = 0; i < corners_last.size(); ++i)
    {
        if (!status_forward[i] && !status_back[i])
        {
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

        const double motion = cv::norm(p_last - p_curr);
        if (motion > 0.2)
        {
            continue;
        }

        pts_last_.push_back(p_last);
        pts_curr_.push_back(p_curr);
        total_motion += motion;
    }

    if (!pts_last_.empty())
    {
        std::cout << "Motion:" <<  total_motion / pts_last_.size();
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

        float depth = frame_last.depth.at<uint16_t>(v, u);
        if (depth == 0)
            continue;
        
        Eigen::Vector3d p = camera_.pixel2camera(u,v, depth / config_.depthScale());

        pts3d_last.push_back(cv::Point3f(p.x(),p.y(),p.z()));
        pts2d_curr.push_back(pts_curr_[i]);

    }
}
