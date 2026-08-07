#ifndef FEATURE_TRACKER_H
#define FEATURE_TRACKER_H

#include <common/frame.hpp>
#include <opencv2/opencv.hpp>
#include "config.hpp"
#include "point_cloud.hpp"


class FeatureTracker
{
public:

    void detect_klt(Frame& frame_last, Frame& frame_curr);
    void get3d2d(Frame& frame_last, std::vector<cv::Point3f>& pts3d_last, std::vector<cv::Point2f>& pts2d_curr);
    const std::vector<cv::Point2f>& ptsLast() const { return pts_last_;}
    const std::vector<cv::Point2f>& ptsCurr() const { return pts_curr_;}

private:
    std::vector<cv::Point2f> pts_last_;
    std::vector<cv::Point2f> pts_curr_;
    
    PointCloud pointcloud_;
};


#endif 
