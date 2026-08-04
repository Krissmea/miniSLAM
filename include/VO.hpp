#ifndef VO_H
#define VO_H

#include <Eigen/Geometry>
#include "frame.hpp"
#include "feature_tracker.hpp"
#include "camera.hpp"
#include "config.hpp"



class VO
{
public:
    VO(Camera& camera, Config& config); 
    bool processFrame(Frame& frame);

private:
    FeatureTracker tracker_;
    Camera camera_;
    Frame frame_;
    Config config_;
    Eigen::Isometry3d pose_;
    bool first_frame = true;

    std::vector<cv::Point3f> pts3d_last_;
    std::vector<cv::Point2f> pts2d_curr_;

    cv::Mat K_;
    cv::Mat dist_;

    
};


#endif