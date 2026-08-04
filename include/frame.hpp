#ifndef FRAME_H
#define FRAME_H


#include <opencv2/opencv.hpp>
#include <Eigen/Core>

struct Frame
{
    int id;
    double timestamp;
    cv::Mat rgb;
    cv::Mat depth;

    //当前帧三维点
    std::vector<Eigen::Vector3d> points3d;

    //特征点
    std::vector<cv::Point2f> keypoints;
};

#endif 
