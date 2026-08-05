#include "pose.hpp"
#include <opencv2/opencv.hpp>

bool Pose::posePath(const PoseData& poseData)
{
    position_.push_back(poseData.position);
    orientation_.push_back(poseData.orientation);

    return true;
}

void Pose::pathShow(const PoseData& poseData)
{
    cv::Mat canvas = cv::Mat::zeros(480, 640, CV_8UC3);

    auto poses = poseData.position;

    double scale = 50.0; // Scale factor for visualization

    for (size_t i = 1; i < poses.size(); ++i)
    {
        cv::Point2d p1(poses[i - 1].position(0) * scale + canvas.cols / 2, poses[i - 1].position(2) * scale + canvas.rows / 2);      
        cv::Point2d p2(poses[i].position(0) * scale + canvas.cols / 2, poses[i].position(2) * scale + canvas.rows / 2);

        cv::line(canvas, p1, p2, cv::Scalar(0, 255, 0), 2);
    }

    cv::imshow("Pose Path", canvas);
    cv::waitKey(1);
    
}