#ifndef CAMERA_H
#define CAMERA_H

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

class Camera
{
public:
    Camera(double fx, double fy, double cx, double cy);
    Eigen::Vector3d pixel2camera(double u, double v, double depth);
    cv::Mat K() const;

private:
    double fx_;
    double fy_;
    double cx_;
    double cy_;

};





#endif 
