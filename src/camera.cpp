#include "camera.hpp"


Camera::Camera(double fx, double fy, double cx, double cy):
    fx_(fx), fy_(fy), cx_(cx), cy_(cy)
    {
        
    }

Eigen::Vector3d Camera::pixel2camera(double u, double v, double depth)
{
    Eigen::Vector3d point;
    point[2] = depth;
    point[0] = (u - cx_) * depth / fx_;
    point[1] = (v - cy_) * depth / fy_;

    return point;

}

cv::Mat Camera::K() const
{
    cv::Mat K =
    (cv::Mat_<double>(3,3)
        <<
        fx_, 0, cx_,
        0, fy_, cy_,
        0, 0, 1
    );

    return K;
}