#ifndef CONFIG_H
#define CONFIG_H

#include <string>
#include <opencv2/opencv.hpp>



class Config
{
public:
    bool load(const std::string &filename);
    double fx() const;
    double fy() const;
    double cx() const;
    double cy() const;
    double depthScale() const;
    std::string datasetPath() const;

private:
    double fx_ = 0;
    double fy_ = 0;
    double cx_ = 0;
    double cy_ = 0;
    double depth_scale_ = 5000.0;
    std::string dataset_path_;
};

#endif