#include <iostream>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include "config.hpp"

double g_fx = 0.0;
double g_fy = 0.0;
double g_cx = 0.0;
double g_cy = 0.0;
cv::Mat g_K;
double g_depth_scale = 5000.0;
std::string g_dataset_path;

bool loadConfig(const std::string& filename)
{
    std::cout 
        << "load config: "
        << filename
        << std::endl;
        
    cv::FileStorage fs(filename, cv::FileStorage::READ);

    if (!fs.isOpened())
    {
        std::cerr << "cannot open config " << filename << std::endl;
        return false;
    }

    fs["camera"]["fx"] >> g_fx;
    fs["camera"]["fy"] >> g_fy;
    fs["camera"]["cx"] >> g_cx;
    fs["camera"]["cy"] >> g_cy;
    fs["camera"]["depth_scale"] >> g_depth_scale;
    fs["dataset"]["path"] >> g_dataset_path;

    g_K = (cv::Mat_<double>(3, 3) << g_fx, 0, g_cx, 0, g_fy, g_cy, 0, 0, 1);

    std::filesystem::path dataset_path(g_dataset_path);
    if (dataset_path.is_relative())
    {
        std::filesystem::path config_dir = std::filesystem::path(filename).parent_path();
        g_dataset_path = (config_dir / dataset_path).lexically_normal().string();
    }


    fs.release();

    return true;

}
