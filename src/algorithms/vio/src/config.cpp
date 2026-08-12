#include <filesystem>
#include <opencv2/opencv.hpp>
#include <krisea_log/logger.hpp>
#include "config.hpp"

double g_fx = 0.0;
double g_fy = 0.0;
double g_cx = 0.0;
double g_cy = 0.0;
cv::Mat g_K;
double g_depth_scale = 5000.0;
std::string g_dataset_path;
std::string g_rgb_frame = "camera_color_optical_frame";
std::string g_imu_frame = "camera_accel_gyro_optical_frame";
cv::Mat g_T_rgb_imu = cv::Mat::eye(4, 4, CV_64F);

bool loadConfig(const std::string& filename)
{
    KR_INFO("Loading config: {}", filename);
        
    cv::FileStorage fs(filename, cv::FileStorage::READ);

    if (!fs.isOpened())
    {
        KR_ERROR("Cannot open config: {}", filename);
        return false;
    }

    fs["camera"]["fx"] >> g_fx;
    fs["camera"]["fy"] >> g_fy;
    fs["camera"]["cx"] >> g_cx;
    fs["camera"]["cy"] >> g_cy;
    fs["camera"]["depth_scale"] >> g_depth_scale;
    fs["dataset"]["path"] >> g_dataset_path;

    const cv::FileNode extrinsic = fs["extrinsic"];
    if (!extrinsic.empty())
    {
        extrinsic["rgb_frame"] >> g_rgb_frame;
        extrinsic["imu_frame"] >> g_imu_frame;
        extrinsic["T_rgb_imu"] >> g_T_rgb_imu;
    }

    if (g_T_rgb_imu.empty() || g_T_rgb_imu.rows != 4 || g_T_rgb_imu.cols != 4)
    {
        KR_ERROR("Invalid T_rgb_imu in config: {}", filename);
        return false;
    }

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
