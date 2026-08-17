#ifndef CONFIG_H
#define CONFIG_H

#include <string>
#include <opencv2/opencv.hpp>

extern double g_fx;
extern double g_fy;
extern double g_cx;
extern double g_cy;
extern cv::Mat g_K;
extern double g_depth_scale;
extern std::string g_dataset_path;
extern std::string g_rgb_frame;
extern std::string g_imu_frame;
extern cv::Mat g_T_rgb_imu;
extern bool SHOW_KLT_TRACKING;

bool loadConfig(const std::string& filename);

#endif
