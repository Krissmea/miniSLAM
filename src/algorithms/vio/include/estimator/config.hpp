#ifndef CONFIG_H
#define CONFIG_H

#include <opencv2/core.hpp>

#include <string>

// Selected input and ROS topics.
extern std::string g_input_mode;
extern std::string g_rgb_topic;
extern std::string g_depth_topic;
extern std::string g_left_topic;
extern std::string g_right_topic;
extern std::string g_imu_topic;
extern double g_input_max_time_diff;

// Common functional settings.
extern double g_rgbd_max_time_diff;
extern double g_stereo_max_time_diff;
extern double g_min_depth_meters;
extern double g_max_depth_meters;
extern int g_sgbm_min_disparity;
extern int g_sgbm_num_disparities;
extern int g_sgbm_block_size;
extern int g_sgbm_uniqueness_ratio;
extern int g_sgbm_speckle_window_size;
extern int g_sgbm_speckle_range;
extern int g_sgbm_disp12_max_diff;
extern bool SHOW_KLT_TRACKING;
extern bool SHOW_TRAJECTORY;

// Active camera model used by tracking/PnP.
extern double g_fx;
extern double g_fy;
extern double g_cx;
extern double g_cy;
extern cv::Mat g_K;
extern double g_depth_scale;

// Dataset and IMU extrinsic.
extern std::string g_dataset_path;
extern std::string g_rgb_frame;
extern std::string g_imu_frame;
extern cv::Mat g_T_rgb_imu;

// Orbbec stereo calibration.
extern int g_stereo_image_width;
extern int g_stereo_image_height;
extern cv::Mat g_stereo_left_K;
extern cv::Mat g_stereo_left_D;
extern cv::Mat g_stereo_right_K;
extern cv::Mat g_stereo_right_D;
extern cv::Mat g_R_right_left;
extern cv::Mat g_t_right_left;

bool loadConfig(const std::string& filename);
bool stereoInputEnabled();
bool validateStereoConfig(std::string& reason);

#endif
