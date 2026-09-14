#include "config.hpp"

#include <krisea_log/logger.hpp>
#include <opencv2/core.hpp>

#include <cmath>
#include <filesystem>

std::string g_input_mode = "rgbd";
std::string g_rgb_topic = "/camera/color/image_raw";
std::string g_depth_topic = "/camera/depth/image_raw";
std::string g_left_topic = "/camera/left_ir/image_raw";
std::string g_right_topic = "/camera/right_ir/image_raw";
std::string g_imu_topic = "/camera/gyro_accel/sample";
double g_input_max_time_diff = 0.03;

double g_rgbd_max_time_diff = 0.03;
double g_stereo_max_time_diff = 0.005;
double g_min_depth_meters = 0.2;
double g_max_depth_meters = 8.0;
int g_sgbm_min_disparity = 0;
int g_sgbm_num_disparities = 128;
int g_sgbm_block_size = 5;
int g_sgbm_uniqueness_ratio = 10;
int g_sgbm_speckle_window_size = 100;
int g_sgbm_speckle_range = 2;
int g_sgbm_disp12_max_diff = 1;
bool SHOW_KLT_TRACKING = false;
bool SHOW_TRAJECTORY = false;

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

int g_stereo_image_width = 0;
int g_stereo_image_height = 0;
cv::Mat g_stereo_left_K;
cv::Mat g_stereo_left_D;
cv::Mat g_stereo_right_K;
cv::Mat g_stereo_right_D;
cv::Mat g_R_right_left;
cv::Mat g_t_right_left;

namespace
{

bool matrixHasShape(const cv::Mat& matrix, int rows, int cols)
{
    return !matrix.empty() && matrix.rows == rows && matrix.cols == cols &&
           cv::checkRange(matrix);
}

void readString(const cv::FileNode& node, const char* key, std::string& value)
{
    const cv::FileNode item = node[key];
    if (!item.empty()) item >> value;
}

void readInt(const cv::FileNode& node, const char* key, int& value)
{
    const cv::FileNode item = node[key];
    if (!item.empty()) item >> value;
}

void readDouble(const cv::FileNode& node, const char* key, double& value)
{
    const cv::FileNode item = node[key];
    if (!item.empty()) item >> value;
}

bool loadCommonConfig(
    const std::string& main_filename,
    const cv::FileNode& filename_node)
{
    if (filename_node.empty())
    {
        return true;
    }

    std::string filename;
    filename_node >> filename;
    std::filesystem::path path(filename);
    if (path.is_relative())
    {
        path = std::filesystem::path(main_filename).parent_path() / path;
    }
    path = path.lexically_normal();

    cv::FileStorage fs(path.string(), cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        KR_ERROR("Cannot open common config: {}", path.string());
        return false;
    }

    const cv::FileNode sync = fs["sync"];
    readDouble(sync, "rgbd_max_time_diff", g_rgbd_max_time_diff);
    readDouble(sync, "stereo_max_time_diff", g_stereo_max_time_diff);

    const cv::FileNode depth = fs["depth"];
    readDouble(depth, "min_meters", g_min_depth_meters);
    readDouble(depth, "max_meters", g_max_depth_meters);

    const cv::FileNode matcher = fs["stereo_matcher"];
    readInt(matcher, "min_disparity", g_sgbm_min_disparity);
    readInt(matcher, "num_disparities", g_sgbm_num_disparities);
    readInt(matcher, "block_size", g_sgbm_block_size);
    readInt(matcher, "uniqueness_ratio", g_sgbm_uniqueness_ratio);
    readInt(matcher, "speckle_window_size", g_sgbm_speckle_window_size);
    readInt(matcher, "speckle_range", g_sgbm_speckle_range);
    readInt(matcher, "disp12_max_diff", g_sgbm_disp12_max_diff);

    const cv::FileNode visualization = fs["visualization"];
    if (!visualization.empty())
    {
        visualization["show_klt_tracking"] >> SHOW_KLT_TRACKING;
        visualization["show_trajectory"] >> SHOW_TRAJECTORY;
    }

    if (g_rgbd_max_time_diff <= 0.0 || g_stereo_max_time_diff <= 0.0 ||
        g_min_depth_meters <= 0.0 ||
        g_max_depth_meters <= g_min_depth_meters)
    {
        KR_ERROR("Invalid sync or depth settings in common config: {}", path.string());
        return false;
    }

    KR_INFO("Loaded common config: {}", path.string());
    return true;
}

bool loadInputConfig(const cv::FileNode& node)
{
    if (!node.empty())
    {
        readString(node, "mode", g_input_mode);
        readString(node, "rgb_topic", g_rgb_topic);
        readString(node, "depth_topic", g_depth_topic);
        readString(node, "left_topic", g_left_topic);
        readString(node, "right_topic", g_right_topic);
        readString(node, "imu_topic", g_imu_topic);
    }

    if (g_input_mode != "rgbd" && g_input_mode != "stereo")
    {
        KR_ERROR("Unsupported input.mode: {} (expected rgbd or stereo)", g_input_mode);
        return false;
    }
    g_input_max_time_diff = stereoInputEnabled()
        ? g_stereo_max_time_diff
        : g_rgbd_max_time_diff;
    return true;
}

bool loadStereoCalibration(const cv::FileNode& node)
{
    if (node.empty())
    {
        KR_ERROR("input.mode=stereo requires a stereo section");
        return false;
    }

    readInt(node, "image_width", g_stereo_image_width);
    readInt(node, "image_height", g_stereo_image_height);
    node["left_K"] >> g_stereo_left_K;
    node["left_D"] >> g_stereo_left_D;
    node["right_K"] >> g_stereo_right_K;
    node["right_D"] >> g_stereo_right_D;
    node["R_right_left"] >> g_R_right_left;
    node["t_right_left"] >> g_t_right_left;

    std::string reason;
    if (!validateStereoConfig(reason))
    {
        KR_ERROR("Invalid stereo calibration: {}", reason);
        return false;
    }
    return true;
}

}  // namespace

bool stereoInputEnabled()
{
    return g_input_mode == "stereo";
}

bool validateStereoConfig(std::string& reason)
{
    if (g_stereo_image_width <= 0 || g_stereo_image_height <= 0)
    {
        reason = "image size must be positive";
        return false;
    }
    if (!matrixHasShape(g_stereo_left_K, 3, 3) ||
        !matrixHasShape(g_stereo_right_K, 3, 3) ||
        !matrixHasShape(g_R_right_left, 3, 3))
    {
        reason = "left_K, right_K and R_right_left must be finite 3x3 matrices";
        return false;
    }
    if (g_stereo_left_D.empty() || g_stereo_right_D.empty() ||
        !cv::checkRange(g_stereo_left_D) ||
        !cv::checkRange(g_stereo_right_D))
    {
        reason = "left_D and right_D must be finite distortion vectors";
        return false;
    }
    if (g_t_right_left.empty() || g_t_right_left.total() != 3 ||
        !cv::checkRange(g_t_right_left))
    {
        reason = "t_right_left must contain three finite values in meters";
        return false;
    }

    cv::Mat translation;
    g_t_right_left.convertTo(translation, CV_64F);
    if (cv::norm(translation) <= 1e-6)
    {
        reason = "stereo baseline must be non-zero";
        return false;
    }
    if (g_sgbm_num_disparities <= 0 || g_sgbm_num_disparities % 16 != 0)
    {
        reason = "stereo_matcher.num_disparities must be a positive multiple of 16";
        return false;
    }
    if (g_sgbm_block_size <= 0 || g_sgbm_block_size % 2 == 0)
    {
        reason = "stereo_matcher.block_size must be a positive odd number";
        return false;
    }
    reason.clear();
    return true;
}

bool loadConfig(const std::string& filename)
{
    KR_INFO("Loading config: {}", filename);
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        KR_ERROR("Cannot open config: {}", filename);
        return false;
    }

    if (!loadCommonConfig(filename, fs["common_config"]) ||
        !loadInputConfig(fs["input"]))
    {
        return false;
    }
    if (stereoInputEnabled() && !loadStereoCalibration(fs["stereo"]))
    {
        return false;
    }

    const cv::FileNode camera = fs["camera"];
    camera["fx"] >> g_fx;
    camera["fy"] >> g_fy;
    camera["cx"] >> g_cx;
    camera["cy"] >> g_cy;
    camera["depth_scale"] >> g_depth_scale;
    g_K = (cv::Mat_<double>(3, 3) <<
        g_fx, 0.0, g_cx,
        0.0, g_fy, g_cy,
        0.0, 0.0, 1.0);

    g_dataset_path.clear();
    const cv::FileNode dataset = fs["dataset"];
    if (!dataset.empty()) dataset["path"] >> g_dataset_path;

    const cv::FileNode extrinsic = fs["extrinsic"];
    if (!extrinsic.empty())
    {
        extrinsic["rgb_frame"] >> g_rgb_frame;
        extrinsic["imu_frame"] >> g_imu_frame;
        extrinsic["T_rgb_imu"] >> g_T_rgb_imu;
    }
    if (g_T_rgb_imu.empty() || g_T_rgb_imu.rows != 4 ||
        g_T_rgb_imu.cols != 4)
    {
        KR_ERROR("Invalid T_rgb_imu in config: {}", filename);
        return false;
    }

    if (!g_dataset_path.empty())
    {
        std::filesystem::path dataset_path(g_dataset_path);
        if (dataset_path.is_relative())
        {
            dataset_path = std::filesystem::path(filename).parent_path() /
                dataset_path;
        }
        g_dataset_path = dataset_path.lexically_normal().string();
    }

    KR_INFO(
        "Input config: mode={} max_time_diff={:.3f}s",
        g_input_mode, g_input_max_time_diff);
    return true;
}
