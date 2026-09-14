#include "stereo/stereo_depth.hpp"
#include "vio_plugin.hpp"

#include <krisea_log/logger.hpp>

#include <cmath>
#include <iostream>

int main()
{
    KR_LOG_INIT(
        "stereo_depth_setup_test",
        "/tmp/krisea_stereo_depth_setup_test.log",
        1024 * 1024,
        1);

    if (!loadConfig(STEREO_TEST_CONFIG_PATH))
    {
        std::cerr << "stereo YAML loading failed\n";
        return 1;
    }
    if (!stereoInputEnabled() ||
        std::abs(g_input_max_time_diff - 0.005) > 1e-9)
    {
        std::cerr << "unexpected input configuration\n";
        return 2;
    }

    StereoDepth stereo;
    if (!stereo.initialize() || !stereo.ready())
    {
        std::cerr << "stereo initialization failed\n";
        return 3;
    }
    if (std::abs(stereo.baseline() - 0.0500326) > 1e-6)
    {
        std::cerr << "unexpected baseline: " << stereo.baseline() << '\n';
        return 4;
    }
    if (stereo.leftProjection().rows != 3 ||
        stereo.leftProjection().cols != 4 || !stereo.matcher())
    {
        std::cerr << "stereo products are incomplete\n";
        return 5;
    }

    constexpr int disparity_pixels = 16;
    const cv::Size image_size(g_stereo_image_width, g_stereo_image_height);
    cv::Mat left(image_size, CV_8UC1);
    cv::RNG random(12345);
    random.fill(left, cv::RNG::UNIFORM, 0, 255);
    cv::Mat right = cv::Mat::zeros(image_size, CV_8UC1);
    left(cv::Rect(
        disparity_pixels, 0,
        left.cols - disparity_pixels, left.rows)).copyTo(
            right(cv::Rect(
                0, 0, right.cols - disparity_pixels, right.rows)));

    cv::Mat rectified_left;
    cv::Mat depth_mm;
    if (!stereo.compute(left, right, rectified_left, depth_mm) ||
        depth_mm.type() != CV_16UC1 || cv::countNonZero(depth_mm) == 0)
    {
        std::cerr << "stereo depth generation failed\n";
        return 6;
    }

    VioPlugin vio;
    bool pose_received = false;
    vio.setPoseCallback([&pose_received](const PoseData&) {
        pose_received = true;
    });

    ImageData left_data;
    left_data.timestamp = 1.0;
    left_data.image = left;
    ImageData right_data;
    right_data.timestamp = 1.0;
    right_data.image = right;
    vio.inputLeftImage(std::move(left_data));
    vio.inputRightImage(std::move(right_data));
    vio.process();
    if (!pose_received)
    {
        std::cerr << "stereo frame did not reach estimator\n";
        return 7;
    }
    return 0;
}
