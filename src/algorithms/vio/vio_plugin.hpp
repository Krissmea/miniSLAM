#ifndef VIO_PLUGIN_H
#define VIO_PLUGIN_H

#include "config.hpp"
#include "estimator.hpp"
#include "stereo/stereo_depth.hpp"
#include <common/frame.hpp>
#include <common/image_data.hpp>
#include <common/imu_data.hpp>
#include <common/posedata.hpp>
#include <functional>
#include <deque>
#include <mutex>


class VioPlugin
{
public:
    VioPlugin();
    ~VioPlugin() = default;

    using PoseCallback = std::function<void(const PoseData&)>;

    // Input functions only cache data. Matching/solving happens in process().
    void inputImage(ImageData image);
    void inputDepth(DepthImageData depth);
    void inputLeftImage(ImageData image);
    void inputRightImage(ImageData image);
    void inputImu(ImuData imu);

    void process();
    void setPoseCallback(PoseCallback callback);
    void reset();
    bool ready() const { return ready_; }

private:
    bool popMatchedRgbdFrame(Frame& frame);
    bool popMatchedStereoFrame(Frame& frame);
    void publishPose(const EstimatorResult& result);
 
    static constexpr std::size_t kMaxImageBufferSize = 100;

    bool stereo_mode_{false};
    double max_time_diff_{0.03};
    bool ready_{true};
    std::deque<ImageData> rgb_buf_;
    std::deque<DepthImageData> depth_buf_;
    std::deque<ImageData> left_buf_;
    std::deque<ImageData> right_buf_;
    std::mutex buffer_mutex_;
    int next_frame_id_{0};
 
    StereoDepth stereo_depth_;
    Estimator estimator_;
    PoseCallback pose_callback_;
    
};




#endif
