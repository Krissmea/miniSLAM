#ifndef VIO_PLUGIN_H
#define VIO_PLUGIN_H

#include "estimator.hpp"
#include <common/frame.hpp>
#include <common/image_data.hpp>
#include <common/imu_data.hpp>
#include <common/posedata.hpp>
#include <functional>


class VioPlugin
{
public:
    VioPlugin() = default;
    ~VioPlugin() = default;

    using PoseCallback = std::function<void(const PoseData&)>;

    // Input functions only cache data. Matching/solving happens in process().
    void inputImage(ImageData image);
    void inputDepth(DepthImageData depth);
    void inputImu(ImuData imu);

    void process();
    void setPoseCallback(PoseCallback callback);
    void reset();

private:
    bool popMatchedFrame(Frame& frame);
    void publishPose(const EstimatorResult& result);
 
    static constexpr double kRgbDepthMaxTimeDiff = 0.03;
    static constexpr std::size_t kMaxImageBufferSize = 100;
 
    std::deque<ImageData> rgb_buf_;
    std::deque<DepthImageData> depth_buf_;
    std::mutex buffer_mutex_;
    int next_frame_id_{0};
 
    Estimator estimator_;
    PoseCallback pose_callback_;
    
};




#endif
