#ifndef VIO_PLUGIN_H
#define VIO_PLUGIN_H

#include "estimator.hpp"
#include <common/frame.hpp>
#include <common/imu_data.hpp>
#include <common/posedata.hpp>
#include <functional>


class VioPlugin
{
public:
    VioPlugin() = default;
    ~VioPlugin() = default;

    using PoseCallback = std::function<void(const PoseData&)>;

    // bool initialize(const std::string& comfig_path);
    void inputFrame(Frame& frame);
    void inputImu(const ImuData& imu);
    void setPoseCallback(PoseCallback callback);
    void reset();

private:
    // FeatureTracker tracker_;
    Estimator estimator_;
    PoseCallback pose_callback_;
    
};




#endif
