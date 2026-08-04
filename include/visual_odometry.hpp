#ifndef VISUAL_ODOMETRY_H
#define VISUAL_ODOMETRY_H

class VisualOdometry
{
public:

    void addFrame(Frame& frame);
    Eigen::Isometry3d pose();

private:

    Frame last_frame;
    Eigen::Isometry3d curr_pose;
    FeatureTracker tracker;
    Camera camera;
};

#endif 