#include "VO.hpp"

VO::VO(
    Camera& camera,
    Config& config
)
:
camera_(camera),
tracker_(camera, config)
{

}


bool VO::processFrame(Frame& frame)
{
    if (first_frame)
    {
        frame_ = frame;
        first_frame=false;
        return true;
    }

    tracker_.detect_klt(frame_, frame);

    //3d-2d
    tracker_.get3d2d(frame_, pts3d_last_, pts2d_curr_);
    if(pts3d_last_.size()<10)
        {
            frame_=frame;
            return false;
        }

    //pnp
    cv::Mat K = camera_.K();
    cv::Mat dist = cv::Mat::zeros(5,1,CV_64F);
    cv::Mat rvec;
    cv::Mat tvec;
    bool success = cv::solvePnPRansac(pts3d_last_, pts2d_curr_, K, dist, rvec, tvec );
    
    if(success)
    {
        std::cout<<"PnP success"<<std::endl;
        std::cout <<" 3D:" << pts3d_last_.size() << " 2D:" << pts2d_curr_.size() <<std::endl;
    }

    //更新pose


    frame_ = frame;

    return true;
}