#include <iostream>
#include "VO.hpp"
#include <krisea_log/logger.hpp>



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

            KR_WARN("Not enough 3D-2D correspondences: {}", pts3d_last_.size());
            frame_=frame;
            return false;
        }

    //pnp
    cv::Mat K = g_K;
    cv::Mat dist = cv::Mat::zeros(5,1,CV_64F);
    cv::Mat rvec;
    cv::Mat tvec;
    bool success = cv::solvePnPRansac(pts3d_last_, pts2d_curr_, K, dist, rvec, tvec );
    
    if(success)
    {
        cv::Mat R_cv;
        cv::Rodrigues(rvec, R_cv);

        Eigen::Matrix3d R;
        for (int row = 0; row < 3; ++row)
        {
            for (int col = 0; col < 3; ++col)
            {
                R(row, col) = R_cv.at<double>(row, col);
            }
        }

        Eigen::Vector3d t(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
        pose_.linear() = R.transpose();
        pose_.translation() = -R.transpose() * t;

        std::cout<<"PnP success"<<std::endl;
        std::cout <<" 3D:" << pts3d_last_.size() << " 2D:" << pts2d_curr_.size() <<std::endl;
    }
    else
    {
        frame_ = frame;
        return false;
    }

    //更新pose


    frame_ = frame;

    return true;
}

const Eigen::Isometry3d& VO::pose() const
{
    return pose_;
}
