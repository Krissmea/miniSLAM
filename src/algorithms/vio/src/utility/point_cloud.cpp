#include "point_cloud.hpp"


//生成点云，将某个帧的深度图转换为三维点云
std::vector<Eigen::Vector3d> PointCloud::generate(DepthImageData& depth, double depth_scale)
{
    std::vector<Eigen::Vector3d> points3d;

    for (int v = 0; v < depth.image.rows; v++)
    {
        for (int u = 0; u < depth.image.cols; u++)
        {
            uint16_t d = depth.image.at<uint16_t>(v,u);
            if (d == 0)
                continue;
            
            double depth_m = d / depth_scale;

            Eigen::Vector3d p = pixel2camera(u, v, depth_m);
            points3d.push_back(p);
        }
    }
    return points3d;
}

//将某个像素坐标转换为3D坐标
Eigen::Vector3d PointCloud::pixel2camera(double u, double v, double depth)
{
    Eigen::Vector3d point;
    point[2] = depth;
    point[0] = (u - g_cx) * depth / g_fx;
    point[1] = (v - g_cy) * depth / g_fy;

    return point;

}
