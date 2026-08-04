#include <iostream>
#include "config.hpp"

bool Config::load(const std::string& filename)
{
    std::cout 
        << "load config: "
        << filename
        << std::endl;
        
    cv::FileStorage fs(filename, cv::FileStorage::READ);

    if (!fs.isOpened())
    {
        std::cerr << "cannot open config" << filename << std::endl;
        return false;
    }

    fs["camera"]["fx"] >> fx_;
    fs["camera"]["fy"] >> fy_;
    fs["camera"]["cx"] >> cx_;
    fs["camera"]["cy"] >> cy_;
    fs["camera"]["depth_scale"] >> depth_scale_;
    fs["dataset"]["path"] >> dataset_path_;


    fs.release();

    return true;

}

double Config::fx() const
{
    return fx_;
}

double Config::fy() const
{
    return fy_;
}

double Config::cx() const
{
    return cx_;
}

double Config::cy() const
{
    return cy_;
}

double Config::depthScale() const
{
    return depth_scale_;
}

std::string Config::datasetPath() const
{
    return dataset_path_;
}
