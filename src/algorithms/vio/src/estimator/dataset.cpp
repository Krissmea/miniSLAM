#include <fstream>
#include <iostream>
#include <sstream>
#include <opencv2/opencv.hpp>
#include "dataset.hpp"


void Dataset::getRgbData(std::ifstream& stream, const std::string& path )
{
    std::string line;
    while (std::getline(stream, line))
    {
        if(line.empty() || line[0]=='#')
            continue;
        
        std::stringstream ss(line);

        double timestamp;
        std::string filename;

        ss >> timestamp >>  filename;

        ImageData rgb;
        rgb.timestamp = timestamp;
        rgb.filename = path + "/" + filename;

        rgb_buf_.push_back(rgb);
    }
}

void Dataset::getDepthData(std::ifstream& stream, const std::string& path )
{
    std::string line;
    while (std::getline(stream, line))
    {
        if(line.empty() || line[0]=='#')
            continue;
        
        std::stringstream ss(line);

        double timestamp;
        std::string filename;

        ss >> timestamp >>  filename;

        ImageData depth;
        depth.timestamp = timestamp;
        depth.filename = path + "/" + filename;

        depth_buf_.push_back(depth);
    }
}

Dataset::Dataset(const std::string& path)
{
    std::string rgb_files = path + "/rgb.txt";
    std::string depth_files = path + "/depth.txt";

    std::ifstream rgb_stream(rgb_files);
    if (!rgb_stream.is_open())
    {
        std::cerr << "cannot open" << rgb_files << std::endl;

        return;
    }
    getRgbData(rgb_stream, path);

    std::ifstream depth_stream(depth_files);
    if (!depth_stream.is_open())
    {
        std::cerr << "cannot open" << depth_files << std::endl;
        return;
    }
    getDepthData(depth_stream, path);

    std::cout << "load rgb" << rgb_buf_.size() << std::endl;
    std::cout << "load depth" << depth_buf_.size() << std::endl;

}


bool Dataset::next(Frame& frame)
{
    if(current_index_ >= rgb_buf_.size())
    {
        return false;
    }

    ImageData rgb_data = rgb_buf_[current_index_];
    ImageData depth_data = depth_buf_[current_index_];

    frame.rgb = cv::imread(rgb_data.filename, cv::IMREAD_COLOR);
    frame.depth = cv::imread(depth_data.filename, cv::IMREAD_UNCHANGED);
    frame.timestamp = rgb_data.timestamp;
    frame.id = current_index_;
    current_index_++;
    return true;
}

