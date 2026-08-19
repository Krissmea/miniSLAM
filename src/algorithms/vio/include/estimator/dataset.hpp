#ifndef DATASET_H
#define DATASET_H

#include <common/frame.hpp>
#include <common/posedata.hpp>
#include <string>
#include <vector>


struct DatasetsImageData
{
    double timestamp;
    std::string filename;
};


class Dataset
{

public:
    Dataset(const std::string& path);
    void getRgbData(std::ifstream& stream, const std::string& path );
    void getDepthData(std::ifstream& stream, const std::string& path );
    bool next(Frame& frame);
private:
    std::vector<DatasetsImageData> rgb_buf_;
    std::vector<DatasetsImageData> depth_buf_;

    int current_index_ = 0; //待梳理初值
};

#endif 