#pragma once

#include <kfusion/types.hpp>
#include <opencv2/core/core.hpp>
//#include <kfusion/kinfu.hpp>
//#include <opencv2/core/core.hpp>

//#include <iostream>
#include <string>

std::string gen_next_filename(const std::string& filebase, const std::string& ext, const int& index);

inline bool is_file_exists (const std::string& name) {
    std::ifstream f(name.c_str());
    return f.good();
}

std::string fetch_filename(const std::string& filebase, const std::string& ext);

int saveOBJFile (const std::string &file_name, const kfusion::Point* cloud, const kfusion::Normal* normal, unsigned cloud_size, unsigned precision = 6);
//int saveOBJFile (const std::string &file_name, const cv::Mat& cloud, unsigned precision);

int savePLYFile (const std::string &file_name, const kfusion::Point* cloud, const kfusion::Normal* normal, unsigned cloud_size, unsigned precision = 6, const std::string& format = "ascii");
