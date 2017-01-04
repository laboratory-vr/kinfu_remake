#pragma once

#include <string>

namespace kfusion
{
    struct Point;
}

std::string gen_next_filename(const std::string& filebase, const std::string& ext, const int& index);

inline bool is_path_exists (const std::string& name);

inline bool is_file_exists (const std::string& name); //{
//    std::ifstream f(name.c_str());
//    return f.good();
//}

std::string fetch_filename(const std::string& filebase, const std::string& ext);

int saveOBJFile (const std::string &file_name, const kfusion::Point* cloud, const kfusion::Normal* normal, unsigned cloud_size, unsigned precision = 6);

int savePLYFile (const std::string &file_name, const kfusion::Point* cloud, const kfusion::Normal* normal, unsigned cloud_size, unsigned precision = 6, const std::string& format = "ascii");
