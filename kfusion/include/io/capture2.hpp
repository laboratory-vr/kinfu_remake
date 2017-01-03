#pragma once

#include <kfusion/kinfu.hpp>
#include <opencv2/core/core.hpp>
#include <string>

namespace kfusion
{
    class KF_EXPORTS OpenNI2Source
    {
    public:
        typedef kfusion::PixelRGB RGB24;
        
        enum FRAME_TYPE {
            NON = 0,
            DEPTH,
            COLOR
        };

        OpenNI2Source();
        OpenNI2Source(const std::string& device_uri);
        //OpenNI2Source(const std::string& oni_filename, bool repeat = false);

        bool open(const std::string& device_uri);
        //bool open(const std::string& oni_filename, bool repeat = false);
        void release();

        ~OpenNI2Source();

        size_t grab(cv::Mat &depth, cv::Mat &image);

        //parameters taken from camera/oni
        int shadow_value, no_sample_value;
        //float depth_focal_length_VGA;
        float baseline;               // mm
        double pixelSize;             // mm
        unsigned short max_depth;     // mm

        bool setRegistration (bool value = false);
    private:
        struct Impl;
        cv::Ptr<Impl> impl_;
        void getParams ();

    };
}
