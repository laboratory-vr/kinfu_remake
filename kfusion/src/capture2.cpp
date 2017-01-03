#pragma warning (disable :4996)
#undef _CRT_SECURE_NO_DEPRECATE

#include <OpenNI.h>
#include <OniVersion.h>

#include <io/capture2.hpp>

using namespace std;

#define REPORT_ERROR(msg) kfusion::cuda::error ((msg), __FILE__, __LINE__)

struct kfusion::OpenNI2Source::Impl
{
    openni::Device device;
    openni::VideoStream depth;
    openni::VideoStream color;
    char strError[1024];
    float width;
    float height;
    
    openni::VideoFrameRef	depthFrame;
    openni::VideoFrameRef	colorFrame;

    bool has_depth;
    bool has_image;

};

kfusion::OpenNI2Source::OpenNI2Source() :
    shadow_value (0), no_sample_value (0), baseline (0.f), pixelSize (0.0), max_depth (0) {}

kfusion::OpenNI2Source::OpenNI2Source(const string& device_uri ) { open (device_uri); }
//kfusion::OpenNI2Source::OpenNI2Source(const string& filename, bool repeat /*= false*/) {open (filename, repeat); }
kfusion::OpenNI2Source::~OpenNI2Source() { release (); }

bool kfusion::OpenNI2Source::open (const string& device_uri)
{
    impl_ = cv::Ptr<Impl>( new Impl () );
    
    openni::Status rc = openni::STATUS_OK;
    
    const char* deviceURI = openni::ANY_DEVICE;
    
    rc = openni::OpenNI::initialize();
    
    sprintf( impl_->strError, "After initialization:\n%s\n", openni::OpenNI::getExtendedError() );
    printf( impl_->strError );
    
    rc = impl_->device.open( deviceURI ); //device_uri.c_str()
    if (rc != openni::STATUS_OK)
    {
        sprintf( impl_->strError, "openni2 interface: Device open failed:\n%s\n", openni::OpenNI::getExtendedError() );
        REPORT_ERROR( impl_->strError );
        
        openni::OpenNI::shutdown();
        return false;
    }
    
    
    impl_->has_depth = false;
    rc = impl_->depth.create( impl_->device, openni::SENSOR_DEPTH);
    if (rc == openni::STATUS_OK)
    {
        rc = impl_->depth.start();
        if (rc != openni::STATUS_OK)
        {
            sprintf( impl_->strError, "openni2 interface: Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError() );
            REPORT_ERROR( impl_->strError );
            impl_->depth.destroy();
            
        }
        else impl_->has_depth = true; impl_->depth.setMirroringEnabled(true);
    }
    else
    {
        sprintf( impl_->strError, "openni2 interface: Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError() );
        REPORT_ERROR( impl_->strError );
    }
    
    impl_->has_image = false;
    rc = impl_->color.create( impl_->device, openni::SENSOR_COLOR);
    if (rc == openni::STATUS_OK)
    {
        rc = impl_->color.start();
        if (rc != openni::STATUS_OK)
        {
            sprintf( impl_->strError, "openni2 interface: Couldn't start color stream:\n%s\n", openni::OpenNI::getExtendedError() );
            REPORT_ERROR(impl_->strError);
            impl_->color.destroy();
            
        }
        else impl_->has_image = true; impl_->color.setMirroringEnabled(true);
    }
    else
    {
        sprintf( impl_->strError, "openni2 interface: Couldn't find color stream:\n%s\n", openni::OpenNI::getExtendedError() );
        REPORT_ERROR(impl_->strError);
    }
    
    if (!impl_->depth.isValid() || !impl_->color.isValid())
    {
        REPORT_ERROR("openni2 interface: No valid streams. Exiting\n");
        openni::OpenNI::shutdown();
        return false;
    }
    
    ////////////////////////
    /*
    openni::VideoMode depthVideoMode;
    openni::VideoMode colorVideoMode;
    
    if (impl_->depth.isValid() && impl_->color.isValid())
    {
        depthVideoMode = impl_->depth.getVideoMode();
        colorVideoMode = impl_->color.getVideoMode();
        
        //getPixelFormat
        int depthWidth  = depthVideoMode.getResolutionX();
        int depthHeight = depthVideoMode.getResolutionY();
        int colorWidth  = colorVideoMode.getResolutionX();
        int colorHeight = colorVideoMode.getResolutionY();
        
        if (depthWidth == colorWidth &&
            depthHeight == colorHeight)
        {
            impl_->width =  depthWidth;
            impl_->height = depthHeight;
        }
        else
        {
            sprintf(impl_->strError,
                    "Error - expect color and depth to be in same resolution: D: %dx%d, C: %dx%d\n",
                   depthWidth, depthHeight,
                   colorWidth, colorHeight);
            REPORT_ERROR(impl_->strError);
            return openni::STATUS_ERROR;
        }
    }
    else if (impl_->depth.isValid())
    {
        depthVideoMode = impl_->depth.getVideoMode();
        impl_->width = depthVideoMode.getResolutionX();
        impl_->height = depthVideoMode.getResolutionY();
    }
    else if (impl_->color.isValid())
    {
        colorVideoMode = impl_->color.getVideoMode();
        impl_->width = colorVideoMode.getResolutionX();
        impl_->height = colorVideoMode.getResolutionY();
    }
    else
    {
        REPORT_ERROR("Error - expects at least one of the streams to be valid...\n");
        return false;
    }
    */
    getParams ();
    
    return rc == openni::STATUS_OK;
}


//bool kfusion::OpenNI2Source::open(const std::string& filename, bool repeat /*= false*/){}

void kfusion::OpenNI2Source::release ()
{
    if (impl_)
    {
        impl_->depthFrame.release();
        impl_->colorFrame.release();
        
        impl_->depth.stop();
        impl_->depth.destroy();
        
        impl_->color.stop();
        impl_->color.destroy();
        
        impl_->device.close();
        
        openni::OpenNI::shutdown();
    }

    impl_.release();
    baseline = 0.f;
    shadow_value = 0;
    no_sample_value = 0;
    pixelSize = 0.0;
}

void fillDepth(const openni::VideoFrameRef& src,  cv::Mat& dst)
{
    const void* pDepth = src.getData();
    int x = src.getWidth();
    int y = src.getHeight();
    cv::Mat(y, x, CV_16U, (void*)pDepth).copyTo(dst);
}

void fillColor(const openni::VideoFrameRef& src,  cv::Mat& dst)
{
    const void* pColor = src.getData();
    int x = src.getWidth ();
    int y = src.getHeight ();
    //cv::Mat(y, x, CV_8UC3, (void*)pColor).copyTo(dst);
    
    dst.create(y, x, CV_8UC3);
    char * pColor2 = (char *)pColor;
    
    cv::Vec3b *dptr = dst.ptr<cv::Vec3b>();
    for(size_t i = 0; i < dst.total(); ++i)
        dptr[i] = cv::Vec3b(pColor2[i*3+2], pColor2[i*3+1], pColor2[i*3]);
    /**/
}

size_t kfusion::OpenNI2Source::grab(cv::Mat& depth, cv::Mat& image)
{
    openni::Status rc = openni::STATUS_OK;
    
    openni::VideoStream ** pStreams = new openni::VideoStream*[2];
    pStreams[0] = &impl_->depth;
    pStreams[1] = &impl_->color;
    
    // Texture map init
    //m_nTexMapX = MIN_CHUNKS_SIZE(m_width, TEXTURE_SIZE);
    //m_nTexMapY = MIN_CHUNKS_SIZE(m_height, TEXTURE_SIZE);
    //m_pTexMap = new openni::RGB888Pixel[m_nTexMapX * m_nTexMapY];
    
    //openni::VideoStream::setMirroringEnabled
    //openni::VideoStream::addNewFrameListener
    //openni::VideoStream::start
    //ref: https://github.com/OpenNI/OpenNI2/blob/master/Samples/EventBasedRead/main.cpp
    
    int changedIndex;
    rc = openni::OpenNI::waitForAnyStream(pStreams, 2, &changedIndex);
    if (rc != openni::STATUS_OK)
    {
        sprintf( impl_->strError, "openni2 interface: Read failed:\n%s\n", openni::OpenNI::getExtendedError() );
        REPORT_ERROR(impl_->strError);
        return 0;
    }
    /*
    impl_->depth.readFrame(&impl_->depthFrame); fillDepth(impl_->depthFrame, depth);
    impl_->color.readFrame(&impl_->colorFrame); fillColor(impl_->colorFrame, image);
    return 2;
    */
    switch (changedIndex)
    {
        case 0:
            impl_->depth.readFrame(&impl_->depthFrame); fillDepth(impl_->depthFrame, depth); //return changedIndex;
        case 1:
            impl_->color.readFrame(&impl_->colorFrame); fillColor(impl_->colorFrame, image); return changedIndex;
        default:
            REPORT_ERROR("Error in wait\n");
            return -1;
    }

    return -1;
}

void kfusion::OpenNI2Source::getParams ()
{

}

bool kfusion::OpenNI2Source::setRegistration (bool value)
{
    openni::Status rc = openni::STATUS_OK;
    openni::ImageRegistrationMode _mode = value? openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR : openni::IMAGE_REGISTRATION_OFF;

    rc = impl_->device.setImageRegistrationMode(_mode);
    if ( rc != openni::STATUS_OK )
    {
        sprintf( impl_->strError, "openni2 interface: SetRegistration failed: \n%s\n", openni::OpenNI::getExtendedError() );
        REPORT_ERROR(impl_->strError);
    }

    getParams ();
    return rc == openni::STATUS_OK;
}
