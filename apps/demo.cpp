#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/viz/vizcore.hpp>
#include <kfusion/kinfu.hpp>
#include <io/capture2.hpp>

//#include <kfusion/src/internal.hpp>

//#include <kfusion/types.hpp>

//#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include <boost/thread/thread.hpp>

#include "obj_writer.h"

using namespace kfusion;

struct KinFuApp
{
    static void KeyboardCallback(const cv::viz::KeyboardEvent& event, void* pthis)
    {
        KinFuApp& app = *static_cast<KinFuApp*>(pthis);

        if(event.action != cv::viz::KeyboardEvent::KEY_DOWN)
            return;
        
        app.KeyboardDispatcher(event.code);
    }
    
    static bool setIntrinsicsExternal( const std::string& calibration_file, kfusion::Intr& output )
    {
        bool bSet = false;
        try{
            using namespace boost::property_tree;
            ptree rootTree;
            xml_parser::read_xml( calibration_file, rootTree );
            
            ptree caliTree = rootTree.get_child("SystemCalibration0.CameraIntrinsics-depth");
            
            std::string id  = caliTree.get<std::string>( "<xmlattr>.id" );
            
            float fx        = caliTree.get<float>( "<xmlattr>.f_x" );
            float fy        = caliTree.get<float>( "<xmlattr>.f_y" );
            float cx        = caliTree.get<float>( "<xmlattr>.c_x" );
            float cy        = caliTree.get<float>( "<xmlattr>.c_y" );
            
            output.fx = fx;
            output.fy = fy;
            output.cx = cx;
            output.cy = cy;

            //std::cout << "intrinsic loaded: : " << output << std::endl;  // operator issue
            std::cout << "intrinsic loaded: focal( " << fx << ", " << fy << " ) princple point( " << cx << ", " << cy  << " )"<< std::endl;

            bSet = true;
        }
        catch(boost::property_tree::xml_parser::xml_parser_error& e){
            std::cout << "Depth intrinsics config file is not exist or invalid." << std::endl;
        }
        return bSet;
    }


    KinFuApp(OpenNI2Source& source, kfusion::Intr& intr) :  pause_(false), exit_ (false),
        iteractive_mode_(false), capture_ (source), compute_normals_(true)
    {
        KinFuParams params = KinFuParams::default_params();
        
        params.intr         = intr;//kfusion::Intr(366.736, 366.736, 259.41, 204.374); //640f * 480f
        //Vec3f volume_size   = Vec3f::all(1.2f);
        float size_seed = .6f;
        int   dim_seed  = (1 << 8);
        Vec3f volume_size   = Vec3f( size_seed * 3.5, size_seed * 2, size_seed * 1.5 );
        Vec3i volume_dims   = Vec3i( dim_seed  * 3.5, dim_seed  * 2, dim_seed  * 1.5 );
        params.volume_size  = volume_size;
        params.volume_pose  = Affine3f().translate(Vec3f(-volume_size[0]/2, -volume_size[1]/2, 0.7f));
        params.volume_dims  = volume_dims;
        
        kinfu_ = KinFu::Ptr( new KinFu(params) );

        capture_.setRegistration(true);
        capture_.setMirroring(false);
        
        cv::viz::WCube cube(cv::Vec3d::all(0), cv::Vec3d(params.volume_size), true, cv::viz::Color::apricot());
        viz_.showWidget("cube", cube, params.volume_pose);
        viz_.showWidget("coor", cv::viz::WCoordinateSystem(0.1));
        viz_.registerKeyboardCallback(KeyboardCallback, this);
        
        cam_poses_.reserve(3000);
    }

    void show_depth(const cv::Mat& depth)
    {
        cv::Mat display;
        //cv::normalize(depth, display, 0, 255, cv::NORM_MINMAX, CV_8U);
        depth.convertTo(display, CV_8U, 255.0/4000);
        cv::imshow("Depth", display);
    }
    
    void show_color(const cv::Mat& color)
    {
        //TODO: BGR to RGB
        cv::imshow("Color", color);
    }

    void show_raycasted(KinFu& kinfu)
    {
        const int mode = 3;
        if (iteractive_mode_)
            kinfu.renderImage(view_device_, viz_.getViewerPose(), mode);
        else
            kinfu.renderImage(view_device_, mode);

        view_host_.create(view_device_.rows(), view_device_.cols(), CV_8UC4);
        view_device_.download(view_host_.ptr<void>(), view_host_.step);
        cv::imshow("Scene", view_host_);
    }
    
    void take_cam_pose()
    {
        //std::vector<cv::Affine3f> until;
        //const std::vector<cv::Affine3f>& from = cam_poses_;
        //std::vector<cv::Affine3f>::const_iterator it = from.cbegin();
        //for ( ; it != from.cend(); ++it )
        //    until.push_back(*it);
        
        cv::Mat cloud_cam_pose  = cv::Mat( 1, (int)cam_poses_.size(), CV_32FC4 );
        Point*  cloud_cam_point = cloud_cam_pose.ptr<Point>();
        
        cv::Mat cloud_cam_norn  = cv::Mat( 1, (int)cam_poses_.size(), CV_32FC4 );
        Normal* cloud_cam_normal= cloud_cam_norn.ptr<Normal>();
        
        int idx = 0;
        std::vector<cv::Affine3f>::const_iterator it = cam_poses_.begin();
        for ( ; it != cam_poses_.end(); ++it, ++idx )
        {
            Point* pi = &cloud_cam_point[idx];
            Point* ni = &cloud_cam_normal[idx];
            cv::Vec3f t = it->translation();
            cv::Affine3f::Mat3 m = it->rotation();
            cv::Matx31f mi = m.col(2);
            memcpy( pi, &t, sizeof(t));
            memcpy( ni, &mi, sizeof(mi));
            //it->rotate()();
        }
        std::string filename    = fetch_filename("Affine","ply");
        savePLYFile ( filename, cloud_cam_point, cloud_cam_normal, cloud_cam_pose.total(), 6);
    }

    void take_cloud(KinFu& kinfu)
    {
        ScopeTime time("take_cloud");
        boost::lock_guard<boost::mutex> lock(cloud_mem_lock_);
        
        cuda::DeviceArray<Point> cloud = kinfu.tsdf().fetchCloud(cloud_buffer_);
        cloud_host_.create( 1, (int)cloud.size(), CV_32FC4 );
        cloud.download(cloud_host_.ptr<Point>());
        viz_.showWidget("cloud", cv::viz::WCloud(cloud_host_));
        std::cout << "[INFO] Done Fetch Cloud. Point Size: " << cloud.size() << std::endl;
      
        if (compute_normals_)
        {
            kinfu.tsdf().fetchNormals (cloud, normal_buffer_);
            normals_host_.create( 1, (int)cloud.size(), CV_32FC4 );
            normal_buffer_.download(normals_host_.ptr<Normal>());
        }
        take_cam_pose();
    }
    
    void save_cloud_async(std::string filename)
    {
        ScopeTime time("save_cloud_async");
        boost::lock_guard<boost::mutex> lock(cloud_mem_lock_);
        if (compute_normals_ && cloud_host_.total() == normals_host_.total() )
            savePLYFile ( filename, cloud_host_.ptr<Point>(), normals_host_.ptr<Normal>(), cloud_host_.total(), 6);
        else
            savePLYFile ( filename, cloud_host_.ptr<Point>(), 0, cloud_host_.total(), 6);
        std::cout << "[INFO] Complete saving Point cloud to " << filename << std::endl;
    }
    
    void save_cloud()
    {
        if ( cloud_host_.empty() ){
            std::cout << "[WARNING] Point cloud is empty." << std::endl;
            return ;
        }

        std::string filename    = fetch_filename("cloud","ply");
        std::cout << "[INFO] Started to save point cloud to " << filename << std::endl;

        boost::thread thrd( boost::bind(&KinFuApp::save_cloud_async, this, filename) );
        //thrd.join();
    }
    
    void KeyboardDispatcher(const unsigned char& key)
    {
        switch(key)
        {
            case 't': case 'T' : take_cloud(*kinfu_); break;
            case 'i': case 'I' : iteractive_mode_ = !iteractive_mode_; break;
            case 'n': case 'N' : compute_normals_ = !compute_normals_,
                std::cout << " compute_normals: " << compute_normals_ << std::endl; break;
            case 's': case 'S' : save_cloud(); break;
            case '0': case ')' : kinfu_->reset(); break;
                
            case 27: exit_ = true; break;       //ESC
            case 32: pause_ = !pause_; break;   //SPACE
        }
    }

    bool execute()
    {
        KinFu& kinfu = *kinfu_;
        cv::Mat depth, image;
        double time_ms = 0;
        bool has_image = false;

        for (int i = 0; !exit_ && !viz_.wasStopped() ; ++i) //
        {
            int frame_type = capture_.grab(depth, image);
            if (frame_type < 0)
                return std::cout << "Can't grab" << std::endl, false;
            
            if ( !image.empty() )
                show_color(image);

            if (depth_device_.empty())
                std::cout << "Depth image cols: " << depth.cols << " rows: "<< depth.rows << std::endl;
            depth_device_.upload(depth.data, depth.step, depth.rows, depth.cols);

            {
                SampledScopeTime fps(time_ms); (void)fps;
                has_image = kinfu(depth_device_);
            }

            if (has_image)
                show_raycasted(kinfu);
            else
                cam_poses_.clear(), cam_poses_.reserve(3000);

            show_depth(depth);
            //cv::imshow("Image", image);
            
            cv::Affine3f current_cam_pos = kinfu.getCameraPose();
            cam_poses_.push_back(current_cam_pos);
            
            if (!iteractive_mode_)
                viz_.setViewerPose(current_cam_pos);

            int key = cv::waitKey(pause_ ? 0 : 3);

            this->KeyboardDispatcher( (const unsigned char) key);

            //exit_ = exit_ || i > 100;
            viz_.spinOnce(3, true);
        }
        
        boost::lock_guard<boost::mutex> lock(cloud_mem_lock_); //Wait for saving file.
        return true;
    }

    bool pause_ /*= false*/;
    bool exit_, iteractive_mode_;
    
    OpenNI2Source& capture_;
    KinFu::Ptr kinfu_;
    cv::viz::Viz3d viz_;

    cv::Mat view_host_;
    cuda::Image view_device_;
    cuda::Depth depth_device_;
    
    // downloader
    
    cuda::DeviceArray<Point> cloud_buffer_;
    cv::Mat cloud_host_;
    
    bool compute_normals_;
    cuda::DeviceArray<Normal> normal_buffer_;
    cv::Mat normals_host_;

    boost::mutex cloud_mem_lock_;
    
    // Affine3f recorder
    
    std::vector<cv::Affine3f> cam_poses_;
};


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main (int argc, char* argv[])
{
    int device = 0;
    cuda::setDevice (device);
    //cuda::printCudaDeviceInfo (device);
    cuda::printShortCudaDeviceInfo (device);

    if(cuda::checkIfPreFermiGPU(device))
        return std::cout << std::endl << "Kinfu is not supported for pre-Fermi GPU architectures, and not built for them by default. Exiting..." << std::endl, 1;
    
    OpenNI2Source capture;
    capture.open ("");
    
    kfusion::Intr intrd(366.0, 366.0, 260.0, 204.0);
    {
        namespace po        = boost::program_options;
        namespace po_style  = boost::program_options::command_line_style;
        
        //capture.open("d:/onis/20111013-224932.oni");
        // Declare the supported options.
        po::options_description desc("Allowed options");
        desc.add_options()
        ("help", "produce help message")
        ("calibration", po::value<std::string>(), "set intrinsics file")
        ;
        
        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);
        
        if (vm.count("help")) {
            std::cout << desc << "\n";
            return 1;
        }
        
        if (vm.count("calibration") and is_file_exists( vm["calibration"].as<std::string>() ) ) {
            std::string cal_file = vm["calibration"].as<std::string>();
            
            std::cout << "Depth intrinsics was in the file: "
            << cal_file << ".\n";
            KinFuApp::setIntrinsicsExternal( cal_file, intrd );
            
        } else {
            std::cout << "Calibration file was not set.\n";
        }
    }

    KinFuApp app (capture, intrd);

    // executing
    try { app.execute (); }
    catch (const std::bad_alloc& /*e*/) { std::cout << "Bad alloc" << std::endl; }
    catch (const std::exception& /*e*/) { std::cout << "Exception" << std::endl; }

    return 0;
}
