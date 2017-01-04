#include <kfusion/types.hpp>

#include <boost/filesystem.hpp> //-->background threading

#include "obj_writer.h"
#include <sys/stat.h>
//#include <iostream>
#include <string>

using namespace kfusion;

#define REPORT_ERROR(msg) kfusion::cuda::error ((msg), __FILE__, __LINE__)

bool is_path_exists(const std::string &strPath) {
    return boost::filesystem::exists(boost::filesystem::path(strPath));
}

bool is_file_exists(const std::string& name) {
    struct stat buffer;
    return (stat (name.c_str(), &buffer) == 0);
}

std::string gen_next_filename(const std::string& filebase, const std::string& ext, const int& index)
{
    std::ostringstream os;
    os << filebase << "_" << index << "." << ext;
    
    return os.str();
}

std::string fetch_filename(const std::string& filebase, const std::string& ext)
{
    int index = 0;
    std::string filename = gen_next_filename(filebase, ext, index);
    
    while( is_file_exists(filename) )
    {
        filename = gen_next_filename(filebase, ext, ++index);
    }
    return filename;
    
}

int
saveOBJFile (const std::string &file_name,
             const Point* cloud,
             const Normal* normal,
             unsigned cloud_size,
             unsigned precision)
{
    if (0 >= cloud_size)
    {
        REPORT_ERROR ("[demo::saveOBJFile] Input point cloud has no data!\n");
        return (-1);
    }
    // Open file
    std::ofstream fs;
    fs.precision (precision);
    fs.open (file_name.c_str ());
    
    /* Write 3D information */
    // number of points
    int nr_points  = cloud_size;
    // point size
    //unsigned point_size = sizeof(Point);
    // number of faces for header
    unsigned nr_faces = 0;
    // Do we have vertices normals?
    int normal_index = normal? 0 : -1;
    
    // Write the header information
    fs << "####" << std::endl;
    fs << "# OBJ dataFile simple version. File name: " << file_name << std::endl;
    fs << "# Vertices: " << nr_points << std::endl;
    if (normal_index != -1)
        fs << "# Vertices normals : " << nr_points << std::endl;
    fs << "# Faces: " << nr_faces << std::endl;
    fs << "####" << std::endl;
    
    // Write vertex coordinates
    fs << "# List of Vertices, with (x,y,z) coordinates, w is optional." << std::endl;
    for (int i = 0; i < nr_points; ++i)
    {
        // write vertices beginning with v
        //const Point* pi = cloud + i * point_size;
        const Point* pi = &cloud[i];
        if (std::isfinite(pi->data[0]))
        {
            fs << "v " << pi->data[0] << " " << pi->data[1] << " " << pi->data[2] << std::endl;
        }
    }
    
    fs << "# "<< nr_points <<" vertices" << std::endl;
    
    if(normal_index != -1)
    {
        fs << "# Normals in (x,y,z) form; normals might not be unit." <<  std::endl;
        for (int i = 0; i < nr_points; ++i)
        {
            // Write vertex normals
            // write vertices beginning with v
            const Normal* ni = &normal[i];
            if (std::isfinite(ni->data[0]))
            {
                fs << "vn " << ni->data[0] << " " << ni->data[1] << " " << ni->data[2] << std::endl;
            }
        }
        fs << "# "<< nr_points <<" vertices normals" << std::endl;
    }

    //fs << "# Face Definitions" << std::endl;

    // Write down faces
    /*
    if(normal_index == -1)
    {
     for(unsigned i = 0; i < nr_faces; i++)
     {
        fs << "f ";
      size_t j = 0;
      for (; j < mesh.polygons[i].vertices.size () - 1; ++j)
        fs << mesh.polygons[i].vertices[j] + 1 << " ";
      fs << mesh.polygons[i].vertices[j] + 1 << std::endl;
    }
  }
  else
  {
    for(unsigned i = 0; i < nr_faces; i++)
    {
      fs << "f ";
      size_t j = 0;
      for (; j < mesh.polygons[i].vertices.size () - 1; ++j)
        fs << mesh.polygons[i].vertices[j] + 1 << "//" << mesh.polygons[i].vertices[j] + 1 << " ";
      fs << mesh.polygons[i].vertices[j] + 1 << "//" << mesh.polygons[i].vertices[j] + 1 << std::endl;
    }
  }
  */
  fs << "# End of File" << std::endl;

  // Close obj file
  fs.close ();
  return 0;
}

int
savePLYFile (const std::string &file_name,
             const Point* cloud,
             const Normal* normal,
             unsigned cloud_size,
             unsigned precision,
             const std::string& format)
{
    if (0 >= cloud_size)
    {
        REPORT_ERROR ("[demo::saveOBJFile] Input point cloud has no data!\n");
        return (-1);
    }
    
    /*
     ply
     format ascii 1.0
     comment VCGLIB generated
     element vertex 1004187
     property float x
     property float y
     property float z
     property float nx
     property float ny
     property float nz
     element face 0
     property list uchar int vertex_indices
     end_header
     */
    
    
    // Open file
    std::ofstream fs;
    fs.precision (precision);
    fs.open (file_name.c_str ());
    
    /* Write 3D information */
    // number of points
    int nr_points  = cloud_size;
    // point size
    // number of faces for header
    unsigned nr_faces = 0;
    // Do we have vertices normals?
    int normal_index = normal? 1 : 0;
    
    // Write the header information
    fs << "ply" << std::endl;
    fs << "format " << format << " 1.0" << std::endl;
    fs << "comment AEMASS generated file: " << file_name << std::endl;
    fs << "element vertex " << nr_points << std::endl;
    fs << "property float x" << std::endl;
    fs << "property float y" << std::endl;
    fs << "property float z" << std::endl;
    
    if (normal_index)
    {
        fs << "property float nx" << std::endl;
        fs << "property float ny" << std::endl;
        fs << "property float nz" << std::endl;
    }
    fs << "element face " << nr_faces << std::endl;
    fs << "property list uchar int vertex_indices" << std::endl;
    fs << "end_header" << std::endl;
    
    // Write vertex coordinates
    for (int i = 0; i < nr_points; ++i)
    {
        const Point*  pi = &cloud[i];
        const Normal* ni = &normal[i];
        if (std::isfinite(pi->data[0]))
        {
            fs << pi->data[0] << " " << pi->data[1] << " " << pi->data[2];
            if(normal_index)
            {
                fs << " " << ni->data[0] << " " << ni->data[1] << " " << ni->data[2];
            }
            fs << std::endl;
        }
    }
    fs << std::endl;
    // Close obj file
    fs.close ();
    return 0;
}
