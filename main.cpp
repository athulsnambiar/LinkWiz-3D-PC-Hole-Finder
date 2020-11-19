#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <array>
#include <tuple>
#include <sstream>
#include <memory>
#include <thread>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std::chrono_literals;

std::vector<std::array<double, 3>> load_point_vector(std::string filename, bool show_values = false)
{
    std::ifstream pcl_file;
    std::vector<std::array<double, 3>> points;
    pcl_file.open("Hole Detection Test Data.txt");
    if(pcl_file.is_open())
    {
        std::string line;
        while(getline(pcl_file, line))
        {
            std::stringstream xyz(line);
            double x, y, z;
            xyz >> x >> y >> z;
            if(show_values)
                std::cout <<"x: " << x << " y: " << y << " z: " << z << '\n';
            points.push_back({x,y,z});
        }
    }
    pcl_file.close();

    return points;
}

pcl::visualization::PCLVisualizer::Ptr visualize(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, std::array<double, 3> coords)
{
      pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
      viewer->setBackgroundColor (0, 0, 0);
      viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
      viewer->addCoordinateSystem (1.0, coords[0], coords[1], coords[2]);
      viewer->initCameraParameters ();
      return (viewer);
}

int main()
{
    std::string filename = "Hole Detection Test Data.txt";
    std::vector<std::array<double, 3>> points = load_point_vector(filename);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    double x = 0, y = 0, z = 0;
    for(const auto point: points)
    {
        pcl::PointXYZ p(point[0], point[1], point[2]);
        cloud_ptr->points.push_back(p);
        x+=point[0]; y+= point[1]; z += point[2];
    }
    x /= points.size();
    y /= points.size();
    z /= points.size();
    
    cloud_ptr->width = cloud_ptr->size();
    cloud_ptr->height = 1;

    
    pcl::visualization::PCLVisualizer::Ptr viewer = visualize(cloud_ptr, {x,y,z});

    
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        std::this_thread::sleep_for(100ms);
    }

}