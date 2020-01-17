#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <string>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include<stdio.h>
#include<stdlib.h>
#define random(x) (rand()%x)


using Point = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<pcl::PointXYZI>;
// 读取文件夹内的点云文件，并排序
void getAllFiles(std::string path, std::vector<std::string> &files)
{
    if (path[path.length() - 1] != '/')
        path = path + "/";
    DIR *dir;
    struct dirent *ptr;
    char base[1000];
    if ((dir = opendir(path.c_str())) == NULL)
    {
        perror("Open dir error...");
        std::cout << "Check: " << path << std::endl;
        exit(1);
    }

    while ((ptr = readdir(dir)) != NULL)
    {
        if (ptr->d_type == 8) // 文件
        {
            std::string name = ptr->d_name;
            int length = name.length();
            if (name.substr(length - 3, length - 1) == "pcd" || name.substr(length - 3, length - 1) == "PCD")
            {
                std::cout << path + name << std::endl;
                files.push_back(path + name);
            }
        }
    }
    closedir(dir);
    std::sort(files.begin(), files.end());
    return;
}


// 修改为自己的grid_map目录
std::string input_dir = "../../map/grid_pcd";

int main(int argc, char **argv)
{

    std::vector<std::string> grid_files;
    getAllFiles(input_dir, grid_files);

    // Load all PCDs
    PointCloud::Ptr map_ptr(new PointCloud);
    PointCloud::Ptr tmp_ptr(new PointCloud);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("map viewer"));

    int num = 0;
    int num2 = 255;

    for (int i = 0; i < grid_files.size(); i++)
    {
        if (pcl::io::loadPCDFile<Point>(grid_files[i], *tmp_ptr) == -1)
        {
            std::cout << "Failed to load " << grid_files[i] << "." << std::endl;
        }

        num++;
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> single_color(tmp_ptr, random(255), random(255), random(255)); // green
        viewer->addPointCloud<pcl::PointXYZI>(tmp_ptr, single_color, "sample cloud"+num);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, "sample cloud"+num); // 设置点云大小

        *map_ptr += *tmp_ptr;
        std::cout << "Finished to load " << grid_files[i] << "." << std::endl;
    }

    std::cout << "map size:" << map_ptr->size()<< "." << std::endl;

//    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> fildColor(map_ptr, "z"); // 按照z字段进行渲染
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return 0;
}
