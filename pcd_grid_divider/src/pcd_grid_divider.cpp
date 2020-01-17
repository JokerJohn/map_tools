#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sstream>
#include <string>
#include <vector>
#include "ros/ros.h"
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

//std::string OUT_DIR;
const std::string FILE_NAME = "pcd_info.csv";
std::string point_type;
int grid_size;
std::string INPUT_DIR;
std::string OUT_DIR;

using Point = pcl::PointXYZ; // default
using PointCloud = pcl::PointCloud<Point>;

struct pcd_xyz_grid {
    std::string filename;
    std::string name;
    int grid_id;
    int grid_id_x;
    int grid_id_y;
    int lower_bound_x;
    int lower_bound_y;
    int upper_bound_x;
    int upper_bound_y;
    pcl::PointCloud <pcl::PointXYZ> cloud;
};

struct pcd_xyzi_grid {
    std::string filename;
    int grid_id;
    int grid_id_x;
    int grid_id_y;
    int lower_bound_x;
    int lower_bound_y;
    int upper_bound_x;
    int upper_bound_y;
    pcl::PointCloud <pcl::PointXYZI> cloud;
};

struct pcd_xyzrgb_grid {
    std::string filename;
    int grid_id;
    int grid_id_x;
    int grid_id_y;
    int lower_bound_x;
    int lower_bound_y;
    int upper_bound_x;
    int upper_bound_y;
    pcl::PointCloud <pcl::PointXYZRGB> cloud;
};

/**
 *  读取指定路径内的pcd文件并按名称排序
 * @param path
 * @param files
 */
void getAllFiles(std::string path, std::vector <std::string> &files) {
    if (path[path.length() - 1] != '/')
        path = path + "/";
    DIR *dir;
    struct dirent *ptr;
    char base[1000];
    if ((dir = opendir(path.c_str())) == NULL) {
        perror("Open dir error...");
        std::cout << "Check: " << path << std::endl;
        exit(1);
    }

    while ((ptr = readdir(dir)) != NULL) {
        if (ptr->d_type == 8) // 文件
        {
            std::string name = ptr->d_name;
            int length = name.length();
            if (name.substr(length - 3, length - 1) == "pcd" || name.substr(length - 3, length - 1) == "PCD") {
                std::cout << path + name << std::endl;
                files.push_back(path + name);
            }
        }
    }
    closedir(dir);
    std::sort(files.begin(), files.end());
    return;
}

/**
 *  清空指定文件夹
 * @param dir_full_path
 * @return
 */


int rm_dir(std::string dir_full_path) {
    DIR *dirp = opendir(dir_full_path.c_str());
    if (!dirp) {
        return -1;
    }
    struct dirent *dir;
    struct stat st;
    while ((dir = readdir(dirp)) != NULL) {
        if (strcmp(dir->d_name, ".") == 0
            || strcmp(dir->d_name, "..") == 0) {
            continue;
        }
        std::string sub_path = dir_full_path + '/' + dir->d_name;
        if (lstat(sub_path.c_str(), &st) == -1) {
            //Log("rm_dir:lstat ",sub_path," error");
            continue;
        }
        if (S_ISDIR(st.st_mode)) {
            if (rm_dir(sub_path) == -1) // 如果是目录文件，递归删除
            {
                closedir(dirp);
                return -1;
            }
            rmdir(sub_path.c_str());
        } else if (S_ISREG(st.st_mode)) {
            unlink(sub_path.c_str());     // 如果是普通文件，则unlink
        } else {
            //Log("rm_dir:st_mode ",sub_path," error");
            continue;
        }
    }
//    if (rmdir(dir_full_path.c_str()) == -1)//delete dir itself.
//    {
//        closedir(dirp);
//        return -1;
//    }
    closedir(dirp);
    return 0;
}

/**
 *  写csv文件
 * @param grids
 */
void write_csv(std::vector <pcd_xyz_grid> &grids) {
    std::string whole_file_name = OUT_DIR + FILE_NAME;
    std::ofstream ofs(whole_file_name.c_str());
    int grid_num = grids.size();
    for (int i = 0; i < grid_num; i++) {
        if (grids[i].cloud.points.size() > 0) {
            ofs << grids[i].name
                << "," << grids[i].lower_bound_x
                << "," << grids[i].lower_bound_y
                << "," << 0.0
                << "," << grids[i].upper_bound_x
                << "," << grids[i].upper_bound_y
                << "," << 0.0 << std::endl;
        }
    }
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "grid divider");
    ros::NodeHandle nh("~");

    nh.getParam("grid_size", grid_size);
    nh.getParam("input_directory", INPUT_DIR);
    nh.getParam("output_directory", OUT_DIR);

    std::vector <std::string> files;

    if (OUT_DIR[OUT_DIR.length() - 1] != '/')
        OUT_DIR = OUT_DIR + "/";
//    清空output
    int code = rm_dir(OUT_DIR);
    if (!code) {
        std::cout << "Clear Files Failed..." << std::endl;
    }


    getAllFiles(INPUT_DIR, files);

    // Load all PCDs
    PointCloud map;
    PointCloud tmp;
    for (int i = 0; i < files.size(); i++) {
        if (pcl::io::loadPCDFile<Point>(files[i], tmp) == -1) {
            std::cout << "Failed to load " << files[i] << "." << std::endl;
        }
        map += tmp;
        std::cout << "Finished to load " << files[i] << "." << std::endl;
    }

    std::cout << "Finished to load all PCDs: " << map.size() << " points." << std::endl;

    double min_x = 10000000000.0;
    double max_x = -10000000000.0;
    double min_y = 10000000000.0;
    double max_y = -10000000000.0;

    // Search minimum and maximum points along x and y axis.
    for (PointCloud::const_iterator p = map.begin(); p != map.end(); p++) {
        if (p->x < min_x) {
            min_x = p->x;
        }
        if (p->x > max_x) {
            max_x = p->x;
        }
        if (p->y < min_y) {
            min_y = p->y;
        }
        if (p->y > max_y) {
            max_y = p->y;
        }
    }

    // Find minimum and maximum boundary 找到xy方向上最大和最小方块的边界
    int min_x_b = grid_size * static_cast<int>(floor(min_x / grid_size));
    int max_x_b = grid_size * static_cast<int>(floor(max_x / grid_size) + 1);
    int min_y_b = grid_size * static_cast<int>(floor(min_y / grid_size));
    int max_y_b = grid_size * static_cast<int>(floor(max_y / grid_size) + 1);

    // Number of grid along x and y axis 计算方格个数
    int div_x = (max_x_b - min_x_b) / grid_size;
    int div_y = (max_y_b - min_y_b) / grid_size;
    int grid_num = div_x * div_y;

    // Define filename, lower/upper bound of every grid
    std::vector <pcd_xyz_grid> grids(grid_num);
    for (int y = 0; y < div_y; y++) {
        for (int x = 0; x < div_x; x++) {
            int id = div_x * y + x;
            grids[id].grid_id = id; //序号
            grids[id].grid_id_x = x; //第几行
            grids[id].grid_id_y = y; //第几列
            grids[id].lower_bound_x = min_x_b + grid_size * x; //方格的四个顶点
            grids[id].lower_bound_y = min_y_b + grid_size * y;
            grids[id].upper_bound_x = min_x_b + grid_size * (x + 1);
            grids[id].upper_bound_y = min_y_b + grid_size * (y + 1);
            grids[id].filename = OUT_DIR + std::to_string(grid_size) + "_" +
                                 std::to_string(grids[id].lower_bound_x) + "_" +
                                 std::to_string(grids[id].lower_bound_y) + ".pcd";
            grids[id].name = std::to_string(grid_size) + "_" +
                             std::to_string(grids[id].lower_bound_x) + "_" +
                             std::to_string(grids[id].lower_bound_y) + ".pcd";
        }
    }

    // Assign all points to appropriate grid according to their x/y value  把每一个point进行分类，塞到具体的网格里面
    for (PointCloud::const_iterator p = map.begin(); p != map.end(); p++) {
        int idx = static_cast<int>(floor((p->x - static_cast<float>(min_x_b)) / grid_size));
        int idy = static_cast<int>(floor((p->y - static_cast<float>(min_y_b)) / grid_size));
        int id = idy * div_x + idx;

        const Point &tmp = *p;
        grids[id].cloud.points.push_back(tmp);
    }

//  写pcd文件
    int points_num = 0;
    for (int i = 0; i < grid_num; i++) {
        if (grids[i].cloud.points.size() > 0) {
            pcl::io::savePCDFileBinary(grids[i].filename, grids[i].cloud);
            std::cout << "Wrote " << grids[i].cloud.points.size() << " points to "
                      << grids[i].filename << "." << std::endl;
            points_num += grids[i].cloud.points.size();
        }
    }
    write_csv(grids);
    std::cout << "Total points num: " << points_num << " points." << std::endl;
    return 0;
}
