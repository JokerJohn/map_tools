cmake_minimum_required(VERSION 2.8.3)
project(map_tools)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
# LIBRARIES map_filter
# CATKIN_DEPENDS pcl_ros roscpp rospy sensor_msgs std_msgs
# DEPENDS system_lib PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pcd_grid_divider src/pcd_grid_divider.cpp)
#add_dependencies(pcd_grid_divider  ${PROJECT_NAME}_gencfg)

target_link_libraries(pcd_grid_divider  ${catkin_LIBRARIES})
