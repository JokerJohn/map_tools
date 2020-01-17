cmake_minimum_required(VERSION 3.5.1)
project(gridmap_viewer)

set(CMAKE_CXX_STANDARD 11)
find_package(PCL 1.7 REQUIRED)
add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})

add_executable(gridmap_viewer src/gridmap_loader.cpp)
target_link_libraries(gridmap_viewer  ${PCL_LIBRARIES})
