cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pointcloud_example)

set(CMAKE_CXX_STANDARD 11)
find_package(PCL 1.7 REQUIRED)

include_directories(
  include/
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (detect_plane
    src/plane_detection.cpp
)
target_link_libraries (detect_plane
    ${PCL_LIBRARIES}
)
