cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(pcl_tracking_sampling)

FIND_PACKAGE(PCL 1.3 REQUIRED COMPONENTS io common visualization tracking)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(pcl_tracking_sampling_test pcl_tracking_sampling_test.cpp)
target_link_libraries(pcl_tracking_sampling_test ${PCL_LIBRARIES})
