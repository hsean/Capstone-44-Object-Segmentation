cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(demo_03)

find_package(PCL 1.6 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (demo03 demo03.cpp)
target_link_libraries (demo03 ${PCL_LIBRARIES})
