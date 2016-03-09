cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(demo_01)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (demo_01 demo_01.cpp)
target_link_libraries (demo_01 ${PCL_LIBRARIES})
