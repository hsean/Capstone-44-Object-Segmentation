cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(global_classification)

#if(UNIX)
#    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#endif()

find_package(PCL 1.8 REQUIRED)

# pcl dependent directories
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# add HEADERS to project
#include_directories(include)

# add SOURCES to project
file(GLOB SOURCES "src/*.cpp")

add_executable (global_classification ${SOURCES})
target_link_libraries (global_classification ${PCL_LIBRARIES})
