cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

if(UNIX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fpermissive")
endif()



set(SUBSYS_NAME c44)
set(SUBSYS_DESC "Application examples/samples that show how PCL works")
set(SUBSYS_DEPS common geometry io filters sample_consensus segmentation visualization kdtree features surface octree registration keypoints tracking search recognition ml stereo)

project ("${SUBSYS_NAME}")

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})
 
find_package(PCL 1.8 REQUIRED)

# pcl dependent directories
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(DEFAULT TRUE)
set(build TRUE)
if(build)
    add_subdirectory("demos")
    add_subdirectory("api")
endif(build)

