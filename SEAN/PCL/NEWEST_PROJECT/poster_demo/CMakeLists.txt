cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(poster_demo)

find_package(PCL 1.7 REQUIRED)

# pcl dependent directories
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# add HEADERS to project
include_directories(include)

# add SOURCES to project
file(GLOB SOURCES "src/*.cpp")

add_executable (poster_demo ${SOURCES})
target_link_libraries (poster_demo ${PCL_LIBRARIES})
