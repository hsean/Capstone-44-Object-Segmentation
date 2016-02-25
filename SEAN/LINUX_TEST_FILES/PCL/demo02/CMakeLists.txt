cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(demo_02_functionTests)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (demo_02_functionTests demo_02_functionTests.cpp)
target_link_libraries (demo_02_functionTests ${PCL_LIBRARIES})
