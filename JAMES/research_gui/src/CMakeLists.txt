cmake_minimum_required (VERSION 2.6 FATAL_ERROR)

project      (pcl-visualizer)
find_package (Qt4 REQUIRED)
find_package (VTK REQUIRED)
find_package (PCL 1.7.1 REQUIRED)
find_file(LIBREALSENSE librealsense.so /usr/local/lib)
if(NOT LIBREALSENSE)
  message(SEND_ERROR "cannot find libaray librealsense.so")
endif(NOT LIBREALSENSE)
if(LIBREALSENSE)
    message("FOUND LIBREALSENSE")
endif(LIBREALSENSE)

exec_program(/bin/ln ARGS "-s \"${LIBREALSENSE}\"
\"${CMAKE_CURRENT_BINARY_DIR}/librealsense.so\"" return_value ln_res)
if("${ln_retval}" GREATER 0)
    message(SEND_ERROR "Problem creating symlink.")
endif("${ln_retval}" GREATER 0)


include(CheckCXXCompilerFlag)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

link_directories    (${PCL_LIBRARY_DIRS} /usr/local/lib)
include_directories (${PCL_INCLUDE_DIRS} ~/librealsense/include)
add_definitions     (${PCL_DEFINITIONS})


set  (project_SOURCES main.cpp pclviewer.cpp)
set  (project_HEADERS pclviewer.h)
set  (project_FORMS   pclviewer.ui)
set  (VTK_LIBRARIES   vtkRendering vtkGraphics vtkHybrid QVTK)


QT4_WRAP_CPP (project_HEADERS_MOC   ${project_HEADERS})
QT4_WRAP_UI  (project_FORMS_HEADERS ${project_FORMS})

INCLUDE         (${QT_USE_FILE})
ADD_DEFINITIONS (${QT_DEFINITIONS})

ADD_EXECUTABLE  (pcl_visualizer ${project_SOURCES}
                                ${project_FORMS_HEADERS}
                                ${project_HEADERS_MOC})

TARGET_LINK_LIBRARIES (pcl_visualizer ${QT_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES} /usr/local/lib/librealsense.so)
