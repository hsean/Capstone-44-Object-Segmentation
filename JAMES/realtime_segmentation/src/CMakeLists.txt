cmake_minimum_required (VERSION 2.6 FATAL_ERROR)

project      (realtime_segmentation)
find_package (Qt4 REQUIRED)
find_package (VTK REQUIRED)
find_package (PCL 1.7.1 REQUIRED)


include(CheckCXXCompilerFlag)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

link_directories    (${PCL_LIBRARY_DIRS} /usr/local/lib)
include_directories (${PCL_INCLUDE_DIRS} ~/librealsense/include)
add_definitions     (${PCL_DEFINITIONS})


set  (project_SOURCES realtime_segmentation.cpp c44_namespace.cpp)
set  (project_HEADERS c44_namespace.hpp)
set  (project_FORMS   )
set  (VTK_LIBRARIES   vtkRendering vtkGraphics vtkHybrid QVTK)


QT4_WRAP_CPP (project_HEADERS_MOC   ${project_HEADERS})
QT4_WRAP_UI  (project_FORMS_HEADERS ${project_FORMS})

INCLUDE         (${QT_USE_FILE})
ADD_DEFINITIONS (${QT_DEFINITIONS})

ADD_EXECUTABLE  (realtime_segmentation ${project_SOURCES}
                                ${project_FORMS_HEADERS}
                                ${project_HEADERS_MOC})

TARGET_LINK_LIBRARIES (realtime_segmentation ${QT_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES} /usr/local/lib/librealsense.so)
