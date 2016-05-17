set(SUBSUBSYS_NAME c44api)
set(SUBSUBSYS_DESC "c44 segmentation api")
set(SUBSUBSYS_DEPS common geometry search sample_consensus kdtree octree features filters)

project("${SUBSUBSYS_NAME}")

set(build TRUE)

if(build)

find_package(PCL 1.8 REQUIRED)

# pcl dependent directories
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


  set(srcs
    src/bounding_box_utils.cpp
    src/realsense_toolkit.cpp    
    src/rigid_body.cpp
    src/segmentation_pipeline.cpp
    src/histogram_utils.cpp
  )

  set(LIB_NAME "${SUBSUBSYS_NAME}")
  set(incl_dirs
   "${CMAKE_CURRENT_SOURCE_DIR}/include"
 )
 include_directories(include)

  MESSAGE("pcl binary dir = ${CMAKE_CURRENT_BINARY_DIR}")
  set(headers 
    "${CMAKE_CURRENT_SOURCE_DIR}/include/bounding_box_utils.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/realsense_toolkit.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/segmentation_pipeline.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/rigid_body.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/histogram_utils.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/model_index.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/c44_types.h"
)
  add_library("${LIB_NAME}" ${srcs} ${headers}) # "${SUBSUBSYS_NAME}" ${incs} ${impl_incs} ${headers})
  target_link_libraries("${LIB_NAME}"
    ${PCL_LIBRARIES}
 )
  target_include_directories("${LIB_NAME}" PUBLIC
    ${incl_dirs}
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
  )
endif(build)




