set(SUBSUBSYS_NAME c44api)
set(SUBSUBSYS_DESC "c44 segmentation api")
set(SUBSUBSYS_DEPS common geometry search sample_consensus kdtree octree features filters)


set(build TRUE)
PCL_SUBSUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSUBSYS_NAME}" "${SUBSUBSYS_DESC}" ${DEFAULT} "${REASON}")
PCL_SUBSUBSYS_DEPEND(build
  "${SUBSYS_NAME}"
  "${SUBSUBSYS_NAME}"
  DEPS ${SUBSUBSYS_DEPS}
  EXT_DEPS vtk eigen boost
)

if(build)

  set(srcs
    src/bounding_box_utils.cpp
    src/realsense_toolkit.cpp    
    src/rigid_body.cpp
    src/segmentation_pipeline.cpp
    src/histogram_utils.cpp
  )

  set(LIB_NAME "${SUBSUBSYS_NAME}")
  set(impl_incls "")
  set(incs "")
  set(incl_dirs 
    "${c44_pcl_src_root}/common/include"
    "${c44_pcl_src_root}/io/include"
    "${c44_pcl_src_root}/visualization/include"
    "${c44_pcl_src_root}/features/include"
    "${c44_pcl_src_root}/filters/include"
    "${c44_pcl_src_root}/features/include"
    "${c44_pcl_src_root}/search/include"
    "${c44_pcl_src_root}/kdtree/include"
    "${c44_pcl_src_root}/octree/include"
    "${c44_pcl_src_root}/sample_consensus/include"
    "${c44_pcl_src_root}/segmentation/include"
    "${c44_pcl_src_root}/surface/include"
    "${c44_pcl_src_root}/geometry/include"
    "${c44_pcl_src_root}/recognition/include"
    /usr/local/include/vtk-7.0
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_BINARY_DIR}/../../pcl/include"
    /usr/local/include
    /usr/local/include/eigen3
  )
  include_directories(${incl_dirs})

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
  PCL_ADD_LIBRARY("${LIB_NAME}" "${SUBSUBSYS_NAME}" ${srcs} ${incs} ${impl_incs} ${headers})
  target_link_libraries("${LIB_NAME}"
    pcl_search
    pcl_sample_consensus
    pcl_filters
    pcl_features
    pcl_common
    pcl_io
    pcl_surface
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY}
  )
  target_include_directories("${LIB_NAME}" PUBLIC 
    ${incl_dirs}
  )
endif(build)




