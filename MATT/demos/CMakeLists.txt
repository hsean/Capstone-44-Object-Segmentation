set(SUBSUBSYS_NAME demos)
set(SUBSUBSYS_DESC "Capstone 44 demos")
set(SUBSUBSYS_DEPS c44api)

set(DEFAULT true)
PCL_SUBSUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSUBSYS_NAME}" "${SUBSUBSYS_DESC}" ${DEFAULT} "${REASON}")
PCL_SUBSUBSYS_DEPEND(build "${SUBSYS_NAME}" "${SUBSUBSYS_NAME}" DEPS ${SUBSUBSYS_DEPS} EXT_DEPS vtk)

set(build TRUE)
#add_definitions(${PCL_DEFINITIONS})
set(PCL_DIR value PARENT_SCOPE)
set(PCL_DIR ${PCL_DIR} PARENT_SCOPE)
MESSAGE("pcl dir = ${PCL_DIR}")
if(build)
    include_directories(
      "${pcl_root_dir_buddy}/common/include"
      "${pcl_root_dir_buddy}/io/include"
      "${pcl_root_dir_buddy}/visualization/include"
      /usr/local/include/vtk-7.0
      "${CMAKE_CURRENT_SOURCE_DIR}/include"
    )

    set(moc_srcs "")
    set(incs "")
    set(impl_incs "")


    
    PCL_ADD_EXECUTABLE(c44_bbox_viewer "${SUBSUBSYS_NAME}" ${moc_srcs} 
      src/matt/c44_bbox_viewer.cpp
      ${incs} 
      ${impl_incs}
    )
    target_link_libraries(c44_bbox_viewer 
      c44api 
      pcl_segmentation
      pcl_visualization
    )


    PCL_ADD_EXECUTABLE(matt_cluster_demo "${SUBSUBSYS_NAME}" ${moc_srcs}
      src/matt/cluster_experiments.cpp
      ${incs}
      ${impl_incs}
    )
    target_link_libraries(matt_cluster_demo
      c44api
      pcl_segmentation
      pcl_visualization
      ${Boost_SYSTEM_LIBRARY}
      ${Boost_FILESYSTEM_LIBRARY}
    )


    PCL_ADD_EXECUTABLE(correspondence_grouping_demo
      "${SUBSUBSYS_NAME}" 
      ${moc_srcs} 
      src/saroj/correspondence_grouping.cpp 
      ${incs} 
      ${impl_incs}
    )

    target_link_libraries(correspondence_grouping_demo 
      c44api
      pcl_visualization 
      pcl_recognition
    )

    PCL_ADD_EXECUTABLE(sean_demo3 
      "${SUBSUBSYS_NAME}" 
      ${moc_srcs} 
      src/sean/sean_demo3.cpp 
      ${incs} 
      ${impl_incs}
    )

    target_link_libraries(sean_demo3 
      c44api
      pcl_segmentation 
    )

    # PCL_ADD_EXECUTABLE(c44_vfh_util 
    #   "${SUBSUBSYS_NAME}" 
    #   ${moc_srcs} 
    #   src/matt/c44_vfh_util.cpp 
    #   ${incs} 
    #   ${impl_incs}
    # )

    # target_link_libraries(c44_vfh_util
    #   c44api
    # )


    PCL_ADD_EXECUTABLE(c44_virtual_scanner
      "${SUBSUBSYS_NAME}"
      ${moc_srcs}
      src/matt/c44_virtual_scanner.cpp
      ${incs}
      ${impl_incs}
    )

    target_link_libraries(c44_virtual_scanner
      c44api
      pcl_segmentation
    )


    PCL_ADD_EXECUTABLE(cluster_rec_demo2
      "${SUBSUBSYS_NAME}"
      ${moc_srcs}
      src/matt/cluster_rec_demo2.cpp
      ${incs}
      ${impl_incs}
    )

    target_link_libraries(cluster_rec_demo2
      c44api
      pcl_visualization
      pcl_segmentation
    )

    PCL_ADD_EXECUTABLE(build_sample_vfh_data
      "${SUBSUBSYS_NAME}"
      ${moc_srcs}
      src/matt/build_vfh_data.cpp
      ${incs}
      ${impl_incs}
    )

    target_link_libraries(build_sample_vfh_data
      c44api
    )
endif(build)

