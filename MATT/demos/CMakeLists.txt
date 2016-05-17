set(SUBSUBSYS_NAME demos)
set(SUBSUBSYS_DESC "Capstone 44 demos")
set(SUBSUBSYS_DEPS c44api)

set(DEFAULT true)
set(build TRUE)
project("${SUBSUBSYS_NAME}")
find_package(PCL 1.8 REQUIRED)

if(build)
    # pcl dependent directories
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})

    find_library(C44API c44api)
    
    add_executable(c44_bbox_viewer
      src/matt/c44_bbox_viewer.cpp
    )
    target_link_libraries(c44_bbox_viewer LINK_PUBLIC
      c44api
      ${PCL_LIBRARIES}
    )

   add_executable(correspondence_grouping_demo
     src/saroj/correspondence_grouping.cpp
   )
   target_link_libraries(correspondence_grouping_demo LINK_PUBLIC
     c44api
     ${PCL_LIBRARIES}
   )

#   PCL_ADD_EXECUTABLE(sean_demo3 
#     "${SUBSUBSYS_NAME}" 
#     ${moc_srcs} 
#     src/sean/sean_demo3.cpp 
#     ${incs} 
#     ${impl_incs}
#   )

#   target_link_libraries(sean_demo3 
#     c44api
#     pcl_segmentation 
#   )
   add_executable(sean_demo3
     src/sean/sean_demo3.cpp
   )
   target_link_libraries(sean_demo3 LINK_PUBLIC
     c44api
     ${PCL_LIBRARIES}
   )


#   # PCL_ADD_EXECUTABLE(c44_vfh_util 
#   #   "${SUBSUBSYS_NAME}" 
#   #   ${moc_srcs} 
#   #   src/matt/c44_vfh_util.cpp 
#   #   ${incs} 
#   #   ${impl_incs}
#   # )

#   # target_link_libraries(c44_vfh_util
#   #   c44api
#   # )

#   PCL_ADD_EXECUTABLE(c44_virtual_scanner
#     "${SUBSUBSYS_NAME}"
#     ${moc_srcs}
#     src/matt/c44_virtual_scanner.cpp
#     ${incs}
#     ${impl_incs}
#   )

#   target_link_libraries(c44_virtual_scanner
#     c44api
#     pcl_segmentation
#   )
   add_executable(c44_virtual_scanner
     src/matt/c44_virtual_scanner.cpp
   )
   target_link_libraries(c44_virtual_scanner LINK_PUBLIC
     c44api
     ${PCL_LIBRARIES}
   )

#   PCL_ADD_EXECUTABLE(cluster_rec_demo2
#     "${SUBSUBSYS_NAME}"
#     ${moc_srcs}
#     src/matt/cluster_rec_demo2.cpp
#     ${incs}
#     ${impl_incs}
#   )

#   target_link_libraries(cluster_rec_demo2
#     c44api
#     pcl_visualization
#     pcl_segmentation
#   )
   add_executable(cluster_rec_demo2
     src/matt/cluster_rec_demo2.cpp
   )
   target_link_libraries(cluster_rec_demo2 LINK_PUBLIC
     c44api
     ${PCL_LIBRARIES}
   )

endif(build)

