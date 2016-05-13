cmake_minimum_required(VERSION 2.6)

set(SUBSYS_NAME c44)
set(SUBSYS_DESC "Application examples/samples that show how PCL works")
set(SUBSYS_DEPS common geometry io filters sample_consensus segmentation visualization kdtree features surface octree registration keypoints tracking search recognition ml stereo)

set(DEFAULT TRUE)
set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ${DEFAULT} "${REASON}")
SET_INSTALL_DIRS()
if(build)
    set(c44_pcl_src_root "${CMAKE_CURRENT_SOURCE_DIR}/../pcl")


    add_subdirectory("demos")
    add_subdirectory("api")


endif(build)

