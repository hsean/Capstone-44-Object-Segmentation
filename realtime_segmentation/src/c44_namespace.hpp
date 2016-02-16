/**************************************************************************************************
 * Utility Function header for C44 group
 * Gives us a place to experiment and organize different functionality in a easy way.
 * Keep an eye out for potential objects.
 * ***********************************************************************************************/

#ifndef C44_NAMESPACE_H_
#define C44_NAMESPACE_H_

#include <librealsense/rs.hpp>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include "pcl/io/pcd_io.h"
#include "pcl/io/file_io.h"
#include "pcl/point_types.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>

namespace c44
{
    //----------------------- REALSENSE RELATED ---------------------------------
    int GetRealsenseDevice(rs::device * dev);

    //pcl::PointCloud<pcl::PointXYZ>::Ptr GrabRealsenseFrame(rs::device *dev, float maxDistanceMeters);
    void GrabRealsenseFrame(rs::device *dev, float maxDistanceMeters, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);


    //-------------------------- VISUALIZER RELATED ---------------------------------

    void AddCloudToVisualizer (
                    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
                    double red,
                    double green,
                    double blue,
                    std::string cloudName
                   );

    void overwriteVis (
                        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
                        double red,
                        double green,
                        double blue
                       );

     // -------------------------- SEGMENTATION RELATED ------------------------------
    void DownsampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

    void RemoveNansFromCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

    void EstimateCloudNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals);

    pcl::PointCloud<pcl::PointXYZ>::Ptr SegmentPlane(
                                                     pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                                     pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals,
                                                     pcl::ModelCoefficients::Ptr &coefficients_plane
                                                    );

    pcl::PointCloud<pcl::PointXYZ>::Ptr ExtractObjects(
                                                        pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                                        pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals,
                                                        pcl::ModelCoefficients::Ptr &coefficients_cylinder
                                                        );
}

#endif
