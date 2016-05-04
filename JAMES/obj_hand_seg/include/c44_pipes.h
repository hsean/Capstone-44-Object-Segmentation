/******************************************************************************
* Author: Sean Hendrickson, James Schiffer
* File: c44_pipes.cpp
* Last Modified: 22 April 2016
* Description: This file contains implementations of PCL pipelines for use
*               in object and hand segmentation of 3D objects sitting on
*               a plane.
*              This is part of Portland State Universitie's 2015-2016 ECE
*              capstone projects, specifically team #44
******************************************************************************/
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/visualization/cloud_viewer.h>

#include <iostream>

#define DEBUG

namespace c44_pipes
{

    typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud3D;

    /**
     * desc:  This function cuts off all points outside a given range
     *        on the field passed the the field parameter.
     * param: (in) source_cloud - ptr to point cloud
     *        (out) filtered_cloud - ptr to cloud after filtering
     *        (in) axis - axis to filter values passed as "x", "y", or "z"
     *        (in) min_point - kept points are >= minPoint
     *        (in) max_point - kept points are <= maxPoint
     */
    void passthroughFilter(
            Cloud3D::Ptr source_cloud,
            Cloud3D::Ptr filtered_cloud,
            const std::string& axis,
            float min_point,
            float max_point
            );


    /**
     * desc: This function uses a voxel filter to downsample a PCD file.
     * param: (in) source_cloud - ptr to cloud to be filtered
     *        (out) filtered_cloud - ptr to cloud after filtering
     *        (in) leaf_size - size of voxel in cm^3. (ex. 0.01 = 1 cm)
     */
    void voxelFilter(
            Cloud3D::Ptr source_cloud,
            Cloud3D::Ptr filtered_cloud,
            float leaf_size
            );

    /**
     * desc: This function computes the average distance between each point in a PCD
     *       file and then uses the standard deviation to designate points as outliers.
     *       All outliers are then removed from the resultant point cloud.
     * param: (in) source_cloud - ptr to input cloud
     *        (out) filtered_cloud - ptr to cloud with outliers removed
     *        (in) neighbors_to_analyze - number of nearest neighbors to analyze (ex. 50)
     *        (in) std_deviation - standard deviation used to find outliers (ex. 1.0)
     */
    void removeNoise(
            Cloud3D::Ptr source_cloud,
            Cloud3D::Ptr filtered_cloud,
            int neighbors_to_analyze,
            double std_deviation
            );

    /**
     * desc: This function takes a source point cloud and segments out a plane.
     *       The inliers of the plane, as well as all information needed to
     *       rebuild the plane is returned through the function parameters.
     * param: (in) source_cloud - ptr to input cloud
     *        (out) inliers_cloud - returns inliers of plane as pointcloud
     *        (out) outliers_cloud - returns outliers of plane as pointcloud
     *        (out) coefficients - returns the coefficients of the plane
     *        (out) indices - returns the indices of the plane
     *        (in) min_angle - minimum radius between plane and camera in radians (ex. 0)
     *        (in) maxRadius - maximum radius between plane and camera in radians (ex. 1.57)
     *        (in) ransac_iterations - maximum amount of times RANSAC can estimate plane (ex. 1000)
     *        (in) threshold_distance - range of points considered inliers from plane model (ex 0.01 = 1cm)
     * pre-cond: all output parameters must be declared before function call
     * ret: -1 if a plane could not be found else
     *      -2 if source PCD is empty
     *       0 if succesfull
     */
    int getPlane(
            Cloud3D::Ptr source_cloud,
            Cloud3D::Ptr inliers_cloud,
            Cloud3D::Ptr outliers_cloud,
            pcl::ModelCoefficients::Ptr coefficients,
            pcl::PointIndices::Ptr plane_indices,
            double min_angle,
            double max_angle,
            int ransac_iterations,
            double threshold_distance
            );

    /**
     * desc: This function takes a source PCD image an segments out objects found
     *       on a plane.
     * param: (in) source_cloud - ptr to input cloud
     *        (out) objects_cloud - returns cloud with objects above plane
     *        (in) min_angle - minimum radius between plane and camera in radians (ex. 0)
     *        (in) max_angle - maximum radius between plane and camera in radians (ex. 1.57)
     *        (in) ransac_iterations - maximum amount of times RANSAC can estimate plane (ex. 1000)
     *        (in) threshold_distance - range of points considered inliers from plane model (ex 0.01 = 1cm)
     *        (in) min_object_distance - lowest point of objects above a plane (ex 0.01 = 1cm)
     *        (in) max_object_distance - highest point of objects above a plane (ex 0.2 = 20cm)
     * pre-cond: all output variables must be declared before function is called.
     * ret: -1 if a plane could not be found else
     *      -2 if source PCD is empty
     *       0 if succesfull
     */
    int getPrism(
            Cloud3D::Ptr source_cloud,
            Cloud3D::Ptr objects_cloud,
            double min_angle,
            double max_angle,
            int ransacIterations,
            double threshold_distance,
            double min_object_distance,
            double max_object_distance
            );


    ///////////////////////////////////////////////////////////////////////////////
    /**
     * desc: This function uses a "Euclidean Cluster Extraction" algorithm to pair
     *       separate data in a Point Cloud into separate objects.
     * param: (in) source_cloud - ptr to input cloud
     *        (out) cluster_indicies - ojects that holds each cluster as an element of
     *                                 a vector. cluster_indices[0] is cluster 1, and
     *                                 cluster_indices[1] is cluster 2, and so on.
     *        (in) cluster_tolerance - The maximum distance between points in the
     *                                same cluster (e.g. 0.02 = 2cm)
     *        (in) min_cluster_size - minimum number of points in a single cluster
     *        (in) max_cluster_size - minimum number of points in a single cluster
     * pre-cond: cluster_indicies needs to be declared before function call
     */
    void clusterExtraction(
            Cloud3D::Ptr source_cloud,
            std::vector<pcl::PointIndices>* cluster_indices,
            double cluster_tolerance,
            int min_cluster_size,
            int max_cluster_size
            );

    ///////////////////////////////////////////////////////////////////////////////
    /**
     * desc:
     * Seperates a cluster discovered by Euclidean Extraction from the
     * cloud of clusters. Not sure why we are not using indicies extraction for this.
     * #TODO - see if manual indicies extraction was actually neccessary or some
     *          weird tutorial related thing
     *
     * To separate each cluster out of the vector<PointIndices> we have to iterate
     * through cluster_indices, create a new PointCloud for each entry
     * and write all points of the current cluster in the PointCloud.
     * param: (inout) it - Iterator holding reference to current cluster to seperate
     *        (in) cloud_of_objs - point cloud of se
     *        (out) cloud_cluster - point cloud of seperated object
     */
    bool seperateClusters(
            std::vector<pcl::PointIndices> cluster_indices,
            std::vector<pcl::PointIndices>::const_iterator obj_clusters_index,
            Cloud3D::Ptr cloud_of_objs,
            Cloud3D::Ptr cloud_cluster
            );
}


