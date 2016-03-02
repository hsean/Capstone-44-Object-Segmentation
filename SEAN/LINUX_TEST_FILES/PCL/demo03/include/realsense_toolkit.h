/******************************************************************************
* Author: Sean Hendrickson
* File: realsense_toolkit.h
* Last Modified: 26 February 2016
* Description: This file contains definitions for functions used as part
*              of a toolkit being designed for Intel's RealSense depth camera.
*              This is part of Portland State Universitie's 2015-2016 ECE
*              capstone projects, specifically team #44
******************************************************************************/
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/visualization/cloud_viewer.h>

#include <iostream>

namespace c44
{
/** 
 * desc:  This function cuts off all points outside a given range
 *        on the field passed the the field parameter.
 * param: (in) sourceCloud - ptr to point cloud 
 *        (out) filteredCloud - ptr to cloud after filtering
 *        (in) axis - axis to filter values passed as "x", "y", or "z"
 *        (in) minPoint - kept points are >= minPoint
 *        (in) maxPoint - kept points are <= maxPoint
 */
void passthroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud,
	                   pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud,
	                   const std::string& axis, float minPoint, float maxPoint);
					   
/**
 * desc: This function uses a voxel filter to downsample a PCD file.
 * param: (in) sourceCloud - ptr to cloud to be filtered
 *        (out) filteredCloud - ptr to cloud after filtering
 *        (in) leafSize - size of voxel in cm^3. (ex. 0.01 = 1 cm)
 */					   
void voxelFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud,
                 float leafSize);
				 
/**
 * desc: This function computes the average distance between each point in a PCD
 *       file and then uses the standard deviation to designate points as outliers.
 *       All outliers are then removed from the resultant point cloud.
 * param: (in) sourceCloud - ptr to input cloud
 *        (out) filteredCloud - ptr to cloud with outliers removed
 *        (in) neighborsToAnalyze - number of nearest neighbors to analyze (ex. 50)
 *        (in) stdDeviation - standard deviation used to find outliers (ex. 1.0)   
 */
void removeNoise(pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud,
                 int neighborsToAnalyze, double stdDeviation);
				 
/** 
 * desc: This function takes a source point cloud and segments out a plane.
 *       The inliers of the plane, as well as all information needed to
 *       rebuild the plane is returned through the function parameters.
 * param: (in) sourceCloud - ptr to input cloud
 *        (out) inliersCloud - returns inliers of plane as pointcloud 
 *        (out) outliersCloud - returns outliers of plane as pointcloud
 *        (out) coefficients - returns the coefficients of the plane
 *        (out) indices - returns the indices of the plane
 *        (in) minRadius - minimum radius between plane and camera in radians (ex. 0)
 *        (in) maxRadius - maximum radius between plane and camera in radians (ex. 1.57)
 *        (in) ransacIterations - maximum amount of times RANSAC can estimate plane (ex. 1000)
 *        (in) thresholdDistance - range of points considered inliers from plane model (ex 0.01 = 1cm)
 * pre-cond: all output parameters must be declared before function call
 * ret: -1 if a plane could not be found else
 *      -2 if source PCD is empty
 *       0 if succesfull
 */				 
int getPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud,  
             pcl::PointCloud<pcl::PointXYZ>::Ptr planeCloud,                 
             pcl::PointCloud<pcl::PointXYZ>::Ptr outliersCloud,              
             pcl::ModelCoefficients::Ptr coefficients,                       
             pcl::PointIndices::Ptr indices,	
             double minRadius, double maxRadius,				
             int ransacIterations, double thresholdDistance);
			 
/**
 * desc: This function takes a source PCD image an segments out objects found 
 *       on a plane.
 * param: (in) sourceCloud - ptr to input cloud
 *        (out) objectsCloud - returns cloud with objects above plane
 *        (in) minRadius - minimum radius between plane and camera in radians (ex. 0)
 *        (in) maxRadius - maximum radius between plane and camera in radians (ex. 1.57)
 *        (in) ransacIterations - maximum amount of times RANSAC can estimate plane (ex. 1000)
 *        (in) thresholdDistance - range of points considered inliers from plane model (ex 0.01 = 1cm)
 *        (in) minObjectDistance - lowest point of objects above a plane (ex 0.01 = 1cm)
 *        (in) maxObjectDistance - highest point of objects above a plane (ex 0.2 = 20cm)
 * pre-cond: all output variables must be declared before function is called.
 * ret: -1 if a plane could not be found else
 *      -2 if source PCD is empty
 *       0 if succesfull
 */
int getPrism(pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud,
             pcl::PointCloud<pcl::PointXYZ>::Ptr objectsCloud,                                       	
             double minRadius, double maxRadius,				
             int ransacIterations, double thresholdDistance, 
             double minObjectDistance, double maxObjectDistance);

/**
 * desc: This function takes a source PCD image an segments out objects found 
 *       on a plane. The source image is downsampled and filtered prior to finding
 *       a plane using RANSAC.
 * param: (in) sourceCloud - ptr to input cloud
 *        (out) objectsCloud - returns cloud with objects above plane
 *        (in) leafSize - size of voxel in cm^3. (ex. 0.01 = 1 cm)
 *        (in) neighborsToAnalyze - number of nearest neighbors to analyze (ex. 50)
 *        (in) stdDeviation - standard deviation used to find outliers (ex. 1.0)   
 *        (in) minRadius - minimum radius between plane and camera in radians (ex. 0)
 *        (in) maxRadius - maximum radius between plane and camera in radians (ex. 1.57)
 *        (in) ransacIterations - maximum amount of times RANSAC can estimate plane (ex. 1000)
 *        (in) thresholdDistance - range of points considered inliers from plane model (ex 0.01 = 1cm)
 *        (in) minObjectDistance - lowest point of objects above a plane (ex 0.01 = 1cm)
 *        (in) maxObjectDistance - highest point of objects above a plane (ex 0.2 = 20cm)
 * pre-cond: source cloud must contain a non-empty PCD file. Both sourceCloud and objectsCloud
 *           must have objects initialized before being passed as parameters.
 * post-cond: filtered data is passed back through objectsCloud
 * ret: -1 if a plane could not be found else
 *      -2 if source PCD is empty
 *       0 if succesfull
 */
int segObjects(pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr objectsCloud,
                        float leafSize, int neighborsToAnalyze, double stdDeviation,
                        double minRadius, double maxRadius,
                        int ransacIterations, double thresholdDistance,
                        double minObjectDistance, double maxObjectDistance);
}



