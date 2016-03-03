/******************************************************************************
* Author: Sean Hendrickson
* File: realsense_toolkit_tests.h
* Last Modified: 29 February 2016
* Description: This file contains prototypes for functions meant to test 
*              the c44 namespace. Functions log data by saving information
*              and displaying information to a screen.
******************************************************************************/
#include "realsense_toolkit.h"
#include <iomanip>
#include <stdlib.h>
#define DEBUG

/***** TEST FUNCTIONS *****/
/** 
 * desc:  This function tests the execution time of a voxel filter by altering
 *        the size of a voxel's edge.
 * param: (in) sourceCloud - ptr to point cloud 
 */
void test_voxelFilter_leafSize(pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud);

/** 
 * desc:  This function displays statistics of an analytical noise removal
 *        function when the parameter determining the number of nearest neighbors
 *        to be analyzed is altered.
 * param: (in) sourceCloud - ptr to point cloud 
 */
void test_removeNoise_neighbors(pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud);

/** 
 * desc:  This function displays statistics of an analytical noise removal
 *        function when the parameter determining the standard deviation is altered
 * param: (in) sourceCloud - ptr to point cloud 
 */
void test_removeNoise_stdDeviation(pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud);

/** 
 * desc:  This function returns statistics for the getPrism() function when the number
 *        of ransac iterations is altered.
 * param: (in) sourceCloud - ptr to point cloud 
 */
void test_getPrism_iterations(pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud);

/** 
 * desc:  This function returns statistics for the getPrism() function when the 
 *        threshold distance is altered.
 * param: (in) sourceCloud - ptr to point cloud 
 */
void test_getPrism_threshold(pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud);

/**
 * desc: This function returns logging information from log_segmentObjects().
 * param: (in) sourceCloud - ptr to point cloud to be tested
 */
void test_segmentObjects(pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud);

/***** DATA LOGGING FUNCTIONS *****/
/** 
 * desc:  This function displays the execution time of a passthrough filter and 
 *        saves the resulting pcd file.
 * param: (in)  sourceCloud - ptr to point cloud 
 *        (out) filteredCloud - ptr to cloud after being filtered
 *        (in)  pcdFileName - name and location of file (e.g ../foo/foo.pcd)
 *        (in)  flavorText - text displayed after execution time
 *        (in)  axis - axis to be filtered along (e.g. "x", "y", "z")
 *        (in)  minPoint - Beginning of range to keep. Cuttoff before this point
 *        (in)  maxPoint - End of range to keep. Cutoff past this point.
 */
void log_passthroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud,
	                       pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud,
                               const std::string& pcdFileName, const std::string& flavorText,
	                       const std::string& axis, float minPoint, float maxPoint);

/** 
 * desc:  This function displays the execution time of a voxel filter by altering
 *        the size of a voxel's edge.
 * param: (in) sourceCloud - ptr to point cloud 
 *        (out) filteredCloud - ptr to cloud after being filtered
 *        (in)  pcdFileName - name and location of file (e.g ../foo/foo.pcd)
 *        (in)  flavorText - text displayed after execution time
 *        (in)  leafSize - size of voxel edge (e.g. 0.01 = 1cm)
 */
void log_voxelFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud,
                     const std::string& pcdFileName, const std::string& flavorText,
                     float leafSize);

/** 
 * desc:  This function displays the execution time of a noise removal filter.
 * param: (in) sourceCloud - ptr to point cloud 
 *        (out) filteredCloud - ptr to cloud after noise is removed
 *        (in)  pcdFileName - name and location of file (e.g ../foo/foo.pcd)
 *        (in)  flavorText - text displayed after execution time
 *        (in)  neighborsToAnalyze - number of nearest neighbors to be analyzed per point
 *                                   when looking for noise
 *        (in)  stdDeviation - the standard deviation used by a gausian filter to remoive noise
 */
void log_removeNoise(pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud,
                     const std::string& pcdFileName, const std::string& flavorText,
                     int neighborsToAnalyze, double stdDeviation);

/** 
 * desc:  This function tests the execution time of a RANSAC based search 
 *        algorithm used to find a plane in a PointCloud.
 * param: (in) sourceCloud - ptr to point cloud
 *        (out) planeCloud - ptr to points found in a plane
 *        (out) outliersCloud - ptr to points found outside of a plane
 *        (in)  pcdFileName - name and location of file (e.g ../foo/foo.pcd)
 *        (in)  flavorText - text displayed after execution time
 *        (in)  minRadius - minimum radius between camera and valid planes
 *        (in)  maxRadius - maximum radius between camera and valid planes
 *        (in)  ransacIterations - maximum number of RANSAC iteration to do 
 *                                 while looking for largest plane  
 *        (in)  thresholdDistance - width of inliers to be included in a 
 *                                  discovered plane.
 */	
void log_getPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud,  
                  pcl::PointCloud<pcl::PointXYZ>::Ptr planeCloud,                 
                  pcl::PointCloud<pcl::PointXYZ>::Ptr outliersCloud,              
                  pcl::ModelCoefficients::Ptr coefficients,                       
                  pcl::PointIndices::Ptr indices,
                  const std::string& pcdFileName, 
                  const std::string& flavorText,
                  double minRadius, double maxRadius,				
                  int ransacIterations, double thresholdDistance);

/** 
 * desc:  This function tests the execution time of a RANSAC based search 
 *        algorithm used to find objects on top of a plane in a PointCloud.
 * param: (in) sourceCloud - ptr to point cloud
 *        (out) objectsCloud - ptr to points found above a plane
 *        (in)  pcdFileName - name and location of file (e.g ../foo/foo.pcd)
 *        (in)  flavorText - text displayed after execution time
 *        (in)  minRadius - minimum radius between camera and valid planes
 *        (in)  maxRadius - maximum radius between camera and valid planes
 *        (in)  ransacIterations - maximum number of RANSAC iteration to do 
 *                                 while looking for largest plane  
 *        (in)  thresholdDistance - width of inliers to be included in a 
 *                                  discovered plane.
 *        (in)  minObjectDistance - minimum distance of points to be included above a plane
 *        (in)  maxObjectDistance - maximum distance of points to be included above a plane
 */
void log_getPrism(pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud,
                  pcl::PointCloud<pcl::PointXYZ>::Ptr objectsCloud,             
                  const std::string& pcdFileName, const std::string& flavorText,				  
                  double minRadius, double maxRadius,				
                  int ransacIterations, double thresholdDistance, 
                  double minObjectDistance, double maxObjectDistance);

/**
 * desc: This function takes a source PCD image an segments out objects found 
 *       on a plane. The source image is downsampled and filtered prior to finding
 *       a plane using RANSAC.
 * param: (in) sourceCloud - ptr to input cloud
 *        (out) objectsCloud - returns cloud with objects above plane
 *        (in)  pcdFileName - name and location of file (e.g ../foo/foo.pcd)
 *        (in)  flavorText - text displayed after execution time
 *        (in) leafSize - size of voxel in cm^3. (ex. 0.01 = 1 cm)
 *        (in) neighborsToAnalyze - number of nearest neighbors to analyze (ex. 50)
 *        (in) stdDeviation - standard deviation used to find outliers (ex. 1.0)   
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
void log_segmentObjects(pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr objectsCloud,
                        const std::string& pcdFileName, const std::string& flavorText,
                        float leafSize, int neighborsToAnalyze, double stdDeviation,
                        double minRadius, double maxRadius,
                        int ransacIterations, double thresholdDistance,
                        double minObjectDistance, double maxObjectDistance);				  


