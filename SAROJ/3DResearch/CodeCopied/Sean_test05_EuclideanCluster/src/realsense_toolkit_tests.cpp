/******************************************************************************
* Author: Sean Hendrickson
* File: realsense_toolkit_tests.cpp
* Last Modified: 29 February 2016
* Description: This file contains implementations for functions meant to test 
*              the c44 namespace. Functions log data by saving information
*              and displaying information to a screen.
******************************************************************************/
#include "realsense_toolkit_tests.h"


///////////////////////////////////////////////////////////////////////////////
/** 
 * desc:  This function tests the execution time of a voxel filter by altering
 *        the size of a voxel's edge.
 * param: (in) sourceCloud - ptr to point cloud 
 */
void test_voxelFilter_leafSize(pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	// test voxelFilter() function
	log_voxelFilter(sourceCloud, cloud_filtered, "../pictures/robot_arm_01.pcd",
	                "voxel filter, 01:", 0.01);
	log_voxelFilter(sourceCloud, cloud_filtered, "../pictures/robot_arm_02.pcd",
	                "voxel filter, 02:", 0.02);
	log_voxelFilter(sourceCloud, cloud_filtered, "../pictures/robot_arm_03.pcd",
	                "voxel filter, 03:", 0.03);
	log_voxelFilter(sourceCloud, cloud_filtered, "../pictures/robot_arm_04.pcd",
	                "voxel filter, 04:", 0.04);
	log_voxelFilter(sourceCloud, cloud_filtered, "../pictures/robot_arm_05.pcd",
	                "voxel filter, 05:", 0.05);	/*
	log_voxelFilter(sourceCloud, cloud_filtered, "../pictures/T02_voxel_06.pcd",
	                "voxel filter, 06:", 0.06);	
	log_voxelFilter(sourceCloud, cloud_filtered, "../pictures/T02_voxel_07.pcd",
	                "voxel filter, 07:", 0.07);	
	log_voxelFilter(sourceCloud, cloud_filtered, "../pictures/T02_voxel_08.pcd",
	                "voxel filter, 08:", 0.08);	
	log_voxelFilter(sourceCloud, cloud_filtered, "../pictures/T02_voxel_09.pcd",
	                "voxel filter, 09:", 0.09);	
	log_voxelFilter(sourceCloud, cloud_filtered, "../pictures/T02_voxel_10.pcd",
	                "voxel filter, 10:", 0.10);		*/
}


///////////////////////////////////////////////////////////////////////////////
/** 
 * desc:  This function displays statistics of an analytical noise removal
 *        function when the parameter determining the number of nearest neighbors
 *        to be analyzed is altered.
 * param: (in) sourceCloud - ptr to point cloud 
 */
void test_removeNoise_neighbors(pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	// test removeNoise() function's nearest neighbors parameter
	log_removeNoise(sourceCloud, cloud_filtered, "../pictures/T03_noiseNeighbors_01.pcd",
	                "noise neighbors, 01: ", 10, 1.0);
	log_removeNoise(sourceCloud, cloud_filtered, "../pictures/T03_noiseNeighbors_02.pcd",
	                "noise neighbors, 02: ", 20, 1.0);
	log_removeNoise(sourceCloud, cloud_filtered, "../pictures/T03_noiseNeighbors_03.pcd",
	                "noise neighbors, 03: ", 30, 1.0);
	log_removeNoise(sourceCloud, cloud_filtered, "../pictures/T03_noiseNeighbors_04.pcd",
	                "noise neighbors, 04: ", 40, 1.0);
	log_removeNoise(sourceCloud, cloud_filtered, "../pictures/T03_noiseNeighbors_05.pcd",
	                "noise neighbors, 05: ", 50, 1.0);
	log_removeNoise(sourceCloud, cloud_filtered, "../pictures/T03_noiseNeighbors_06.pcd",
	                "noise neighbors, 06: ", 60, 1.0);
	log_removeNoise(sourceCloud, cloud_filtered, "../pictures/T03_noiseNeighbors_07.pcd",
	                "noise neighbors, 07: ", 70, 1.0);
	log_removeNoise(sourceCloud, cloud_filtered, "../pictures/T03_noiseNeighbors_08.pcd",
	                "noise neighbors, 08: ", 80, 1.0);
	log_removeNoise(sourceCloud, cloud_filtered, "../pictures/T03_noiseNeighbors_09.pcd",
	                "noise neighbors, 09: ", 90, 1.0);
	log_removeNoise(sourceCloud, cloud_filtered, "../pictures/T03_noiseNeighbors_10.pcd",
	                "noise neighbors, 10: ", 100, 1.0);
}


///////////////////////////////////////////////////////////////////////////////
/** 
 * desc:  This function displays statistics of an analytical noise removal
 *        function when the parameter determining the standard deviation is altered
 * param: (in) sourceCloud - ptr to point cloud 
 */
void test_removeNoise_stdDeviation(pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	// test removeNoise() function's threshold parameter
	log_removeNoise(sourceCloud, cloud_filtered, "../pictures/T03_noiseThreshold_01.pcd",
	                "noise threshold, 01: ", 50, 0.1);
	log_removeNoise(sourceCloud, cloud_filtered, "../pictures/T03_noiseThreshold_02.pcd",
	                "noise threshold, 02: ", 50, 0.3);
	log_removeNoise(sourceCloud, cloud_filtered, "../pictures/T03_noiseThreshold_03.pcd",
	                "noise threshold, 03: ", 50, 0.5);
	log_removeNoise(sourceCloud, cloud_filtered, "../pictures/T03_noiseThreshold_04.pcd",
	                "noise threshold, 04: ", 50, 0.7);
	log_removeNoise(sourceCloud, cloud_filtered, "../pictures/T03_noiseThreshold_05.pcd",
	                "noise threshold, 05: ", 50, 0.9);
	log_removeNoise(sourceCloud, cloud_filtered, "../pictures/T03_noiseThreshold_06.pcd",
	                "noise threshold, 06: ", 50, 1.1);
	log_removeNoise(sourceCloud, cloud_filtered, "../pictures/T03_noiseThreshold_07.pcd",
	                "noise threshold, 07: ", 50, 1.3);
	log_removeNoise(sourceCloud, cloud_filtered, "../pictures/T03_noiseThreshold_08.pcd",
	                "noise threshold, 08: ", 50, 1.5);
	log_removeNoise(sourceCloud, cloud_filtered, "../pictures/T03_noiseThreshold_09.pcd",
	                "noise threshold, 09: ", 50, 1.7);
	log_removeNoise(sourceCloud, cloud_filtered, "../pictures/T03_noiseThreshold_10.pcd",
	                "noise threshold, 10: ", 50, 1.9);
	log_removeNoise(sourceCloud, cloud_filtered, "../pictures/T03_noiseThreshold_11.pcd",
	                "noise threshold, 11: ", 50, 2.1);		
}


///////////////////////////////////////////////////////////////////////////////
/** 
 * desc:  This function returns statistics for the getPrism() function when the number
 *        of ransac iterations is altered.
 * param: (in) sourceCloud - ptr to point cloud 
 */
void test_getPrism_iterations(pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_objects (new pcl::PointCloud<pcl::PointXYZ>);

	log_getPrism(sourceCloud, cloud_objects, "../pictures/T05_prismIteration_01.pcd",
	             "prism Iterations, 01: ", 0, 1.57, 200, 0.01, 0.02, 0.2);
	log_getPrism(sourceCloud, cloud_objects, "../pictures/T05_prismIteration_02.pcd",
	             "prism Iterations, 02: ", 0, 1.57, 400, 0.01, 0.02, 0.2);
	log_getPrism(sourceCloud, cloud_objects, "../pictures/T05_prismIteration_03.pcd",
	             "prism Iterations, 03: ", 0, 1.57, 600, 0.01, 0.02, 0.2);
	log_getPrism(sourceCloud, cloud_objects, "../pictures/T05_prismIteration_04.pcd",
	             "prism Iterations, 04: ", 0, 1.57, 800, 0.01, 0.02, 0.2);
	log_getPrism(sourceCloud, cloud_objects, "../pictures/T05_prismIteration_05.pcd",
	             "prism Iterations, 05: ", 0, 1.57, 1000, 0.01, 0.02, 0.2);
	log_getPrism(sourceCloud, cloud_objects, "../pictures/T05_prismIteration_06.pcd",
	             "prism Iterations, 06: ", 0, 1.57, 1200, 0.01, 0.02, 0.2);
	log_getPrism(sourceCloud, cloud_objects, "../pictures/T05_prismIteration_07.pcd",
	             "prism Iterations, 07: ", 0, 1.57, 1400, 0.01, 0.02, 0.2);
	log_getPrism(sourceCloud, cloud_objects, "../pictures/T05_prismIteration_08.pcd",
	             "prism Iterations, 08: ", 0, 1.57, 1600, 0.01, 0.02, 0.2);
	log_getPrism(sourceCloud, cloud_objects, "../pictures/T05_prismIteration_09.pcd",
	             "prism Iterations, 09: ", 0, 1.57, 1800, 0.01, 0.02, 0.2);
	log_getPrism(sourceCloud, cloud_objects, "../pictures/T05_prismIteration_10.pcd",
	             "prism Iterations, 10: ", 0, 1.57, 2000, 0.01, 0.02, 0.2);
}


///////////////////////////////////////////////////////////////////////////////
/** 
 * desc:  This function returns statistics for the getPrism() function when the 
 *        threshold distance is altered.
 * param: (in) sourceCloud - ptr to point cloud 
 */
void test_getPrism_threshold(pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_objects (new pcl::PointCloud<pcl::PointXYZ>);

	log_getPrism(sourceCloud, cloud_objects, "../pictures/T05_prismThreshold_01.pcd",
	             "prism threshold, 01: ", 0, 1.57, 1000, 0.01, 0.02, 0.2);
	log_getPrism(sourceCloud, cloud_objects, "../pictures/T05_prismThreshold_02.pcd",
	             "prism threshold, 02: ", 0, 1.57, 1000, 0.02, 0.02, 0.2);
	log_getPrism(sourceCloud, cloud_objects, "../pictures/T05_prismThreshold_03.pcd",
	             "prism threshold, 03: ", 0, 1.57, 1000, 0.03, 0.02, 0.2);
	log_getPrism(sourceCloud, cloud_objects, "../pictures/T05_prismThreshold_04.pcd",
	             "prism threshold, 04: ", 0, 1.57, 1000, 0.04, 0.02, 0.2);
	log_getPrism(sourceCloud, cloud_objects, "../pictures/T05_prismThreshold_05.pcd",
	             "prism threshold, 05: ", 0, 1.57, 1000, 0.05, 0.02, 0.2);
	log_getPrism(sourceCloud, cloud_objects, "../pictures/T05_prismThreshold_06.pcd",
	             "prism threshold, 06: ", 0, 1.57, 1000, 0.06, 0.02, 0.2);
	log_getPrism(sourceCloud, cloud_objects, "../pictures/T05_prismThreshold_07.pcd",
	             "prism threshold, 07: ", 0, 1.57, 1000, 0.07, 0.02, 0.2);
	log_getPrism(sourceCloud, cloud_objects, "../pictures/T05_prismThreshold_08.pcd",
	             "prism threshold, 08: ", 0, 1.57, 1000, 0.08, 0.02, 0.2);
	log_getPrism(sourceCloud, cloud_objects, "../pictures/T05_prismThreshold_09.pcd",
	             "prism threshold, 09: ", 0, 1.57, 1000, 0.09, 0.02, 0.2);
	log_getPrism(sourceCloud, cloud_objects, "../pictures/T05_prismThreshold_10.pcd",
	             "prism threshold, 10: ", 0, 1.57, 1000, 0.10, 0.02, 0.2);
}


///////////////////////////////////////////////////////////////////////////////
/**
 * desc: This function returns logging information from log_segmentObjects().
 * param: (in) sourceCloud - ptr to point cloud to be tested
 */
void test_segmentObjects(pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_objects (new pcl::PointCloud<pcl::PointXYZ>);
	
	// begin log_segmentObjects() tests
	// default values on pcl tutorials
	log_segmentObjects(sourceCloud, cloud_objects, "../pictures/T06_objectSeg_01.pcd",
	                   "object segmentation, 01: ", 0.01, 50, 1.0, 0, 1.57,
	                   1000, 0.01, 0.02, 0.2);

	// RANSAC iterations altered
	log_segmentObjects(sourceCloud, cloud_objects, "../pictures/T06_objectSeg_iter_01.pcd",
	                   "obj seg, Iterations, 01: ", 0.01, 50, 1.0, 0, 1.57,
	                   200, 0.01, 0.02, 0.2);
	log_segmentObjects(sourceCloud, cloud_objects, "../pictures/T06_objectSeg_iter_02.pcd",
	                   "obj seg, Iterations, 02: ", 0.01, 50, 1.0, 0, 1.57,
	                   400, 0.01, 0.02, 0.2);
	log_segmentObjects(sourceCloud, cloud_objects, "../pictures/T06_objectSeg_iter_03.pcd",
	                   "obj seg, Iterations, 03: ", 0.01, 50, 1.0, 0, 1.57,
	                   600, 0.01, 0.02, 0.2);
	log_segmentObjects(sourceCloud, cloud_objects, "../pictures/T06_objectSeg_iter_04.pcd",
	                   "obj seg, Iterations, 04: ", 0.01, 50, 1.0, 0, 1.57,
	                   800, 0.01, 0.02, 0.2);

	// voxel filter leaf size altered
	log_segmentObjects(sourceCloud, cloud_objects, "../pictures/T06_objectSeg_voxel_01.pcd",
	                   "obj seg, voxel, 01: ", 0.01, 50, 1.0, 0, 1.57,
	                   500, 0.01, 0.02, 0.2);
	log_segmentObjects(sourceCloud, cloud_objects, "../pictures/T06_objectSeg_voxel_02.pcd",
	                   "obj seg, voxel, 02: ", 0.02, 50, 1.0, 0, 1.57,
	                   500, 0.01, 0.02, 0.2);
	log_segmentObjects(sourceCloud, cloud_objects, "../pictures/T06_objectSeg_voxel_03.pcd",
	                   "obj seg, voxel, 03: ", 0.03, 50, 1.0, 0, 1.57,
	                   500, 0.01, 0.02, 0.2);
	log_segmentObjects(sourceCloud, cloud_objects, "../pictures/T06_objectSeg_voxel_04.pcd",
	                   "obj seg, voxel, 04: ", 0.04, 50, 1.0, 0, 1.57,
	                   500, 0.01, 0.02, 0.2);

	// optimized inputs based on tests so far
	log_segmentObjects(sourceCloud, cloud_objects, "../pictures/T06_optimized_01.pcd",
	                   "obj seg, optimized, 01: ", 0.02, 20, 1.0, 0, 1.57,
	                   500, 0.01, 0.02, 0.2);
}


///////////////////////////////////////////////////////////////////////////////
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
	                   const std::string& axis, float minPoint, float maxPoint)
{
	pcl::PCDWriter writer;  // for writting pcd to disk

	// enable variables for time logging
	boost::posix_time::ptime time_before_execution;
	boost::posix_time::ptime time_after_execution;
	boost::posix_time::time_duration difference;

	time_before_execution = boost::posix_time::microsec_clock::local_time();  // time before
	c44::passthroughFilter(sourceCloud,filteredCloud, axis, minPoint, maxPoint);  // call filter
	time_after_execution = boost::posix_time::microsec_clock::local_time(); // time after
	writer.write<pcl::PointXYZ> (pcdFileName, *filteredCloud); // write out cloud
	#ifdef DEBUG
	difference = time_after_execution - time_before_execution;  // get execution time
	std::cout << std::setw(5) << difference.total_milliseconds() << ": "
	          << flavorText << filteredCloud->points.size() << " points" << std::endl;
	#endif
}
						   

///////////////////////////////////////////////////////////////////////////////
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
                     float leafSize)
{
	pcl::PCDWriter writer;  // for writting pcd to disk	

	// enable variables for time logging
	boost::posix_time::ptime time_before_execution;
	boost::posix_time::ptime time_after_execution;
	boost::posix_time::time_duration difference;
	
	time_before_execution = boost::posix_time::microsec_clock::local_time();  // time before
	c44::voxelFilter(sourceCloud, filteredCloud, leafSize);  // call voxel filter
	time_after_execution = boost::posix_time::microsec_clock::local_time(); // time after
	writer.write<pcl::PointXYZ> (pcdFileName, *filteredCloud); // write out cloud
	#ifdef DEBUG
	difference = time_after_execution - time_before_execution;  // get execution time
	std::cout << std::setw(5) << difference.total_milliseconds() << ": "
	          << flavorText << filteredCloud->points.size() << " points" << std::endl;
	#endif
}
					 

///////////////////////////////////////////////////////////////////////////////
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
                     int neighborsToAnalyze, double stdDeviation)
{
	pcl::PCDWriter writer;  // for writting pcd to disk

	// enable variables for time logging
	boost::posix_time::ptime time_before_execution;
	boost::posix_time::ptime time_after_execution;
	boost::posix_time::time_duration difference;
	
	time_before_execution = boost::posix_time::microsec_clock::local_time();  // time before
	c44::removeNoise(sourceCloud, filteredCloud, neighborsToAnalyze, stdDeviation);  // call noise removal
	time_after_execution = boost::posix_time::microsec_clock::local_time(); // time after
	writer.write<pcl::PointXYZ> (pcdFileName, *filteredCloud); // write out cloud
	#ifdef DEBUG
	difference = time_after_execution - time_before_execution;  // get execution time
	std::cout << std::setw(5) << difference.total_milliseconds() << ": "
	          << flavorText << filteredCloud->points.size() << " points" << std::endl;
	#endif
}

					 
///////////////////////////////////////////////////////////////////////////////
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
                  const std::string& pcdFileName, const std::string& flavorText,
                  double minRadius, double maxRadius,				
                  int ransacIterations, double thresholdDistance)
{
	pcl::PCDWriter writer;  // for writting pcd to disk

	// enable variables for time logging
	boost::posix_time::ptime time_before_execution;
	boost::posix_time::ptime time_after_execution;
	boost::posix_time::time_duration difference;
	
	// create new cloud to hold original cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_unaltered (new pcl::PointCloud<pcl::PointXYZ>);
	
	pcl::copyPointCloud(*sourceCloud, *cloud_unaltered);  // save original cloud
	time_before_execution = boost::posix_time::microsec_clock::local_time();  // time before
	int result = c44::getPlane(cloud_unaltered, planeCloud, outliersCloud, coefficients, 
	                           indices, minRadius, maxRadius, ransacIterations, thresholdDistance);

	time_after_execution = boost::posix_time::microsec_clock::local_time(); // time after
	if(result < 0)  // default settings
	{ 
		std::cout << "ERROR: No plane found" << std::endl;
	}
	else
	{
		difference = time_after_execution - time_before_execution;  // get execution time
		writer.write<pcl::PointXYZ> (pcdFileName, *planeCloud); // write out cloud
		#ifdef DEBUG
		std::cout << std::setw(5) << difference.total_milliseconds() << ": "
		          << flavorText << planeCloud->points.size() << " points" << std::endl;
		#endif
	}	
}

				  
///////////////////////////////////////////////////////////////////////////////
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
                  double minObjectDistance, double maxObjectDistance)
{
	pcl::PCDWriter writer;  // for writting pcd to disk
	int result = 0;

	// enable variables for time logging
	boost::posix_time::ptime time_before_execution;
	boost::posix_time::ptime time_after_execution;
	boost::posix_time::time_duration difference;
	
	// run getPrism() function and record execution time
	time_before_execution = boost::posix_time::microsec_clock::local_time();  // time before
	result = c44::getPrism(sourceCloud, objectsCloud, minRadius, maxRadius, ransacIterations, 
	                       thresholdDistance, minObjectDistance, maxObjectDistance);
	time_after_execution = boost::posix_time::microsec_clock::local_time(); // time after
	if(result < 0)
	{
		std::cout << "ERROR: could not find polygonal prism data" << std::endl;
	}
	else
	{
		writer.write<pcl::PointXYZ> (pcdFileName, *objectsCloud); // write out cloud
		#ifdef DEBUG
		difference = time_after_execution - time_before_execution;  // get execution time
		std::cout << std::setw(5) << difference.total_milliseconds() << ": "
		          << flavorText << objectsCloud->points.size() << " points" << std::endl;
		#endif
	}
}


///////////////////////////////////////////////////////////////////////////////
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
                        double minObjectDistance, double maxObjectDistance)
{
	pcl::PCDWriter writer;  // for writting pcd to disk
	int result = 0;         // catch return from function calls

	// enable variables for time logging
	boost::posix_time::ptime time_before_execution;
	boost::posix_time::ptime time_after_execution;
	boost::posix_time::time_duration difference;
	
	// capture time before execution
	time_before_execution = boost::posix_time::microsec_clock::local_time();
	
	result = c44::segObjects(sourceCloud, objectsCloud, leafSize, neighborsToAnalyze, 
	                         stdDeviation, minRadius, maxRadius, ransacIterations, 
	                         thresholdDistance, minObjectDistance, maxObjectDistance);
						   
	// check for errors from getPrism() 
	if(result < 0)  
	{
		std::cout << "ERROR: could not find polygonal prism data" << std::endl;
		return;
	}
	
	// capture time at end of execution
	time_after_execution = boost::posix_time::microsec_clock::local_time(); 
	
	// display and write out logging information
	writer.write<pcl::PointXYZ> (pcdFileName, *objectsCloud); // write out cloud
	#ifdef DEBUG
	difference = time_after_execution - time_before_execution;  // get execution time
	std::cout << std::setw(5) << difference.total_milliseconds() << ": "
	<< flavorText << objectsCloud->points.size() << " points" << std::endl;
	#endif
}
