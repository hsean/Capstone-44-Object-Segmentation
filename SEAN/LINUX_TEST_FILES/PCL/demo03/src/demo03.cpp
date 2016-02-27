/******************************************************************************
* Author: Sean Hendrickson
* File: demo03.cpp
* Last Modified: 26 February 2016
* Description: This file gets benchmark data from the realsense_toolkit 
*              functions.  The benchmarks are based on PCD files taken
*              by a RealSense camera.
******************************************************************************/
#include "realsense_toolkit.h"
#include <iomanip>
#include <stdlib.h>

#define LINUX
#define DEBUG
//#define DEBUG_PLANE
#define Z_CUTOFF 2.5     // 1 meter from camera


/* wrapper functions */
void log_passthroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud,
	                       pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud,
                               const std::string& pcdFileName, const std::string& flavorText,
	                       const std::string& axis, float minPoint, float maxPoint);
void log_voxelFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud,
                     const std::string& pcdFileName, const std::string& flavorText,
                     float leafSize);
void log_removeNoise(pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud,
                     const std::string& pcdFileName, const std::string& flavorText,
                     int neighborsToAnalyze, double stdDeviation);
void log_getPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud,  
                  pcl::PointCloud<pcl::PointXYZ>::Ptr planeCloud,                 
                  pcl::PointCloud<pcl::PointXYZ>::Ptr outliersCloud,              
                  pcl::ModelCoefficients::Ptr coefficients,                       
                  pcl::PointIndices::Ptr indices,
                  const std::string& pcdFileName, 
                  const std::string& flavorText,
                  double minRadius, double maxRadius,				
                  int ransacIterations, double thresholdDistance);
void log_getPrism(pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud,
                  pcl::PointCloud<pcl::PointXYZ>::Ptr objectsCloud,             
                  const std::string& pcdFileName, const std::string& flavorText,				  
                  double minRadius, double maxRadius,				
                  int ransacIterations, double thresholdDistance, 
                  double minObjectDistance, double maxObjectDistance);
			 

int main(int argc, char** argv)
{
	// initialize variables
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_original (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_unaltered (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_objects (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_planeInliers (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_planeOutliers (new pcl::PointCloud<pcl::PointXYZ>);             
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());                  
	pcl::PointIndices::Ptr indices (new pcl::PointIndices());
	pcl::PCDReader reader;
	pcl::PCDWriter writer;
	int result = 0;  // catch function return

	// enable variables for time logging
	boost::posix_time::ptime time_before_execution;
	boost::posix_time::ptime time_after_execution;
	boost::posix_time::time_duration difference;
	
	// read point cloud through command line
	if (argc != 2)
	{
		std::cout << "usage: " << argv[0] << " <filename>\n";
		return 0;
	}
	else
	{	// read cloud and display its size
		std::cout << "Reading Point Cloud" << std::endl;
		reader.read(argv[1], *cloud_original);
		#ifdef DEBUG
		std::cout << "size of original cloud: " 
		          << cloud_original->points.size() << " points" << std::endl;
		#endif
	}
	
	//**************************************************************************
	// test passthroughFilter() function
	std::cout << "RUNNING PASSTHROUGH FILTER TESTS" << std::endl;
	log_passthroughFilter(cloud_original, cloud_filtered, "../pictures/T01_passthrough_01.pcd",
	                      "passthrough filter, custom 1: ","z", -2.5, 0);
	log_passthroughFilter(cloud_original, cloud_filtered, "../pictures/T01_passthrough_02.pcd",
	                      "passthrough filter, custom 2: ","y", -3.0, 3.0);
	log_passthroughFilter(cloud_original, cloud_filtered, "../pictures/T01_passthrough_03.pcd", 
	                      "passthrough filter, custom 3: ","x", -3.0, 3.0);
	
	//**************************************************************************
	// test voxelFilter() function
	std::cout << "RUNNING VOXEL FILTER TEST" << std::endl;
	log_voxelFilter(cloud_original, cloud_filtered, "../pictures/T02_voxel_01.pcd",
	                "voxel filter, custom 1:", 0.01);
	log_voxelFilter(cloud_original, cloud_filtered, "../pictures/T02_voxel_02.pcd",
                        "voxel filter, custom 2:", 0.02);
	log_voxelFilter(cloud_original, cloud_filtered, "../pictures/T02_voxel_03.pcd",
                        "voxel filter, custom 3:", 0.04);
	log_voxelFilter(cloud_original, cloud_filtered, "../pictures/T02_voxel_04.pcd",
                        "voxel filter, custom 4:", 0.08);				
	
	//**************************************************************************
	// test removeNoise() function
	std::cout << "RUNNING NOISE REMOVAL TEST" << std::endl;
	log_removeNoise(cloud_original, cloud_filtered, "../pictures/T03_noise_01.pcd",
	                "noise removal, custom 1: ", 50, 1.0);
	log_removeNoise(cloud_original, cloud_filtered, "../pictures/T03_noise_01.pcd",
	                "noise removal, custom 1: ", 100, 1.0);
	log_removeNoise(cloud_original, cloud_filtered, "../pictures/T03_noise_01.pcd",
	                "noise removal, custom 1: ", 10, 1.0);
	log_removeNoise(cloud_original, cloud_filtered, "../pictures/T03_noise_01.pcd",
	                "noise removal, custom 1: ", 50, 1.9);
	log_removeNoise(cloud_original, cloud_filtered, "../pictures/T03_noise_01.pcd",
	                "noise removal, custom 1: ", 50, 0.1);				
	
	//**************************************************************************
	// test getPlane() function
	std::cout << "RUNNING PLANE SEGMENTATION TEST" << std::endl;
	log_getPlane(cloud_original, cloud_planeInliers, cloud_planeOutliers,
	             coefficients, indices, "../pictures/T04_planeInliers_01.pcd",
	             "plane segmentation, custom 1: ", 0, 1.57, 1000, 0.01);
	log_getPlane(cloud_original, cloud_planeInliers, cloud_planeOutliers,
	             coefficients, indices, "../pictures/T04_planeInliers_02.pcd",
	             "plane segmentation, custom 2: ", 0, 1.57, 1, 0.01);
	log_getPlane(cloud_original, cloud_planeInliers, cloud_planeOutliers,
	             coefficients, indices, "../pictures/T04_planeInliers_03.pcd",
	             "plane segmentation, custom 3: ", 0, 1.57, 2000, 0.01);
	log_getPlane(cloud_original, cloud_planeInliers, cloud_planeOutliers,
	             coefficients, indices, "../pictures/T04_planeInliers_04.pcd",
	             "plane segmentation, custom 4: ", 0, 0.76, 1000, 0.01);
	log_getPlane(cloud_original, cloud_planeInliers, cloud_planeOutliers,
	             coefficients, indices, "../pictures/T04_planeInliers_05.pcd",
	             "plane segmentation, custom 5: ", 0, 2.09, 1000, 0.01);
	log_getPlane(cloud_original, cloud_planeInliers, cloud_planeOutliers,
	             coefficients, indices, "../pictures/T04_planeInliers_06.pcd",
	             "plane segmentation, custom 6: ", 0.35, 1.57, 1000, 0.01);
	log_getPlane(cloud_original, cloud_planeInliers, cloud_planeOutliers,
	             coefficients, indices, "../pictures/T04_planeInliers_07.pcd",
	             "plane segmentation, custom 7: ", 0.79, 1.57, 1000, 0.01);
	log_getPlane(cloud_original, cloud_planeInliers, cloud_planeOutliers,
	             coefficients, indices, "../pictures/T04_planeInliers_08.pcd",
	             "plane segmentation, custom 8: ", 0, 1.57, 1000, 0.001);
	log_getPlane(cloud_original, cloud_planeInliers, cloud_planeOutliers,
	             coefficients, indices, "../pictures/T04_planeInliers_09.pcd",
	             "plane segmentation, custom 9: ", 0, 1.57, 1000, 0.01);			 

	//**************************************************************************
	// test getPrism() function
	/*
	std::cout << "RUNNING EXTRACT PRISM TEST" << std::endl;
	log_getPrism(cloud_original, cloud_objects, "../pictures/T05_objects_01.pcd",
	             "polygonal prism, custom 1: ", 0, 1.57, 1000, 0.01, 0.02, 0.2);
	*/

	// test getPrism() function
	std::cout << "RUNNING EXTRACT PRISM TEST" << std::endl;
	// get output from default plane
	time_before_execution = boost::posix_time::microsec_clock::local_time();  // time before
	result = c44::getPrism(cloud_original, cloud_objects, 0, 1.57, 1000, 0.01, 0.02, 0.2);
	time_after_execution = boost::posix_time::microsec_clock::local_time(); // time after
	if(result < 0)
	{
		std::cout << "ERROR: could not find polygonal prism data" << std::endl;
	}
	else
	{
		writer.write<pcl::PointXYZ> ("../pictures/T05_objects_01.pcd", *cloud_objects); // write out cloud
		#ifdef DEBUG
		difference = time_after_execution - time_before_execution;  // get execution time
		std::cout << std::setw(5) << difference.total_milliseconds() << ": "
		          << "polygonal prism, default: " 
		          << cloud_objects->points.size() << " points" << std::endl;
		#endif
	}

	return 0;
}


/* WRAPPER IMPLEMENTATION */
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
				  
				  
void log_getPrism(pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud,
                  pcl::PointCloud<pcl::PointXYZ>::Ptr objectsCloud,
                  const std::string& pcdFileName, const std::string& flavorText,
                  double minRadius, double maxRadius,				
                  int ransacIterations, double thresholdDistance, 
                  double minObjectDistance, double maxObjectDistance)
{
	std::cout << "LOG PRISM: ENTERING FUNCTION" << std::endl;
	pcl::PCDWriter writer;  // for writting pcd to disk
	int result = 0;

	// enable variables for time logging
	boost::posix_time::ptime time_before_execution;
	boost::posix_time::ptime time_after_execution;
	boost::posix_time::time_duration difference;
	
        // create new cloud to hold original cloud
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_unaltered (new pcl::PointCloud<pcl::PointXYZ>);

        //pcl::copyPointCloud(*sourceCloud, *sourceCloud);  // save original cloud
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
	std::cout << "LOG PRISM: LEAVING FUNCTION" << std::endl;

	/*
	// test getPrism() function
	std::cout << "RUNNING EXTRACT PRISM TEST" << std::endl;
	// get output from default plane
	time_before_execution = boost::posix_time::microsec_clock::local_time();  // time before
	result = c44::getPrism(cloud_original, cloud_objects, 0, 1.57, 1000, 0.01, 0.02, 0.2);
	time_after_execution = boost::posix_time::microsec_clock::local_time(); // time after
	if(result < 0)
	{
		std::cout << "ERROR: could not find polygonal prism data" << std::endl;
	}
	else
	{
		writer.write<pcl::PointXYZ> ("T05_objects_01.pcd", *cloud_objects); // write out cloud
		#ifdef DEBUG
		difference = time_after_execution - time_before_execution;  // get execution time
		std::cout << std::setw(5) << difference.total_milliseconds() << ": "
		          << "polygonal prism, default: " 
		          << cloud_objects->points.size() << " points" << std::endl;
		#endif
	}
	*/
}
				  




