/******************************************************************************
* Author: Sean Hendrickson
* File: demo03.cpp
* Last Modified: 29 February 2016
* Description: This file gets benchmark data from the realsense_toolkit 
*              functions.  The benchmarks are based on PCD files taken
*              by a RealSense camera.
******************************************************************************/
#include "realsense_toolkit.h"
#include "realsense_toolkit_tests.h"

//#define PASSTHROUGH_FILTER_TEST
#define VOXEL_FILTER_TEST
//#define NOISE_REMOVAL_TEST
//#define EXTRACT_PRISM_DATA_TEST
//#define SEGMENT_OBJECTS_TEST
//#define EUCLIDEAN_CLUSTER_TEST

#define LINUX
#define DEBUG
//#define DEBUG_PLANE
#define Z_CUTOFF 2.5     // 1 meter from camera


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
	#ifdef PASSTHROUGH_FILTER_TEST
	std::cout << "RUNNING PASSTHROUGH FILTER TESTS" << std::endl;
	log_passthroughFilter(cloud_original, cloud_filtered, "../pictures/T01_passthrough_01.pcd",
	                      "passthrough filter, custom 1: ","z", -2.5, 0);
	log_passthroughFilter(cloud_original, cloud_filtered, "../pictures/T01_passthrough_02.pcd",
	                      "passthrough filter, custom 2: ","y", -3.0, 3.0);
	log_passthroughFilter(cloud_original, cloud_filtered, "../pictures/T01_passthrough_03.pcd", 
	                      "passthrough filter, custom 3: ","x", -3.0, 3.0);
	#endif
	
	//**************************************************************************
	// test voxelFilter() function
	#ifdef VOXEL_FILTER_TEST
	std::cout << "RUNNING VOXEL FILTER LEAF SIZE TEST" << std::endl;
	test_voxelFilter_leafSize(cloud_original);	
	#endif			
	
	//**************************************************************************
	// test removeNoise() function
	#ifdef NOISE_REMOVAL_TEST
	std::cout << "RUNNING NOISE REMOVAL NEAREST NEIGHBORS TEST" << std::endl;
	test_removeNoise_neighbors(cloud_original);
	std::cout << "RUNNING NOISE REMOVAL THRESHOLD TEST" << std::endl;
	test_removeNoise_stdDeviation(cloud_original);		
	#endif	
				 
	//**************************************************************************
	// test getPrism() function
	#ifdef EXTRACT_PRISM_DATA_TEST
	std::cout << "RUNNING EXTRACT PRISM ITERATIONS TEST" << std::endl;
	test_getPrism_iterations(cloud_original);
	std::cout << "RUNNING EXTRACT PRISM THRESHOLD TEST" << std::endl;
	test_getPrism_threshold(cloud_original);
	#endif
	
	//**************************************************************************
	// test objectSegmentation() function
	#ifdef SEGMENT_OBJECTS_TEST
	std::cout << "RUNNING OBJECT SEGMENTATION TEST" << std::endl;
	test_segmentObjects(cloud_original);
	#endif
	
	//**************************************************************************
	// test objectSegmentation() function
	#ifdef EUCLIDEAN_CLUSTER_TEST
	std::cout << "RUNNING EUCLIDEAN CLUSTER TEST" << std::endl;
	time_before_execution = boost::posix_time::microsec_clock::local_time();  // time before
	c44::segObjects(cloud_original, cloud_objects, 0.02, 20, 1.0, 0, 1.57,
	                500, 0.01, 0.02, 0.2);
	// find clusters
	c44::clusterExtraction(cloud_objects, 0.02, 10, 1000);
	time_after_execution = boost::posix_time::microsec_clock::local_time(); // time after
	//std::cout << "FINNISHED EUCLIDEAN CLUSTER TEST" << std::endl;

		#ifdef DEBUG
		difference = time_after_execution - time_before_execution;  // get execution time
		std::cout << std::setw(5) << difference.total_milliseconds() << ": "
		          << "Euclidean Cluster: " << cloud_objects->points.size() 
		          << " points" << std::endl;
		#endif

	#endif

	return 0;
}
