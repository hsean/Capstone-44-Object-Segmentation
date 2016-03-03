/******************************************************************************
* Author: Sean Hendrickson
* File: realsense_toolkit.cpp
* Last Modified: 26 February 2016
* Description: This file contains implements functions used as part
*              of a toolkit being designed for Intel's RealSense depth camera.
*              This is part of Portland State Universitie's 2015-2016 ECE
*              capstone projects, specifically team #44
******************************************************************************/
#include "realsense_toolkit.h"

///////////////////////////////////////////////////////////////////////////////
/** 
 * desc:  This function cuts off all points outside a given range
 *        on the field passed the the field parameter.
 * param: (in) sourceCloud - ptr to point cloud 
 *        (out) filteredCloud - ptr to cloud after filtering
 *        (in) axis - axis to filter values passed as 'x', 'y', or 'z'
 *        (in) minPoint - kept points are >= minPoint
 *        (in) maxPoint - kept points are <= maxPoint
 */
void c44::passthroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud,
                            const std::string& axis, float minPoint, float maxPoint)
{
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(sourceCloud);
	pass.setFilterFieldName(axis);
	pass.setFilterLimits(minPoint, maxPoint);
	pass.filter(*filteredCloud);
}


///////////////////////////////////////////////////////////////////////////////
/**
 * desc: This function uses a voxel filter to downsample a PCD file.
 * param: (in) sourceCloud - ptr to cloud to be filtered
 *        (out) filteredCloud - ptr to cloud after filtering
 *        (in) leafSize - size of voxel in cm^3. (ex. 0.01 = 1 cm)
 */
void c44::voxelFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud,
                      float leafSize)
{
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(sourceCloud);
	sor.setLeafSize(leafSize, leafSize, leafSize);
	sor.filter(*filteredCloud);
}


///////////////////////////////////////////////////////////////////////////////
/**
 * desc: This function computes the average distance between each point in a PCD
 *       file and then uses the standard deviation to designate points as outliers.
 *       All outliers are then removed from the resultant point cloud.
 * param: (in) sourceCloud - ptr to input cloud
 *        (out) filteredCloud - ptr to cloud with outliers removed
 *        (in) neighborsToAnalyze - number of nearest neighbors to analyze (ex. 50)
 *        (in) stdDeviation - standard deviation used to find outliers (ex. 1.0)   
 */
void c44::removeNoise(pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud,
                      int neighborsToAnalyze, double stdDeviation)
{
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(sourceCloud);
	sor.setMeanK(neighborsToAnalyze);
	sor.setStddevMulThresh(stdDeviation);
	sor.filter(*filteredCloud);
}


///////////////////////////////////////////////////////////////////////////////
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
int c44::getPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud,  
                  pcl::PointCloud<pcl::PointXYZ>::Ptr inliersCloud,                 
                  pcl::PointCloud<pcl::PointXYZ>::Ptr outliersCloud,              
                  pcl::ModelCoefficients::Ptr coefficients,                       
                  pcl::PointIndices::Ptr plane_indices,	
                  double minRadius, double maxRadius,				
                  int ransacIterations, double thresholdDistance)
{

	// check that data in cloud is valid, since errors keep occuring
	if(NULL == sourceCloud)
	{
		return -2;
	}
	#ifdef DEBUG
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ> ("cloud_before_plane_segmentation.pcd", *sourceCloud, false);
	#endif

	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;

	// Optional
	seg.setOptimizeCoefficients(true);

	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(ransacIterations);
	seg.setDistanceThreshold(thresholdDistance);
	seg.setRadiusLimits(minRadius, maxRadius);  // TODO: may need to use different function

	// Segment the largest planar component from the remaining cloud
	seg.setInputCloud(sourceCloud);
	seg.segment(*plane_indices, *coefficients);

	// check if a plane was found	
	if (plane_indices->indices.size() == 0)
	{	// No plane was found
		return -1;
	}
	
	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZ> extract;

	// Extract the inliers
	extract.setInputCloud(sourceCloud);
	extract.setIndices(plane_indices);
	extract.setNegative(false);
	extract.filter(*inliersCloud);

	#ifdef DEBUG_PLANE
	std::cerr << "PointCloud representing the planar component: "
	          << inliersCloud->width * inliersCloud->height << " data points."
	          << std::endl;
	#endif

	// Create the filtering object
	extract.setNegative(true);
	extract.filter(*outliersCloud);
	return 0;
}


///////////////////////////////////////////////////////////////////////////////
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
int c44::getPrism(pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud,
                  pcl::PointCloud<pcl::PointXYZ>::Ptr objectsCloud,                                       	
                  double minRadius, double maxRadius,				
                  int ransacIterations, double thresholdDistance,
                  double minObjectDistance, double maxObjectDistance)
{
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr plane_indices (new pcl::PointIndices);
	pcl::PointCloud<pcl::PointXYZ>::Ptr inliersCloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCDWriter writer;

	// find plane parameters
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(ransacIterations);
	seg.setDistanceThreshold(thresholdDistance);
	seg.setRadiusLimits(minRadius, maxRadius);  // TODO: may need to use different function
	seg.setInputCloud(sourceCloud);
	seg.segment(*plane_indices, *coefficients);

	// check if a plane was found	
	if (plane_indices->indices.size() == 0)
	{	// No plane was found
		return -1;
	}
	else
	{
		// Create the filtering object
		pcl::ExtractIndices<pcl::PointXYZ> extract;

		// Extract the inliers
		extract.setInputCloud(sourceCloud);
		extract.setIndices(plane_indices);
		extract.filter(*inliersCloud);

		// retrieve the convex hull
		pcl::PointCloud<pcl::PointXYZ>::Ptr convexHull (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::ConvexHull<pcl::PointXYZ> chull;
		chull.setInputCloud (inliersCloud);
		chull.setDimension(2);
		chull.reconstruct(*convexHull);

		// redundant check
		if (chull.getDimension() == 2)
		{
			#ifdef DEBUG
			writer.write<pcl::PointXYZ> ("cloud_convex_hull.pcd", *convexHull, false);
			#endif

			// segment those points that are in the polygonal prism
			pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;
			prism.setInputCloud(sourceCloud);
			prism.setInputPlanarHull(convexHull);
			prism.setHeightLimits(minObjectDistance, maxObjectDistance);
			pcl::PointIndices::Ptr objectIndices (new pcl::PointIndices);
			prism.segment (*objectIndices);
	
			// get all points retrieved by the hull
			extract.setIndices(objectIndices);
			extract.filter(*objectsCloud);

			// check if any objects were found
			if(0 == objectsCloud->points.size())
			{	// no objects found
				return -1;
			}
		}
		else
		{	// the chosen hull is not planar
			return -1;
		}
	}
	return 0;
}


///////////////////////////////////////////////////////////////////////////////
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
 * dependencies: c44::voxelFilter(), c44::removeNoise(), c44::getPrism()
 * pre-cond: source cloud must contain a non-empty PCD file. Both sourceCloud and objectsCloud
 *           must have objects initialized before being passed as parameters.
 * post-cond: filtered data is passed back through objectsCloud
 * ret: -1 if a plane could not be found else
 *      -2 if source PCD is empty
 *       0 if succesfull
 */
int c44::segObjects(pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr objectsCloud,
                    float leafSize, int neighborsToAnalyze, double stdDeviation,
                    double minRadius, double maxRadius,
                    int ransacIterations, double thresholdDistance,
                    double minObjectDistance, double maxObjectDistance)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_objects (new pcl::PointCloud<pcl::PointXYZ>);
	int result = 0;         // catch return from function calls

	// downsample source PCD image
	c44::voxelFilter(sourceCloud, cloud_filtered, leafSize);  // call voxel filter
	
	// find objects on top of plane
	result = c44::getPrism(cloud_filtered, cloud_objects, minRadius, maxRadius, ransacIterations, 
	                       thresholdDistance, minObjectDistance, maxObjectDistance);
						   
	// check for errors from getPrism() 
	if(result < 0)  
	{
		return -1;
	}
	
	// remove noise from objects PCD using gaussian filter
	c44::removeNoise(cloud_objects, objectsCloud, neighborsToAnalyze, stdDeviation);
	
	return 0;
}


///////////////////////////////////////////////////////////////////////////////
/**
 * desc: This function uses a "Euclidean Cluster Extraction" algorithm to pair
 *       separate data in a Point Cloud into separate objects.
 * param: (in) sourceCloud - ptr to input cloud
 *        (in) cluster_indicies - ojects that holds each cluster as an element of
 *                                a vector. cluster_indices[0] is cluster 1, and
 *                                cluster_indices[1] is cluster 2, and so on.
 *        (in) clusterTolerance - The maximum distance between points in the
 *                                same cluster (e.g. 0.02 = 2cm)
 *        (in) minClusterSize - minimum number of points in a single cluster 
 *        (in) maxClusterSize - minimum number of points in a single cluster
 */
void c44::clusterExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud,
                            std::vector<pcl::PointIndices>& cluster_indices,
                            double clusterTolerance, int minClusterSize, 
                            int maxClusterSize)
{
	// create the kdTree object for the search method of extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (sourceCloud);
	pcl::PCDWriter writer;
	
	// store each cluster as a vector of indices
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (clusterTolerance);
	ec.setMinClusterSize (minClusterSize);
	ec.setMaxClusterSize (maxClusterSize);
	ec.setSearchMethod (tree);
	ec.setInputCloud (sourceCloud);
	ec.extract (cluster_indices);

	// move each cluster into a separate PCD file

	#ifdef DEBUG
	// TODO: fix passing cluster_indices through parameter. Right now passing out cluster_indices and then running the loop
	//       below results in PCD files that, while not empty, display nothing
	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
		{ 
			cloud_cluster->points.push_back (sourceCloud->points[*pit]); //*
		}
		
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
		
		// write out cluster as a PCD file
		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
		std::stringstream ss;
		ss << "../pictures/cloud_cluster_" << j << ".pcd";
		writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
		j++;
	}	
	#endif
}
