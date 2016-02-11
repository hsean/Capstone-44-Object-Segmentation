//
//  segmentation_pipeline.cpp
//  PCLTestbench
//
//  Created on 2/5/16.
//


#include "segmentation_pipeline.hpp"
using namespace C44;

SegmentationPipeline::SegmentationPipeline(Cloud3D::Ptr rawCloud) :
	tree(new search::KdTree<PointXYZ>()),
	noiseFreeCloud(new Cloud3D),
	cloud_normals(new PointCloud<Normal>),
  plane(nullptr),
  normalsMinusObjectsAndPlane(new PointCloud<Normal>),
  cloudMinusObjectsAndPlane(new Cloud3D)
{
 
	pcl::PassThrough<pcl::PointXYZ> pass;
	
	pass.setInputCloud(rawCloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0, 1.5);
	
	pass.filter(*noiseFreeCloud);
	
	std::cerr << "PointCloud after filtering has: " <<
	noiseFreeCloud->points.size ()      <<
	" data points."                    <<
	std::endl;
	
	NormalEstimation<PointXYZ, Normal> ne;
	
	
	ne.setSearchMethod(tree);
	ne.setInputCloud(noiseFreeCloud);
	ne.setKSearch(50);
	ne.compute(*cloud_normals);

  
}

bool SegmentationPipeline::performSegmentation(){
  if (extractPlane()){
    int max = 1;
    bool stillFindingStuff = true;
    do {
      stillFindingStuff = extractGraspableObject(SACMODEL_CYLINDER);
    } while (stillFindingStuff && graspableObjects.size() < max);
    return true;
  } else {
    return false;
  }
}


bool SegmentationPipeline::extractPlane()
{
	ModelCoefficients::Ptr coefficients_plane (new ModelCoefficients);
	PointIndices::Ptr inliers (new PointIndices);
	
	ExtractIndices<Normal> extract_normals;
	ExtractIndices<PointXYZ> extract;
	
	PointCloud<Normal>::Ptr cloud_normals2(new pcl::PointCloud<Normal>);
	
	// Estimate point normals
	
	// Create the segmentation object for the planar
	// model and set all the parameters
	seg.setOptimizeCoefficients (true);
	seg.setModelType(SACMODEL_NORMAL_PLANE);
	seg.setNormalDistanceWeight (0.1);
	seg.setMethodType(SAC_RANSAC);
	seg.setMaxIterations (100);
	seg.setDistanceThreshold (0.03);
	seg.setInputCloud (noiseFreeCloud);
	seg.setInputNormals (cloud_normals);
	// Obtain the plane inliers and coefficients
	seg.segment (*inliers, *coefficients_plane);
	std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

	// Extract the planar inliers from the input cloud
	extract.setInputCloud (noiseFreeCloud);
	extract.setIndices (inliers);
	extract.setNegative (false);

	// Write the planar inliers to disk
	pcl::PointCloud<PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
	extract.filter (*cloud_plane);
	std::cerr << "PointCloud representing the planar component: " <<
							 cloud_plane->points.size () <<
							 " data points." <<
							 std::endl;
	
  
  // Extract the planar inliers from the input cloud
  if (cloud_plane->points.size() > 0){
    this->plane = new Plane(*coefficients_plane,cloud_plane,cloud_normals,inliers);
    extract.setInputCloud (noiseFreeCloud);
    extract.setIndices (plane->inliers);
    
    
    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloudMinusObjectsAndPlane);
    extract_normals.setNegative (true);
    extract_normals.setInputCloud (cloud_normals);
    extract_normals.setIndices(plane->inliers);
    extract_normals.filter (*normalsMinusObjectsAndPlane);
    return true;
  } else{
    return false;
  }
}

bool SegmentationPipeline::extractGraspableObject(SacModel model)
{
	ModelCoefficients::Ptr objectCoefficients(new ModelCoefficients);
	ExtractIndices<PointXYZ> extract;
	ExtractIndices<Normal> extract_normals;
	
	PointIndices::Ptr inliers(new PointIndices);
  PointCloud<Normal>::Ptr objectNormals(new PointCloud<Normal>);
	
	// Create the segmentation object for cylinder segmentation and set all the parameters
	seg.setOptimizeCoefficients (true);
	seg.setModelType (model);
	seg.setMethodType (SAC_RANSAC);
	seg.setNormalDistanceWeight (0.1);
	seg.setMaxIterations (10000);
	seg.setDistanceThreshold (0.05);
	seg.setRadiusLimits (0, 0.1);
	seg.setInputCloud (cloudMinusObjectsAndPlane);
	seg.setInputNormals (normalsMinusObjectsAndPlane);
	
	// Obtain the cylinder inliers and coefficients
	seg.segment (*inliers, *objectCoefficients);
	std::cerr << "Object coefficients: " << *objectCoefficients << std::endl;
	
	extract.setInputCloud (cloudMinusObjectsAndPlane);
	extract.setIndices (inliers);
	extract.setNegative (false);
	Cloud3D::Ptr cylinderCloud(new Cloud3D);
	extract.filter(*cylinderCloud);
  //extract_normals.filter(*objectNormals);
  if (cylinderCloud->size() == 0){
    return false;
  } else {
    //subtract out the object just found from the original input cloud
    extract.setInputCloud (cloudMinusObjectsAndPlane);
    extract.setNegative (true);
    extract.setIndices (inliers);
    extract.filter (*cloudMinusObjectsAndPlane);
    
    extract_normals.setInputCloud (normalsMinusObjectsAndPlane);
    extract_normals.setNegative (true);
    extract_normals.setIndices(inliers);
    extract_normals.filter (*normalsMinusObjectsAndPlane);

    
    GraspableObject obj(*objectCoefficients,cylinderCloud,objectNormals);
    graspableObjects.push_back(obj);
    return true;
  }
	
	
	
	
}


SegmentationPipeline::~SegmentationPipeline(){
  delete plane;
  
}

