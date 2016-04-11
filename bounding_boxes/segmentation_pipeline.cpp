//
//  segmentation_pipeline.cpp
//  PCLTestbench
//
//  Created on 2/5/16.
//


#include "segmentation_pipeline.hpp"
using namespace c44;
using namespace c44::filters;

SegmentationPipeline::SegmentationPipeline(Cloud3D::Ptr rawCloud,
                                           float voxelSize,
                                           float sampleSize,
                                           float stdDev,
                                           float iterationDivisor) :
	tree(new search::KdTree<PointXYZ>()),
  normals(new PointCloud<Normal>),
  objectCloud(new Cloud3D),
  denoisedCloud(rawCloud),
  iterationDivisor(iterationDivisor),
  convexHull (new Cloud3D),
  planeCloud(new Cloud3D),
  sampleSize(sampleSize),
  stdDev(stdDev)
{
 
	

	
  cutoffZ(denoisedCloud,denoisedCloud);
  


  
  voxelFilter(denoisedCloud, denoisedCloud, voxelSize);
  
  
	NormalEstimation<PointXYZ, Normal> ne;

  std::cerr << "PointCloud after filtering has: " <<
  denoisedCloud->points.size ()      <<
  " data points."                    <<
  std::endl;

	
	ne.setSearchMethod(tree);
	ne.setInputCloud(denoisedCloud);
	ne.setKSearch(25);
	ne.compute(*normals);

  
}

bool SegmentationPipeline::performSegmentation(){
  if (extractPrism()){
    int max = 1;
    bool stillFindingStuff = true;
    //this is to support extracting multiple objects
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
  planeCoefficients = ModelCoefficients::Ptr(new ModelCoefficients);
  PointIndices::Ptr plane_indices(new PointIndices);
  
  
  ExtractIndices<PointXYZ> extractor;
  
  // Create the segmentation object for the planar
  // model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType(SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight (0.1);
  
  seg.setMethodType(SAC_RANSAC);
  int plane_iterations = int(ceil(float(100)/iterationDivisor));
  seg.setMaxIterations(plane_iterations);
  seg.setDistanceThreshold (0.03);
  seg.setInputCloud (denoisedCloud);
  seg.setInputNormals (normals);
  // Obtain the plane inliers and coefficients
  seg.segment(*plane_indices, *planeCoefficients);
  std::cerr << "Plane coefficients: " << *planeCoefficients << std::endl;
  
  // Extract the planar inliers from the input cloud
  extractor.setInputCloud (denoisedCloud);
  extractor.setIndices (plane_indices);

  extractor.filter (*planeCloud);
  return planeCloud->points.size() > 0;
}


bool SegmentationPipeline::extractPrism()
{
  
  if (extractPlane()){
    
    pcl::ConvexHull<pcl::PointXYZ> chull;    
    chull.setInputCloud (planeCloud);
    chull.setDimension(2);
    chull.reconstruct(*convexHull);
    
      
    //segment out those points that are in the polygonal prism
    pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prismExtractor;
    prismExtractor.setInputCloud(denoisedCloud);
    prismExtractor.setInputPlanarHull(convexHull);
    prismExtractor.setHeightLimits(0.02, 0.2);
    pcl::PointIndices::Ptr objectIndices (new pcl::PointIndices);
    prismExtractor.segment (*objectIndices);
    
    
    //remove the convex-hull's points from the main cloud
    ExtractIndices<PointXYZ> extractor;
    extractor.setInputCloud(denoisedCloud);
    extractor.setIndices(objectIndices);
    extractor.filter(*objectCloud);
    
    ExtractIndices<Normal> normalExtractor;
    normalExtractor.setInputCloud (normals);
    normalExtractor.setIndices(objectIndices);
    normalExtractor.filter (*normals);


    return objectCloud->points.size() > 0;
  } else{
    return false;
  }
 
}

bool SegmentationPipeline::extractGraspableObject(SacModel model)
{
	ModelCoefficients::Ptr objectCoefficients(new ModelCoefficients);
	ExtractIndices<PointXYZ> extractor;
	ExtractIndices<Normal> normalExtractor;
	
	PointIndices::Ptr inliers(new PointIndices);
  PointCloud<Normal>::Ptr objectNormals(new PointCloud<Normal>);
  Cloud3D::Ptr cylinderCloud(new Cloud3D);
  Cloud3D::Ptr denoisedObjectCloud(objectCloud);
  
  removeNoise(denoisedObjectCloud,
              denoisedObjectCloud,
              sampleSize, stdDev);
  


  normals.reset(new PointCloud<Normal>);
  
  NormalEstimation<PointXYZ, Normal> ne;
  
  std::cerr << "PointCloud after filtering has: " <<
  denoisedCloud->points.size ()      <<
  " data points."                    <<
  std::endl;
  
  
  ne.setSearchMethod(tree);
  ne.setInputCloud(objectCloud);
  ne.setKSearch(25);
  ne.compute(*normals);
  
  
	// Create the segmentation object for cylinder segmentation and set all the parameters
	seg.setOptimizeCoefficients (true);
	seg.setModelType (model);
  seg.setMethodType (SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  int objIterations = int(ceil(float(10000)/iterationDivisor));
	seg.setMaxIterations(objIterations);
	seg.setDistanceThreshold (0.05);
	seg.setRadiusLimits (0.01, 0.12);
	seg.setInputCloud (objectCloud);
	seg.setInputNormals (normals);
	
	// Obtain the cylinder inliers and coefficients
	seg.segment (*inliers, *objectCoefficients);
	std::cerr << "Object coefficients: " << *objectCoefficients << std::endl;
	
	extractor.setInputCloud (objectCloud);
	extractor.setIndices (inliers);
	extractor.setNegative (false);
  normalExtractor.setIndices(inliers);
	extractor.filter(*cylinderCloud);
  normalExtractor.filter(*objectNormals);
  if (cylinderCloud->size() == 0){
    return false;
  } else {
    //subtract out the object just found from the original input cloud
    extractor.setInputCloud (objectCloud);
    extractor.setNegative (true);
    extractor.setIndices (inliers);
    extractor.filter (*objectCloud);        
    GraspableObject obj(*objectCoefficients,cylinderCloud,objectNormals);
    graspableObjects.push_back(obj);
    return true;
  }
	
  
	
	
}




