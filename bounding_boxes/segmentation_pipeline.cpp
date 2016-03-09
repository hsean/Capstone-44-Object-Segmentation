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
                                           float iterationDivisor,
                                           bool downsample) :
	tree(new search::KdTree<PointXYZ>()),
	cloud_normals(new PointCloud<Normal>),
  normalsMinusObjectsAndPlane(new PointCloud<Normal>),
  cloudMinusObjectsAndPlane(new Cloud3D),
  denoisedCloud(rawCloud),
  iterationDivisor(iterationDivisor),
  convexHull (new Cloud3D)
{
 
	
//	pass.setInputCloud(rawCloud);
//  unsigned nonNaNCount = 0;
//  for (PointXYZ p :  (*rawCloud).points){
//    if (!isnan(p.z)){
//      nonNaNCount++;
//      std::cout << p << std::endl;
//    }
//  }
	
  cutoffZ(denoisedCloud,denoisedCloud);
  //removeStatOutliers(cloudMinusObjectsAndPlane, cloudMinusObjectsAndPlane);
  
  removeNoise(denoisedCloud,
              denoisedCloud,
              sampleSize, stdDev);
  //downsample(cloudMinusObjectsAndPlane, cloudMinusObjectsAndPlane, 0.005);
  if (downsample){
    voxelFilter(denoisedCloud, denoisedCloud, voxelSize);
  }
	NormalEstimation<PointXYZ, Normal> ne;

  std::cerr << "PointCloud after filtering has: " <<
  denoisedCloud->points.size ()      <<
  " data points."                    <<
  std::endl;

	
	ne.setSearchMethod(tree);
	ne.setInputCloud(denoisedCloud);
	ne.setKSearch(50);
	ne.compute(*normalsMinusObjectsAndPlane);

  
}

bool SegmentationPipeline::performSegmentation(){
  if (extractPrism()){
    
    int max = 1;
    bool stillFindingStuff = true;
    do {
      stillFindingStuff = extractGraspableObject(SACMODEL_CYLINDER);
//      if (graspableObjects.size() < max){
//        extractPlane();
//      }
    } while (stillFindingStuff && graspableObjects.size() < max);
    return true;
  } else {
    return false;
  }
}


bool SegmentationPipeline::extractPrism()
{
//  int result = c44::getPrism(originalCloud, cloudMinusObjectsAndPlane,
//                             0, 1.57, 100, 0.01, 0.02, 0.2);
//  return result == 0;

  ModelCoefficients::Ptr coefficients_plane (new ModelCoefficients);
	PointIndices::Ptr plane_indices(new PointIndices);
	
	
	ExtractIndices<PointXYZ> extract;
	
	
	// Estimate point normals
	
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
	seg.setInputNormals (normalsMinusObjectsAndPlane);
	// Obtain the plane inliers and coefficients
	seg.segment (*plane_indices, *coefficients_plane);
	std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

	// Extract the planar inliers from the input cloud
	extract.setInputCloud (denoisedCloud);
	extract.setIndices (plane_indices);
  //extract.setNegative (false);

	pcl::PointCloud<PointXYZ>::Ptr planeCloud(new pcl::PointCloud<pcl::PointXYZ> ());
	extract.filter (*planeCloud);
	std::cerr << "PointCloud representing the planar component: " <<
							 planeCloud->points.size () <<
							 " data points." <<
							 std::endl;
	
  
  // Extract the planar inliers from the input cloud
  if (planeCloud->points.size() > 0){
    //this->plane = new Plane(*coefficients_plane,cloud_plane,cloud_normals,inliers);
    
    
    // Remove the planar inliers, extract the rest
    
    
    
    pcl::ConvexHull<pcl::PointXYZ> chull;
    chull.setInputCloud (planeCloud);
    chull.setDimension(2);
    chull.reconstruct(*convexHull);
    
      
    // segment those points that are in the polygonal prism
    pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;
    prism.setInputCloud(denoisedCloud);
    prism.setInputPlanarHull(convexHull);
    //0, 1.57, 1000, 0.01, 0.02, 0.2
    prism.setHeightLimits(0.02, 0.2);
    pcl::PointIndices::Ptr objectIndices (new pcl::PointIndices);
    prism.segment (*objectIndices);
    
    // get all points retrieved by the hull
    extract.setInputCloud(denoisedCloud);
    extract.setIndices(objectIndices);
    cloudMinusObjectsAndPlane.reset(new Cloud3D);
    extract.filter(*cloudMinusObjectsAndPlane);
    
    ExtractIndices<Normal> extract_normals;
    //extract_normals.setNegative (true);
    extract_normals.setInputCloud (normalsMinusObjectsAndPlane);
    extract_normals.setIndices(objectIndices);
    extract_normals.filter (*normalsMinusObjectsAndPlane);

    
    // check if any objects were found
    if(0 == cloudMinusObjectsAndPlane->points.size())
    {	// no objects found
      return false;
    }
    

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
  Cloud3D::Ptr cylinderCloud(new Cloud3D);
	
  //cloudMinusObjectsAndPlane.reset(&(*cloudMinusObjectsAndPlane));
  
	// Create the segmentation object for cylinder segmentation and set all the parameters
	seg.setOptimizeCoefficients (true);
	seg.setModelType (model);
  seg.setMethodType (SAC_RANSAC);
  //seg.setMethodType(SAC_MLESAC);//terribly slow
  seg.setNormalDistanceWeight (0.1);
  int objIterations = int(ceil(float(100)/iterationDivisor));
	seg.setMaxIterations (objIterations);
	seg.setDistanceThreshold (0.05);
	seg.setRadiusLimits (0.01, 0.12);
	seg.setInputCloud (cloudMinusObjectsAndPlane);
	seg.setInputNormals (normalsMinusObjectsAndPlane);
	
	// Obtain the cylinder inliers and coefficients
	seg.segment (*inliers, *objectCoefficients);
	std::cerr << "Object coefficients: " << *objectCoefficients << std::endl;
	
	extract.setInputCloud (cloudMinusObjectsAndPlane);
	extract.setIndices (inliers);
	extract.setNegative (false);
  extract_normals.setIndices(inliers);
	extract.filter(*cylinderCloud);
  extract_normals.filter(*objectNormals);
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




