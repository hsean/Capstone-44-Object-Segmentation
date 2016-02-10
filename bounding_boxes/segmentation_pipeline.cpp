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
	filteredCloud(new Cloud3D),
	cloud_normals(new PointCloud<Normal>)
{
	
	pcl::PassThrough<pcl::PointXYZ> pass;
	
	pass.setInputCloud(rawCloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0, 1.5);
	
	pass.filter(*filteredCloud);
	
	std::cerr << "PointCloud after filtering has: " <<
	filteredCloud->points.size ()      <<
	" data points."                    <<
	std::endl;
	
	NormalEstimation<PointXYZ, Normal> ne;
	
	
	ne.setSearchMethod(tree);
	ne.setInputCloud(filteredCloud);
	ne.setKSearch(50);
	ne.compute(*cloud_normals);


}



Plane SegmentationPipeline::getPlane()
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
	seg.setInputCloud (filteredCloud);
	seg.setInputNormals (cloud_normals);
	// Obtain the plane inliers and coefficients
	seg.segment (*inliers, *coefficients_plane);
	std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

	// Extract the planar inliers from the input cloud
	extract.setInputCloud (filteredCloud);
	extract.setIndices (inliers);
	extract.setNegative (false);

	// Write the planar inliers to disk
	pcl::PointCloud<PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
	extract.filter (*cloud_plane);
	std::cerr << "PointCloud representing the planar component: " <<
							 cloud_plane->points.size () <<
							 " data points." <<
							 std::endl;
	
	

	
	Plane ret = Plane(*coefficients_plane,cloud_plane,cloud_normals,inliers);
	
	return ret;

}

GraspableObject SegmentationPipeline::getObject(SacModel model)
{
	ModelCoefficients::Ptr objectCoefficients(new ModelCoefficients);
	ExtractIndices<PointXYZ> extract;
	ExtractIndices<Normal> extract_normals;
	PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);
	PointIndices::Ptr inliers(new PointIndices);
	
	// Extract the planar inliers from the input cloud
	
	Plane plane = getPlane();
	extract.setInputCloud (filteredCloud);
	extract.setIndices (plane.inliers);
	
	Cloud3D::Ptr cloudMinusPlane(new Cloud3D);
	
	// Remove the planar inliers, extract the rest
	extract.setNegative (true);
	extract.filter (*cloudMinusPlane);
	extract_normals.setNegative (true);
  extract_normals.setInputCloud (cloud_normals);
 	extract_normals.setIndices(plane.inliers);
	extract_normals.filter (*normals);

	
	// Create the segmentation object for cylinder segmentation and set all the parameters
	seg.setOptimizeCoefficients (true);
	seg.setModelType (model);
	seg.setMethodType (SAC_RANSAC);
	seg.setNormalDistanceWeight (0.1);
	seg.setMaxIterations (10000);
	seg.setDistanceThreshold (0.05);
	seg.setRadiusLimits (0, 0.1);
	seg.setInputCloud (cloudMinusPlane);
	seg.setInputNormals (normals);
	
	// Obtain the cylinder inliers and coefficients
	seg.segment (*inliers, *objectCoefficients);
	std::cerr << "Object coefficients: " << *objectCoefficients << std::endl;
	
	extract.setInputCloud (cloudMinusPlane);
	extract.setIndices (inliers);
	extract.setNegative (false);
	Cloud3D::Ptr cylinderCloud(new Cloud3D);
	extract.filter(*cylinderCloud);
	
	GraspableObject ret(*objectCoefficients,cylinderCloud,normals);
	
	return ret;
}




BoundingBox GraspableObject::getBoundingBox() const{
	MomentOfInertiaEstimation<PointXYZ> feature_extractor;
	feature_extractor.setInputCloud (this->pointCloud);
	feature_extractor.compute();

	std::vector<float> moment_of_inertia;
	std::vector<float> eccentricity;
	pcl::PointXYZ min_point_AABB;
	pcl::PointXYZ max_point_AABB;
	pcl::PointXYZ min_point_OBB;
	pcl::PointXYZ max_point_OBB;
	pcl::PointXYZ position_OBB;
	Eigen::Matrix3f rotational_matrix_OBB;
	float major_value, middle_value, minor_value;
	Vector3f major_vector, middle_vector, minor_vector;
	Vector3f mass_center;

	feature_extractor.getMomentOfInertia (moment_of_inertia);
	feature_extractor.getEccentricity (eccentricity);
	feature_extractor.getAABB (min_point_AABB, max_point_AABB);
	feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
	feature_extractor.getEigenValues (major_value, middle_value, minor_value);
	feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
	feature_extractor.getMassCenter (mass_center);


	BoundingBox ret = {
		moment_of_inertia,
		eccentricity,
		min_point_AABB,
		max_point_AABB,
		min_point_OBB,
		max_point_OBB,
		position_OBB,
		rotational_matrix_OBB,
		major_value,
		middle_value,
		minor_value,
		major_vector, middle_vector, minor_vector,
		mass_center
	};

	return ret;
}