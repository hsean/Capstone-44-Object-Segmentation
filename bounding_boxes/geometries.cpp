//
//  Geometries.cpp
//  PCLTestbench
//
//  Created on 2/10/16.
//

#include "geometries.hpp"
using namespace C44;

BoundingBox GraspableObject::getBoundingBox() const{
  MomentOfInertiaEstimation<PointXYZ> feature_extractor;
  feature_extractor.setInputCloud (this->pointCloud);
  feature_extractor.compute();
  
  std::vector<float> moment_of_inertia;
  std::vector<float> eccentricity;
  pcl::PointXYZ min_point_AABB,
                max_point_AABB,
                min_point_OBB,
                max_point_OBB,
                position_OBB;
  
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