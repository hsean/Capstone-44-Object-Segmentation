//
//  Geometries.cpp
//  PCLTestbench
//
//  Created on 2/10/16.
//

#include "geometries.hpp"
using namespace c44;

BoundingBox::BoundingBox(Cloud3D::Ptr cloud){
  MomentOfInertiaEstimation<PointXYZ> feature_extractor;
  feature_extractor.setInputCloud (cloud);
  feature_extractor.compute();
  feature_extractor.getMomentOfInertia (moment_of_inertia);
  feature_extractor.getEccentricity (eccentricity);
  feature_extractor.getAABB (min_point_AABB, max_point_AABB);
  feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
  feature_extractor.getEigenValues (major_value, middle_value, minor_value);
  feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
  feature_extractor.getMassCenter (centroid);
}

float BoundingBox::operator -(const BoundingBox& rhs) const{
  
  //jump through a few hoops to build up to a 4x4 translation matrix...
  Translation3f __translation(
                              centroid.x() - rhs.centroid.x(),
                              centroid.y() - rhs.centroid.y(),
                              centroid.z() - rhs.centroid.z());
  
  Affine3f _translation(__translation);
  
  
  
  
  _translation *= Scaling(
                         (rhs.max_point_OBB.x - rhs.min_point_OBB.x)/(max_point_OBB.x - min_point_OBB.x),
                         (rhs.max_point_OBB.y - rhs.min_point_OBB.y)/(max_point_OBB.y - min_point_OBB.y),
                         (rhs.max_point_OBB.z - rhs.min_point_OBB.z)/(max_point_OBB.z - min_point_OBB.z));
  Matrix4f translation = _translation.matrix();
  //calculate the inverse of `this` bounding box's rotation matrix,
  // which we can then take and left multiply the other matrix by;
  // in other words, we want to find the matrix X in
  //  X * this->rotational_matrix_OBB = rhs.rotational_matrix_OBB
  // so,
  //  X = rhs.rotational_matrix_OBB * this->rotational_matrix_OBB.invers()
  
  Matrix3f invTransform = rotational_matrix_OBB.inverse();
  Matrix3f _rotateLHStoRHS = rhs.rotational_matrix_OBB * invTransform;
  Eigen::Matrix4f rotateLHStoRHS = Eigen::Matrix4f::Identity();
  rotateLHStoRHS.block(0,0,3,3) = _rotateLHStoRHS;
  
  auto det1 = (rotateLHStoRHS * translation).determinant();
  return 1.0 - det1;
}


BoundingBox GraspableObject::getBoundingBox() const{
  return BoundingBox(this->pointCloud);
}