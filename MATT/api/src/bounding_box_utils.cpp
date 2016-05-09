//
//  Geometries.cpp
//  PCLTestbench
//
//  Created on 2/10/16.
//

#include <bounding_box_utils.h>
#include <pcl/features/crh.h>
#include <pcl/recognition/crh_alignment.h>

#define TARGET_OBJ_DIAMETER 4.0
using namespace c44;
using namespace search;

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



float BoundingBox::accuracyWRT(const BoundingBox& rhs) const{
  auto lhsCorners = this->getCorners();
  auto rhsCorners = rhs.getCorners();

  std::vector<int> cornersToCheck = {0,1,2,3,4,5,6,7};
  std::vector<int> possibleMatches = cornersToCheck;
  float cumulativeAccuracy = 1.0;
  for (auto i : cornersToCheck){
    float shortestDistance = FLT_MAX;
    int bestMatchingCorner = -1;
    for (auto j : possibleMatches){
      float distance = sqrtf(powf(lhsCorners[i].x - rhsCorners[j].x,2) +
                             powf(lhsCorners[i].y - rhsCorners[j].y,2) +
                             powf(lhsCorners[i].z - rhsCorners[j].z,2));
      if (distance < shortestDistance){
        shortestDistance = distance;
        bestMatchingCorner = j;
      }
    }
    
    
    
    auto it = std::find(possibleMatches.begin(),
                        possibleMatches.end(),
                        bestMatchingCorner);
    if(it != possibleMatches.end())
      possibleMatches.erase(it);
    
    cumulativeAccuracy *= powf(2,-4.0*shortestDistance);
  }
  return powf(cumulativeAccuracy,1.0/8.0);
}



std::vector<PointXYZ> BoundingBox::getCorners() const{
  std::vector<Vector3f> _corners;
  std::vector<PointXYZ> corners;
  _corners.push_back(Vector3f(min_point_OBB.x,
                             min_point_OBB.y,
                             min_point_OBB.z));
  _corners.push_back(Vector3f(max_point_OBB.x,
                             min_point_OBB.y,
                             min_point_OBB.z));
  _corners.push_back(Vector3f(max_point_OBB.x,
                             max_point_OBB.y,
                             min_point_OBB.z));
  _corners.push_back(Vector3f(min_point_OBB.x,
                             max_point_OBB.y,
                             min_point_OBB.z));
  _corners.push_back(Vector3f(min_point_OBB.x,
                             min_point_OBB.y,
                             max_point_OBB.z));
  _corners.push_back(Vector3f(max_point_OBB.x,
                             min_point_OBB.y,
                             max_point_OBB.z));
  _corners.push_back(Vector3f(max_point_OBB.x,
                             max_point_OBB.y,
                             max_point_OBB.z));
  _corners.push_back(Vector3f(min_point_OBB.x,
                             max_point_OBB.y,
                             max_point_OBB.z));
  
  Eigen::Translation3f translation(centroid.x(),
                                   centroid.y(),
                                   centroid.z());

  for (auto corner : _corners){
    auto transformedCorner = translation * rotational_matrix_OBB * corner;
    corners.push_back(PointXYZ(transformedCorner.x(),
                               transformedCorner.y(),
                               transformedCorner.z()));
  }
  return corners;
}








