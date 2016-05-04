//
//  c44_geometry.cpp
//  PCLTestbench
//
//  Created on 2/10/16.
//  Edited on 4/22/16.

#include "c44_geometry.hpp"
#define TARGET_OBJ_DIAMETER 4.0
using namespace c44Geometry;

BoundingBox::BoundingBox(Cloud3D::Ptr cloud){
  MomentOfInertiaEstimation<PointXYZRGB> feature_extractor;
  feature_extractor.setInputCloud (cloud);
  feature_extractor.compute();
  feature_extractor.getMomentOfInertia (moment_of_inertia);
  feature_extractor.getEccentricity (eccentricity);
  feature_extractor.getAABB (min_point_AABB, max_point_AABB);
  feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
  feature_extractor.getEigenValues (major_value, middle_value, minor_value);
  feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
  feature_extractor.getMassCenter (centroid);
  width = max_point_OBB.x - min_point_OBB.x;
  height = max_point_OBB.y - min_point_OBB.y;
  depth = max_point_OBB.z - min_point_OBB.z;
}


float BoundingBox::accuracyWRT(const BoundingBox& rhs) const{
    std::vector<PointXYZRGB> lhsCorners = this->getCorners();
    std::vector<PointXYZRGB> rhsCorners = rhs.getCorners();

    // construct vector with corner indexes
    int corners_list[] = {0,1,2,3,4,5,6,7};
    std::vector<int> corners_to_check (corners_list, corners_list + sizeof(corners_list) / sizeof(int));

    std::vector<int> possible_matches = corners_to_check;
    float cumulative_accuracy = 1.0;
    int ctc_size = corners_to_check.size();
    for(int i = 0; i < ctc_size; i++)                  //for (it = cornersToCheck.begin(); it != cornersToCheck.end(); ++it)
    {
        float shortest_distance = FLT_MAX;
        int best_matching_corner = -1;
        for(int j = 0; j < ctc_size; j++)//for (std::vector<int>::iterator jt = possibleMatches.begin(); jt != possibleMatches.end(); ++jt)
        {

            float distance = sqrtf(powf(lhsCorners[i].x - rhsCorners[j].x,2) +
                                 powf(lhsCorners[i].y - rhsCorners[j].y,2) +
                                 powf(lhsCorners[i].z - rhsCorners[j].z,2));

            if (distance < shortest_distance)
            {
                shortest_distance = distance;
                best_matching_corner = j;
            }
        }

        std::vector<int>::iterator it = std::find(possible_matches.begin(),
                                                    possible_matches.end(),
                                                    best_matching_corner);
        if(it != possible_matches.end())
            possible_matches.erase(it);

        cumulative_accuracy *= powf(2,-4.0*shortest_distance);

    }

    return powf(cumulative_accuracy,1.0/8.0);
}



std::vector<PointXYZRGB> BoundingBox::getCorners() const{
  std::vector<Vector3f> _corners;
  std::vector<PointXYZRGB> corners;
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

  for (int i = 0; i < corners.size(); i++){
    Vector3f transformed_corner = translation * rotational_matrix_OBB * _corners[i];
    corners.push_back(PointXYZRGB(transformed_corner.x(),
                               transformed_corner.y(),
                               transformed_corner.z()));
  }
  return corners;
}




BoundingBox GraspableObject::getBoundingBox() const{
  return BoundingBox(this->point_cloud);
}
