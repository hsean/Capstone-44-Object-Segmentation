//
//  Geometries.cpp
//  PCLTestbench
//
//  Created on 2/10/16.
//

#include "bounding_box_utils.hpp"
#define TARGET_OBJ_DIAMETER 4.0
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

AccuracyReport BoundingBox::accuracyWRT(const BoundingBox& rhs) const{
  
  Vector3f translation(centroid.x(),centroid.y(),centroid.z());
  Vector3f rhsTranslation(rhs.centroid.x(),rhs.centroid.y(),rhs.centroid.z());
  
  
  auto distance = sqrtf(powf(position_OBB.x - rhs.position_OBB.x,2) +
                        powf(position_OBB.y - rhs.position_OBB.y,2) +
                        powf(position_OBB.z - rhs.position_OBB.z,2));

  //float translationAccuracy = 1 - spread(translation,rhsTranslation);
   float translationAccuracy = powf(2,-2.0*distance);
  auto bestFitRHSRotor = Quaternion<float>(rhs.rotational_matrix_OBB).
                                          normalized();
  auto lhsRotor = Quaternion<float>(rotational_matrix_OBB).normalized();
//  float orientationAccuracy = 1 - spread(lhsRotor.normalized(),
//                                         bestFitRHSRotor.normalized());
  Quaternionf reversedRHSRotor = Quaternionf(-bestFitRHSRotor.x(),
                                             -bestFitRHSRotor.y(),
                                             -bestFitRHSRotor.z(),
                                             -bestFitRHSRotor.w());
  float orientationAccuracy = fmaxf(powf(lhsRotor.dot(bestFitRHSRotor),2.0),
                                    powf(lhsRotor.dot(reversedRHSRotor),2.0));
/*
  for (unsigned i = 1; i < 7; i++){
    //auto shift = sizeof(float);
    auto rhsRotationMatrix = rhs.rotational_matrix_OBB;
    for (int signBitPosn = 0; signBitPosn < 3; signBitPosn++){
      int signBit = (i >> signBitPosn) & 0x1;
      if (signBit){
        for (int j = 0; j < 3; j++){
          rhsRotationMatrix(j,signBitPosn) *= -1.0;
        }
      }
    }
    auto rhsRotor = Quaternion<float>(rhsRotationMatrix).normalized();
    
    float _orientationAccuracy = 1 - spread(lhsRotor.normalized(),
                                            rhsRotor.normalized());
    if (_orientationAccuracy > orientationAccuracy){
      orientationAccuracy = _orientationAccuracy;
      bestFitRHSRotor = rhsRotor;
    }
                                            
  }
*/
  
  auto unshiftedMinPoint = Vector3f(min_point_OBB.x - centroid.x(),
                                    min_point_OBB.y - centroid.y(),
                                    min_point_OBB.z - centroid.z());
  auto unshiftedMaxPoint = Vector3f(max_point_OBB.x - centroid.x(),
                                    max_point_OBB.y - centroid.y(),
                                    max_point_OBB.z - centroid.z());
  
  auto rhs_unshiftedMinPoint = Vector3f(rhs.min_point_OBB.x - rhs.centroid.x(),
                                        rhs.min_point_OBB.y - rhs.centroid.y(),
                                        rhs.min_point_OBB.z - rhs.centroid.z());
  
  auto rhs_unshiftedMaxPoint = Vector3f(rhs.max_point_OBB.x - rhs.centroid.x(),
                                        rhs.max_point_OBB.y - rhs.centroid.y(),
                                        rhs.max_point_OBB.z - rhs.centroid.z());
  
  Quaternion<float> invRotation = lhsRotor.inverse();
  Quaternion<float> rhsInvRotation = bestFitRHSRotor.inverse();
  
  unshiftedMinPoint = invRotation * unshiftedMinPoint;
  unshiftedMaxPoint = invRotation * unshiftedMaxPoint;
  
  rhs_unshiftedMinPoint = rhsInvRotation * rhs_unshiftedMinPoint;
  rhs_unshiftedMaxPoint = rhsInvRotation * rhs_unshiftedMaxPoint;

  Vector3f scaleComponents(unshiftedMaxPoint.x() - unshiftedMinPoint.x(),
                           unshiftedMaxPoint.y() - unshiftedMinPoint.y(),
                           unshiftedMaxPoint.z() - unshiftedMinPoint.z());
  
  Vector3f rhsScaleComponents(rhs_unshiftedMaxPoint.x() -
                              rhs_unshiftedMinPoint.x(),
                              rhs_unshiftedMaxPoint.y() -
                              rhs_unshiftedMinPoint.y(),
                              rhs_unshiftedMaxPoint.z() -
                              rhs_unshiftedMinPoint.z());
  
  
  auto scaleAccuracy = 1 - spread(scaleComponents, rhsScaleComponents);
  
  AccuracyReport ret = {
    translationAccuracy,
    orientationAccuracy,
    scaleAccuracy
  };
  return ret;
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




BoundingBox GraspableObject::getBoundingBox() const{
  return BoundingBox(this->pointCloud);
}


float c44::spread(const Eigen::Quaternion<float>& lhs,
                  const Eigen::Quaternion<float>& rhs)
{
  float lhsQuadrance, rhsQuadrance, dotProd;
    lhsQuadrance = lhs.x()*lhs.x() + lhs.y()*lhs.y() + lhs.z()*lhs.z() +
  lhs.w()*lhs.w();
  rhsQuadrance = rhs.x()*rhs.x() + rhs.y()*rhs.y() + rhs.z()*rhs.z() +
  rhs.w()*rhs.w();
  dotProd      = lhs.x()*rhs.x() + lhs.y()*rhs.y() + lhs.z()*rhs.z() +
  lhs.w()*rhs.w();
  
  
  auto numerator = dotProd * dotProd;
  auto denominator = lhsQuadrance * rhsQuadrance;
  return 1 - numerator/denominator;
  
  
}



