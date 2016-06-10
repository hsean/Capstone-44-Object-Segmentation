//
//  bounding_box_utils.cpp
//
//  Created on 2/10/16.
//  significant revision 5/29/2016
//

#include <bounding_box_utils.h>
#include <pcl/features/crh.h>
#include <pcl/recognition/crh_alignment.h>

#define TARGET_OBJ_DIAMETER 4.0
using namespace c44;
using namespace search;

void BoundingBox::computeMeanValue(){
  
  mean_value_ (0) = 0.0f;
  mean_value_ (1) = 0.0f;
  mean_value_ (2) = 0.0f;
  
  
  unsigned int number_of_points = static_cast <unsigned int> (indices_->size ());
  for (unsigned int i_point = 0; i_point < number_of_points; i_point++)
  {
    mean_value_ (0) += input_->points[(*indices_)[i_point]].x;
    mean_value_ (1) += input_->points[(*indices_)[i_point]].y;
    mean_value_ (2) += input_->points[(*indices_)[i_point]].z;
    
  }
  
  if (number_of_points == 0)
    number_of_points = 1;
  
  mean_value_ (0) /= number_of_points;
  mean_value_ (1) /= number_of_points;
  mean_value_ (2) /= number_of_points;
  
}

BoundingBox::BoundingBox(const Cloud3D::Ptr cloud)
{

  auto startTime = boost::posix_time::microsec_clock::local_time();
  setInputCloud(cloud);
  
  if (!initCompute ())
  {
    deinitCompute ();
    throw "could not initialize compute";
  }

  
  computeMeanValue ();
  
  Eigen::Matrix <float, 3, 3> covariance_matrix;
  covariance_matrix.setZero ();
  computeCovarianceMatrix (covariance_matrix);
  
  computeEigenVectors (covariance_matrix, major_axis_, middle_axis_, minor_axis_, major_value_, middle_value_, minor_value_);
  
  

  computeOBB ();
  

  deinitCompute ();

  //the old way
//  pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
//  feature_extractor.setInputCloud (cloud);
//  feature_extractor.compute ();
//  feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
//  feature_extractor.getMassCenter (centroid);
  auto runtime = boost::posix_time::microsec_clock::local_time() - startTime;
  std::cout << "bbox calculation time = " << runtime << std::endl;

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

void BoundingBox::computeEigenVectors (const Eigen::Matrix <float, 3, 3>& covariance_matrix,
                                       Eigen::Vector3f& major_axis,
                                       Eigen::Vector3f& middle_axis,
                                       Eigen::Vector3f& minor_axis,
                                       float& major_value,
                                       float& middle_value,
                                       float& minor_value)
{
  Eigen::EigenSolver <Eigen::Matrix <float, 3, 3> > eigen_solver;
  eigen_solver.compute (covariance_matrix);
  
  Eigen::EigenSolver <Eigen::Matrix <float, 3, 3> >::EigenvectorsType eigen_vectors;
  Eigen::EigenSolver <Eigen::Matrix <float, 3, 3> >::EigenvalueType eigen_values;
  eigen_vectors = eigen_solver.eigenvectors ();
  eigen_values = eigen_solver.eigenvalues ();
  
  unsigned int temp = 0;
  unsigned int major_index = 0;
  unsigned int middle_index = 1;
  unsigned int minor_index = 2;
  
  if (eigen_values.real () (major_index) < eigen_values.real () (middle_index))
  {
    temp = major_index;
    major_index = middle_index;
    middle_index = temp;
  }
  
  if (eigen_values.real () (major_index) < eigen_values.real () (minor_index))
  {
    temp = major_index;
    major_index = minor_index;
    minor_index = temp;
  }
  
  if (eigen_values.real () (middle_index) < eigen_values.real () (minor_index))
  {
    temp = minor_index;
    minor_index = middle_index;
    middle_index = temp;
  }
  
  major_value = eigen_values.real () (major_index);
  middle_value = eigen_values.real () (middle_index);
  minor_value = eigen_values.real () (minor_index);
  
  major_axis = eigen_vectors.col (major_index).real ();
  middle_axis = eigen_vectors.col (middle_index).real ();
  minor_axis = eigen_vectors.col (minor_index).real ();
  
  major_axis.normalize ();
  middle_axis.normalize ();
  minor_axis.normalize ();
  
  float det = major_axis.dot (middle_axis.cross (minor_axis));
  if (det <= 0.0f)
  {
    major_axis (0) = -major_axis (0);
    major_axis (1) = -major_axis (1);
    major_axis (2) = -major_axis (2);
  }
}



void BoundingBox::computeCovarianceMatrix (Eigen::Matrix <float, 3, 3>& covariance_matrix) const
{
  covariance_matrix.setZero ();
  
  unsigned int number_of_points = static_cast <unsigned int> (indices_->size ());
  float factor = 1.0f / static_cast <float> ((number_of_points - 1 > 0)?(number_of_points - 1):1);
  for (unsigned int i_point = 0; i_point < number_of_points; i_point++)
  {
    Eigen::Vector3f current_point (0.0f, 0.0f, 0.0f);
    current_point (0) = input_->points[(*indices_)[i_point]].x - mean_value_ (0);
    current_point (1) = input_->points[(*indices_)[i_point]].y - mean_value_ (1);
    current_point (2) = input_->points[(*indices_)[i_point]].z - mean_value_ (2);
    
    covariance_matrix += current_point * current_point.transpose ();
  }
  
  covariance_matrix *= factor;
}




void BoundingBox::computeOBB ()
{
  min_point_OBB.x = std::numeric_limits <float>::max ();
  min_point_OBB.y = std::numeric_limits <float>::max ();
  min_point_OBB.z = std::numeric_limits <float>::max ();
  
  max_point_OBB.x = std::numeric_limits <float>::min ();
  max_point_OBB.y = std::numeric_limits <float>::min ();
  max_point_OBB.z = std::numeric_limits <float>::min ();
  
  rotational_matrix_OBB << major_axis_ (0), middle_axis_ (0), minor_axis_ (0),
  major_axis_ (1), middle_axis_ (1), minor_axis_ (1),
  major_axis_ (2), middle_axis_ (2), minor_axis_ (2);

  
  unsigned int number_of_points = static_cast <unsigned int> (indices_->size ());
  
  for (unsigned int i_point = 0; i_point < number_of_points; i_point++)
  {
    
//    auto p = Eigen::Vector3f(input_->points[(*indices_)[i_point]].x,
//                             input_->points[(*indices_)[i_point]].y,
//                             input_->points[(*indices_)[i_point]].z);
//    auto v = obb_rotational_matrix_.transpose() * (p - mean_value_);

    float x = (input_->points[(*indices_)[i_point]].x - mean_value_ (0)) * major_axis_ (0) +
    (input_->points[(*indices_)[i_point]].y - mean_value_ (1)) * major_axis_ (1) +
    (input_->points[(*indices_)[i_point]].z - mean_value_ (2)) * major_axis_ (2);
    float y = (input_->points[(*indices_)[i_point]].x - mean_value_ (0)) * middle_axis_ (0) +
    (input_->points[(*indices_)[i_point]].y - mean_value_ (1)) * middle_axis_ (1) +
    (input_->points[(*indices_)[i_point]].z - mean_value_ (2)) * middle_axis_ (2);
    float z = (input_->points[(*indices_)[i_point]].x - mean_value_ (0)) * minor_axis_ (0) +
    (input_->points[(*indices_)[i_point]].y - mean_value_ (1)) * minor_axis_ (1) +
    (input_->points[(*indices_)[i_point]].z - mean_value_ (2)) * minor_axis_ (2);
    
    
    if (x <= min_point_OBB.x) min_point_OBB.x = x;
    if (y <= min_point_OBB.y) min_point_OBB.y = y;
    if (z <= min_point_OBB.z) min_point_OBB.z = z;
    
    if (x >= max_point_OBB.x) max_point_OBB.x = x;
    if (y >= max_point_OBB.y) max_point_OBB.y = y;
    if (z >= max_point_OBB.z) max_point_OBB.z = z;
    
    
  }
  
  
  Eigen::Vector3f shift ((max_point_OBB.x + min_point_OBB.x) / 2.0f,
                         (max_point_OBB.y + min_point_OBB.y) / 2.0f,
                         (max_point_OBB.z + min_point_OBB.z) / 2.0f);
  
  min_point_OBB.x -= shift (0);
  min_point_OBB.y -= shift (1);
  min_point_OBB.z -= shift (2);
  
  max_point_OBB.x -= shift (0);
  max_point_OBB.y -= shift (1);
  max_point_OBB.z -= shift (2);
  //Vector3f position;
  centroid = mean_value_ + rotational_matrix_OBB * shift;
  position_OBB = PointXYZ(centroid.x(),centroid.y(),centroid.z());
}





