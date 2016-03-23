//
//  Geometries.hpp
//  PCLTestbench
//
//  Created on 2/10/16.
//

#ifndef Geometries_hpp
#define Geometries_hpp
#include <stdio.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include <stdio.h>

namespace c44{
  using namespace pcl;
  using namespace Eigen;
  typedef PointCloud<PointXYZ> Cloud3D;

  struct BoundingBox{
  public:
    BoundingBox(Cloud3D::Ptr cloud);
      
    std::vector<float> moment_of_inertia;
    std::vector<float> eccentricity;
    PointXYZ min_point_AABB;
    PointXYZ max_point_AABB;
    PointXYZ min_point_OBB;
    PointXYZ max_point_OBB;
    PointXYZ position_OBB;
    Matrix3f rotational_matrix_OBB;
    float major_value, middle_value, minor_value;
    Vector3f major_vector, middle_vector, minor_vector;
    Vector3f centroid;
    
    //returns the accuracy of this bounding box with respect to `other`
    float accuracyWRT(const BoundingBox& other) const;
  };


  struct GeometricObject{
  public:
      const ModelCoefficients coefficients;
      const Cloud3D::Ptr pointCloud;
      const PointCloud<Normal>::Ptr normalCloud;
    
      GeometricObject(ModelCoefficients mc,
                      Cloud3D::Ptr cloud,
                      PointCloud<Normal>::Ptr normals) :
      coefficients(mc), pointCloud(cloud), normalCloud(normals) {};
      
  };

  struct Plane : public GeometricObject{
  public:
      const PointIndices::Ptr inliers;
      Plane(ModelCoefficients mc,
            Cloud3D::Ptr points,
            PointCloud<Normal>::Ptr normals,
            PointIndices::Ptr _inliers) :
      GeometricObject(mc,points,normals), inliers(_inliers){}
      
  };



  struct GraspableObject : GeometricObject{
  public:
      
      GraspableObject(ModelCoefficients mc,
                      Cloud3D::Ptr points,
                      PointCloud<Normal>::Ptr normals) :
      GeometricObject(mc,points,normals){}
      
      BoundingBox getBoundingBox() const;
      
      
  };
  
  template<int rows, int cols>
  float spread(const Eigen::Matrix<float, rows, cols>& lhs,
               const Eigen::Matrix<float, rows, cols>& rhs)
  {
    float lhsQuadrance, rhsQuadrance, dotProd;
    lhsQuadrance = rhsQuadrance = dotProd = 0.0;
    for (unsigned i = 0; i < rows; i++){
      for (unsigned j = 0; j < cols; j++){
        lhsQuadrance += lhs(i,j)*lhs(i,j);
        rhsQuadrance += rhs(i,j)*rhs(i,j);
        dotProd += lhs(i,j)*rhs(i,j);
      }
    }
    auto numerator = dotProd * dotProd;
    auto denominator = lhsQuadrance * rhsQuadrance;
    return 1 - numerator/denominator;
  }
  

  float spread(const Eigen::Quaternion<float>& lhs,
               const Eigen::Quaternion<float>& rhs);
  
}
#endif /* Geometries_hpp */
