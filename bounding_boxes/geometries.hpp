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
      //from documentation/tutorials/moment_of_inertia.php#moment-of-inertia
      const std::vector<float> moment_of_inertia;
      const std::vector<float> eccentricity;
      const PointXYZ min_point_AABB;
      const PointXYZ max_point_AABB;
      const PointXYZ min_point_OBB;
      const PointXYZ max_point_OBB;
      const PointXYZ position_OBB;
      const Matrix3f rotational_matrix_OBB;
      const float major_value, middle_value, minor_value;
      const Vector3f major_vector, middle_vector, minor_vector;
      const Vector3f centroid;
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
}
#endif /* Geometries_hpp */
