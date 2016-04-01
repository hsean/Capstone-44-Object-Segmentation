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

  struct SegmentationConfig{
  public:
    float voxelSize = 0.015;
    float sampleSize = 50;
    float stdDev = 1.0;
    float iterationDivisor = 1.0;
  };

  
  struct AccuracyReport{
  public:
    float translational, orientational, scalar;
  };

  
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
    
    /*------------------------------------------------------------------------*/
    /**
     * desc: computes the accuracy of a bounding box with respect to some other
     *       bounding box which is considered the 'golden model', by finding a
     *       a best fit of the corners, and then taking a decaying exponential
     *       of the distance between the corresponding corners, and taking the
     *       product of all accuracy these individual, per-corner accuracy
     *       values of each corner.
     *
     * param: (rhs) the bounding box to be compared
     *
     * ret: a value between zero and one; 1 corresponds to 100% accuracy.
     */
    float accuracyWRT(const BoundingBox& rhs) const;
    std::vector<PointXYZ> getCorners() const;
  };
  


  //struct which wraps a point cloud with a bit of extra functionality
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
