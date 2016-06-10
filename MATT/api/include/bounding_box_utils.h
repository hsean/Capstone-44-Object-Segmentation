//
//  bounding_box_utils.h
//  PCLTestbench
//
//  Created on 2/10/16.
//

#ifndef bounding_box_utils_hpp
#define bounding_box_utils_hpp
#include <stdio.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/vfh.h>
#include <stdio.h>
#include <pcl/features/cvfh.h>
#include <pcl/surface/mls.h>
#include <pcl/features/our_cvfh.h>
#include <pcl/features/grsd.h>
#include <pcl/features/esf.h>



namespace c44{
  using namespace pcl;
  using namespace Eigen;
  using namespace std;
  typedef PointCloud<PointXYZ> Cloud3D;

  struct SegmentationConfig{
  public:
    float voxelSize = 0.015;
    float sampleSize = 50;
    float stdDev = 1.0;
    float iterationDivisor = 1.0;
  };

  
  
  struct BoundingBox : public pcl::PCLBase <PointXYZ>{
  private:
    void computeMeanValue();
    Eigen::Vector3f mean_value_;
    
    /** \brief Major eigen vector */
    Eigen::Vector3f major_axis_;
    
    /** \brief Middle eigen vector */
    Eigen::Vector3f middle_axis_;
    
    /** \brief Minor eigen vector */
    Eigen::Vector3f minor_axis_;
    
    /** \brief Major eigen value */
    float major_value_;
    
    /** \brief Middle eigen value */
    float middle_value_;
    
    /** \brief Minor eigen value */
    float minor_value_;

    using PCLBase <PointXYZ>::indices_;
    
    //pcl::PointIndices::Ptr indices_;
    void computeEigenVectors(const Eigen::Matrix <float, 3, 3>& covariance_matrix,
                        Eigen::Vector3f& major_axis,
                        Eigen::Vector3f& middle_axis,
                        Eigen::Vector3f& minor_axis,
                        float& major_value,
                        float& middle_value,
                        float& minor_value);
    void computeCovarianceMatrix(Eigen::Matrix <float, 3, 3>& covariance_matrix) const;
    void computeOBB ();
  public:
    BoundingBox(const Cloud3D::Ptr cloud);
      
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
  


  

}
#endif /* bounding_box_utils_hpp  */
