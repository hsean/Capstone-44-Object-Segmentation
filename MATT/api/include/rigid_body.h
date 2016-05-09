//

#ifndef RigidBody_h
#define RigidBody_h
#include <stdio.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
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
#include <bounding_box_utils.h>
#include <boost/fusion/sequence/intrinsic/value_at.hpp>
#include <boost/fusion/include/value_at.hpp>


using namespace boost;

enum EstimationMethod{
  VFH,
  CVFH,
  OURCVFH,
  ESF,
  GRSD
};


namespace c44{
  using namespace pcl;
  using namespace boost;
  
  typedef fusion::vector
  <
    VFHSignature308,
    VFHSignature308,
    VFHSignature308,
    ESFSignature640,
    GRSDSignature21
  > type_vec;
  

  struct RigidBody{
    
  public:
    
    template <EstimationMethod E>
    static size_t descriptorSize()
    {
      return fusion::result_of::value_at<c44::type_vec, boost::mpl::int_<E>>::type::descriptorSize();
    }

    
    const ModelCoefficients coefficients;
    Cloud3D::Ptr point_cloud;
    PointCloud<Normal>::Ptr normal_cloud;
      
    RigidBody(ModelCoefficients mc,
              Cloud3D::Ptr cloud,
              PointCloud<Normal>::Ptr normals) :
    coefficients(mc), point_cloud(cloud), normal_cloud(normals) {};
    RigidBody(PointCloud<PointXYZ>::Ptr cloud) : point_cloud(cloud){};

    template<EstimationMethod E>
    typename PointCloud<typename fusion::result_of::value_at<type_vec, mpl::int_<E>>::type>::Ptr
    computeDescriptor() const;

  };

  
  struct Plane : public RigidBody{
  public:
    const PointIndices::Ptr inliers;
    Plane(ModelCoefficients mc,
          Cloud3D::Ptr points,
          PointCloud<Normal>::Ptr normals,
          PointIndices::Ptr _inliers) :
    RigidBody(mc,points,normals), inliers(_inliers){}
  };
  

  struct GraspableObject : RigidBody{
  public:
    GraspableObject(ModelCoefficients mc,
                    Cloud3D::Ptr points,
                    PointCloud<Normal>::Ptr normals) :
    RigidBody(mc,points,normals){}
    
    BoundingBox getBoundingBox() const;
    
  };
}

#endif