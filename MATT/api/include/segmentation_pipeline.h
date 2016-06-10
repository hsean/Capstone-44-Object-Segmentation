//
// Created by Matthew Whiteside on 4/14/16.
//

#ifndef CAPSTONE44_PLANAR_SCENE_H
#define CAPSTONE44_PLANAR_SCENE_H
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <bounding_box_utils.h>
#include <rigid_body.h>
#include <realsense_toolkit.h>
#include <histogram_utils.h>
#include <algorithm>

template<class T>
struct CVFHDistMetric
{
  typedef bool is_kdtree_distance;
  
  typedef T ElementType;
  typedef typename flann::Accumulator<T>::Type ResultType;
  
  /**
   *  Compute the chi-square distance
   */
  template <typename Iterator1, typename Iterator2>
  ResultType operator()(Iterator1 a, Iterator2 b, size_t size, ResultType worst_dist = -1) const
  {
    ResultType numerator, denominator;
    Iterator1 last = a + size;
    numerator = ResultType();
    denominator = ResultType();
    while (a < last) {
      T a_ = *a;
      T b_ = *b;
      numerator += (ResultType)(std::min<T>(a_,b_));
      denominator += (ResultType)(std::max<T>(a_,b_));
      ++a;
      ++b;
//      if ((worst_dist>0)&&(result>worst_dist)) {
//        return result;
//      }

    }
    return 1 - (1 + numerator)/(1 + denominator);
  }
  
  /**
   * Partial distance, used by the kd-tree.
   */
  template <typename U, typename V>
  inline ResultType accum_dist(const U& a, const V& b, int) const
  {
//    ResultType result = ResultType();
//    ResultType sum, diff;
//    
//    sum = (ResultType)(a+b);
//    if (sum>0) {
//      diff = (ResultType)(a-b);
//      result = diff*diff/sum;
//    }
//    return result;
    throw "needs implementation.";
  }
};



namespace c44{
  using namespace pcl;
  using namespace Eigen;
  
  
  class SegmentationPipeline{

    
    ModelCoefficients::Ptr planeCoefficients;
    SACSegmentationFromNormals<PointXYZ, Normal> seg;
    PointCloud<Normal>::Ptr normals;
    const float stdDev, sampleSize;
    vector<RigidBody> objects;

    bool extractPlane();
    bool extractPrism();
    //bool extractGraspableObject(SacModel model);
    void clusterize(float cluster_tolerance);
    float iterationDivisor;

  public:
    static const float normal_search_radius;
    Cloud3D::Ptr convexHull, planeCloud,objectCloud,cloudMinusPrism,
    denoisedCloud;
    
    
    SegmentationPipeline(Cloud3D::Ptr rawCloud,
                         float voxelSize,
                         float sampleSize,
                         float stdDev,
                         float iterationDivisor);
    bool performSegmentation(float cluster_tolerance = 0.1);
    //void extractObjects();
    Cloud3D::Ptr getConvexHull() const{
      return convexHull;
    }
    
    
    size_t objectCount() const {
      return objects.size();
    }
    
    Vector3f getPlaneNormal() const{
        Vector3f ret(
                (*planeCoefficients).values[0],
                (*planeCoefficients).values[1],
                (*planeCoefficients).values[2]
        );
        return ret.normalized();
    }
    float getPlaneOffset() const{
        return (*planeCoefficients).values[3];
    }
    
    RigidBody& getObjectAtIndex(const int index){
      return objects[index];
    }
    
    
  };


}
#endif //CAPSTONE44_PLANAR_SCENE_H
