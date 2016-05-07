//
// Created by Matthew Whiteside on 4/14/16.
//

#ifndef CAPSTONE44_PLANAR_SCENE_H
#define CAPSTONE44_PLANAR_SCENE_H
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <flann/flann.h>
#include <flann/io/hdf5.h>
#include <bounding_box_utils.hpp>
#include <rigid_body.h>
#include <realsense_toolkit.h>
#include <histogram_utils.h>
#include <algorithm>

template<class T>
struct CVFHDistMetric
{
  typedef bool is_kdtree_distance;
  
  typedef T ElementType;
  typedef typename Accumulator<T>::Type ResultType;
  
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
  
  //typedef typename flann::ChiSquareDistance<float> dist_type;
  typedef CVFHDistMetric<float> dist_type;
  template <HistogramType histogram_t, typename dist_metric_t = dist_type>
  class SegmentationPipeline{
    
    static vector<vfh_model> models;
    static flann::Index<dist_metric_t> *index;
    
    
    
    Cloud3D::Ptr objectCloud,
            denoisedCloud,
            convexHull,
            planeCloud;
    ModelCoefficients::Ptr planeCoefficients;
    SACSegmentationFromNormals<PointXYZ, Normal> seg;
    PointCloud<Normal>::Ptr normals;
    const float stdDev, sampleSize;
    vector<RigidBodyWithHistogram<histogram_t>> objects;

    bool extractPlane();
    bool extractPrism();
    bool extractGraspableObject(SacModel model);
    void clusterize();
    float iterationDivisor;

  public:
    
    static void init(const std::string& model_src_dr);
    
    static const vector<vfh_model>& getModels(){
      return models;
    }
    SegmentationPipeline(Cloud3D::Ptr rawCloud,
                         float voxelSize,
                         float sampleSize,
                         float stdDev,
                         float iterationDivisor);
    bool performSegmentation();
    std::vector<GraspableObject> graspableObjects;
    void extractObjects();
    Cloud3D::Ptr getConvexHull() const{
      return convexHull;
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
    
    std::vector<RigidBodyWithHistogram<histogram_t>>& getObjects(){
      return objects;
    }
    
    
    void findModel(const vfh_model &model,
                   int k,
                   flann::Matrix<int> &indices,
                   flann::Matrix<float> &distances);

  };


}
#endif //CAPSTONE44_PLANAR_SCENE_H
