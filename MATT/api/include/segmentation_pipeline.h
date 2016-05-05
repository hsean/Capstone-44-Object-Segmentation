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
//#include "c44_filters.hpp"
#include <realsense_toolkit.h>
#include <histogram_utils.h>
#ifndef MODEL_DIR
#define MODEL_DIR "/Users/matt/code/cpp/pcl-stuff/cap44/xcode-build/c44/demos/Debug/3DScanOfInmoovHand.ply_output/"
#endif
const boost::filesystem::path base_dir(MODEL_DIR);

namespace c44{
  using namespace pcl;
  using namespace Eigen;
  
  
  template <HistogramType histogram_t>
  class SegmentationPipeline{
    
    static vector<vfh_model> models;
    static flann::Index<flann::ChiSquareDistance<float>> *index;
    
    
    
    Cloud3D::Ptr objectCloud,
            denoisedCloud,
            convexHull,
            planeCloud;
    ModelCoefficients::Ptr planeCoefficients;
    SACSegmentationFromNormals<PointXYZ, Normal> seg;
    //search::KdTree<PointXYZ>::Ptr tree;
    //std::vector<Cloud3D::Ptr> clusters;
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
    
    bool findHand() const;
    
    void findModel(const vfh_model &model,
                   int k,
                   flann::Matrix<int> &indices,
                   flann::Matrix<float> &distances);

  };


}
#endif //CAPSTONE44_PLANAR_SCENE_H
