//
//  segmentation_pipeline.hpp
//  PCLTestbench
//
//  A few utility data structures and functions to encapsulate
//  the extraction of bounding boxes.
//
//  Created 2/5/16
//

#ifndef segmentation_pipeline_hpp
#define segmentation_pipeline_hpp

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "bounding_box_utils.hpp"
#include "c44_filters.hpp"
#include "realsense_toolkit.h"

namespace c44{
	using namespace pcl;
	using namespace Eigen;
  
	
	class SegmentationPipeline{
		Cloud3D::Ptr objectCloud,
                 denoisedCloud,
                 convexHull,
                 planeCloud;
    ModelCoefficients::Ptr planeCoefficients;
		SACSegmentationFromNormals<PointXYZ, Normal> seg;
		search::KdTree<PointXYZ>::Ptr tree;
		PointCloud<Normal>::Ptr normals;
    const float stdDev, sampleSize;
    
    bool extractPlane();
    bool extractPrism();    
    bool extractGraspableObject(SacModel model);
    float iterationDivisor;
    
	public:
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
	};


}

#endif //segmentation_pipeline