//
//  segmentation_pipeline.hpp
//  PCLTestbench
//
//  A few utility data structures and functions to encapsulate
//  the extraction of bounding boxes.
//
//  Created 2/5/16
//

#ifndef geometric_object_hpp
#define geometric_object_hpp

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "geometries.hpp"
#include "c44_filters.hpp"
#include "realsense_toolkit.h"

namespace c44{
	using namespace pcl;
	using namespace Eigen;			
	
	class SegmentationPipeline{
		Cloud3D::Ptr cloudMinusObjectsAndPlane, denoisedCloud, convexHull;
		SACSegmentationFromNormals<PointXYZ, Normal> seg;
		search::KdTree<PointXYZ>::Ptr tree;
		PointCloud<Normal>::Ptr cloud_normals, normalsMinusObjectsAndPlane;
    //Plane* plane;
    
    
    bool extractPrism();
    bool extractGraspableObject(SacModel model);
    float iterationDivisor;
	public:
		SegmentationPipeline(Cloud3D::Ptr unfilteredCloud,
                         float voxelSize,
                         float sampleSize,
                         float stdDev,
                         float iterationDivisor,
                         bool downsample = true);
    bool performSegmentation();
		std::vector<GraspableObject> graspableObjects;
    void extractObjects();
    Cloud3D::Ptr getConvexHull() const{
      return convexHull;
    }
	};


}

#endif //geometric_object_hpp