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

namespace c44{
	using namespace pcl;
	using namespace Eigen;			
	
	class SegmentationPipeline{
		Cloud3D::Ptr cloudMinusObjectsAndPlane;
		SACSegmentationFromNormals<PointXYZ, Normal> seg;
		search::KdTree<PointXYZ>::Ptr tree;
		PointCloud<Normal>::Ptr cloud_normals, normalsMinusObjectsAndPlane;
    Plane* plane;
    
    bool extractPlane();
    bool extractGraspableObject(SacModel model);
	public:
		SegmentationPipeline(Cloud3D::Ptr unfilteredCloud);
    bool performSegmentation();
		std::vector<GraspableObject> graspableObjects;
		
    void extractObjects();
    ~SegmentationPipeline();
	};


}

#endif //geometric_object_hpp