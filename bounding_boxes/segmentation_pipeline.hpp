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

#include "geometries.hpp"
namespace C44{
	using namespace pcl;
	using namespace Eigen;			
	
	class SegmentationPipeline{
		Cloud3D::Ptr noiseFreeCloud, cloudMinusObjectsAndPlane;
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
		
    Cloud3D::Ptr getNoiseFreeCloud() const{
      return noiseFreeCloud;
    }
    void extractObjects();
    ~SegmentationPipeline();
	};


}

#endif //geometric_object_hpp