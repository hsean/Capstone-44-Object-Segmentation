//
//  api.hpp
//  PCLTestbench
//
//

#ifndef api_hpp
#define api_hpp

#include <stdio.h>
#include "bounding_box_utils.hpp"
#include <queue>

namespace c44{
  struct ReferenceFrame{
    const PointXYZ origin, lookAt;
    const Vector3f& up;
  };
  
  

  class API{
  private:
    static SegmentationConfig config;
    static std::queue<AccuracyReport> accuracyStats;

    //provided by Jame's realsense code
    static bool getCurrentFrame(PointCloud<PointXYZ>::Ptr dest);
    
    //provided by Sean's segmentation code
    //the name of this function is open to suggestions
    static bool segmentCloud(PointCloud<PointXYZ>::Ptr src,
                             PointCloud<PointXYZ>::Ptr dest);


  public:
    static std::vector<PointXYZ>
    getObjBoundingBoxWRT(const ReferenceFrame& frame);
    
    static bool handIsVisible();
    
    static PointXYZ getHandPositionWRT(const ReferenceFrame& frame);

  };
}

#endif /* api_hpp */
