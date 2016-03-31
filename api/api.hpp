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
    
    //maintain stats on the last few segmentation runs
    //to see if accuracy is improving or degrading
    static std::queue<AccuracyReport> accuracyStats;

    //provided by Jame's realsense code
    static bool getCurrentFrame(PointCloud<PointXYZ>::Ptr dest);
    
    //provided by Sean's segmentation code
    //the name of this function is open to suggestions
    static bool segmentCloud(PointCloud<PointXYZ>::Ptr src,
                             PointCloud<PointXYZ>::Ptr dest);


  public:

    /*------------------------------------------------------------------------*/
    /**
     * desc: computes the bounding box of the object to be picked up
     *       by the robot
     *
     * param: (frame) the reference frame (i.e., camera position, orientation)
     *        with respect to which the bounding box will be calculated
     *
     * ret: a std::vector of points; the eight corners of the bounding box
     */
    static std::vector<PointXYZ>
    getObjBoundingBoxWRT(const ReferenceFrame& frame);
    /*------------------------------------------------------------------------*/
    
    
    
    /*------------------------------------------------------------------------*/
    /**
     * desc: queries the API to see if the hand is visible
     *
     * ret: true if the hand is in the current frame of the camera, false
     *      otherwise
     */
    static bool handIsVisible();
    /*------------------------------------------------------------------------*/
    
    
    
    /*------------------------------------------------------------------------*/
    /**
     * desc: calculates the position of the robot's hand in the scene
     *
     * param: (frame) the reference frame (i.e., camera position, orientation)
     *        with respect to which the hand position will be calculated
     *
     * ret: the xyz coordinates of the hand
     */
    static PointXYZ getHandPositionWRT(const ReferenceFrame& frame);
    /*------------------------------------------------------------------------*/

  };
}

#endif /* api_hpp */
