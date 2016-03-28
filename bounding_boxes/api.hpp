//
//  api.hpp
//  PCLTestbench
//
//

#ifndef api_hpp
#define api_hpp

#include <stdio.h>
#include "bounding_box_utils.hpp"
namespace c44{
  namespace api{
    
    
    
    struct ReferenceFrame{
      const PointXYZ origin,
                     lookAt;
      const Vector3f& up;
    };
    
    
    
    std::vector<PointXYZ> objBoundingBoxWRT(const ReferenceFrame& frame){
      std::vector<PointXYZ> ret;

      /*do the segmentation here\
       * store the bounding box corners into the return vector
       */

      return ret;
    }
    
    bool handIsVisible(){
      return false;
    }
    
    bool elbowIsVisible(){
      return false;
    }
    
    PointXYZ getHandPositionWRT(const ReferenceFrame& frame){
      PointXYZ ret;
      throw "not implemented yet";
      return ret;
    }
    
    PointXYZ getElbowPositionWRT(const ReferenceFrame& frame){
      PointXYZ ret;
      throw "not implemented yet";
      return ret;
    }
    
    Vector3f getElbowToHandVectorWRT(const ReferenceFrame& frame){
      Vector3f ret(0,0,0);
      throw "not implemented yet";
      return ret;
    }
  }
}

#endif /* api_hpp */
