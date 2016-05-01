//
//  Geometries.cpp
//  PCLTestbench
//
//  Created on 2/10/16.
//

#include "rigid_body.h"
#include <pcl/features/crh.h>
#include <pcl/recognition/crh_alignment.h>

#define TARGET_OBJ_DIAMETER 4.0
using namespace c44;
using namespace pcl;

BoundingBox GraspableObject::getBoundingBox() const{
  return BoundingBox(this->point_cloud);
}




