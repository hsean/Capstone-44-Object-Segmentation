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

const string RigidBodyWithHistogram<VFH>::fieldName = "vfh";
const string RigidBodyWithHistogram<VFH>::fileExt = ".vfh";

const string RigidBodyWithHistogram<CVFH>::fieldName = "vfh";
const string RigidBodyWithHistogram<CVFH>::fileExt = ".cvfh";

const string RigidBodyWithHistogram<ESF>::fieldName = "esf";
const string RigidBodyWithHistogram<ESF>::fileExt = ".esf";

const string RigidBodyWithHistogram<GRSD>::fieldName = "grsd";
const string RigidBodyWithHistogram<GRSD>::fileExt = ".grsd";

const string RigidBodyWithHistogram<OURCVFH>::fieldName = "vfh";
const string RigidBodyWithHistogram<OURCVFH>::fileExt = ".ourcvfh";



