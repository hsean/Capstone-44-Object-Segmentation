//
//  api.cpp
//  PCLTestbench
//
//  Created by Matthew Whiteside on 3/24/16.
//  Copyright © 2016 capstone44. All rights reserved.
//

#include "api.hpp"

using namespace pcl;
using namespace c44;
using namespace std;
vector<PointXYZ> API::getObjBoundingBoxWRT(const ReferenceFrame& frame){
  vector<PointXYZ> ret;
  PointCloud<PointXYZ>::Ptr rawCloud(new PointCloud<PointXYZ>),
                            objCloud(new PointCloud<PointXYZ>);
  
  if ( getCurrentFrame(rawCloud) && segmentCloud(rawCloud,objCloud) ){
    BoundingBox bbox(objCloud);
    ret = bbox.getCorners();
  } else {
    std::cerr << "warning: no object found in current frame" << std::endl;
  }
  
  return ret;
}

//private
bool API::getCurrentFrame(PointCloud<PointXYZ>::Ptr dest){
  throw "not implemented yet";
  return false;
}

//private
bool API::segmentCloud(PointCloud<PointXYZ>::Ptr src,
                       PointCloud<PointXYZ>::Ptr dest){
  throw "not implemented yet";
  return false;
}