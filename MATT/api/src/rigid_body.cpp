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
using namespace fusion;

BoundingBox GraspableObject::getBoundingBox() const{
  return BoundingBox(this->point_cloud);
}

//
//std::string RigidBody::fileExt(EstimationMethod E){
//  switch (E){
//    case VFH:
//      return "vfh";
//    case CVFH:
//      return "cvfh";
//    case OURCVFH:
//      return "ourcvfh";
//    case ESF:
//      return "esf";
//    case GRSD:
//      return "grsd";
//  }
//}
//
//std::string RigidBody::fieldName(EstimationMethod E){
//  switch (E){
//    case VFH:
//      return "vfh";
//    case CVFH:
//      return "vfh";
//    case OURCVFH:
//      return "vfh";
//    case ESF:
//      return "esf";
//    case GRSD:
//      return "grsd";
//  }
//}

//
//template <>
//std::string RigidBody::fieldName<VFH>(){
//  return "vfh";
//}
//
//
//template <>
//std::string RigidBody::fieldName<CVFH>(){
//  return "vfh";
//}
//
//template <>
//std::string RigidBody::fieldName<OURCVFH>(){
//  return "vfh";
//}
//
//
//template <>
//std::string RigidBody::fieldName<ESF>(){
//  return "esf";
//}
//
//template <>
//std::string RigidBody::fieldName<GRSD>(){
//  return "grsd";
//}
//
//template <>
//std::string RigidBody::fileExt<VFH>(){
//  return "vfh";
//}
//
//
//template <>
//std::string RigidBody::fileExt<CVFH>(){
//  return "cvfh";
//}
//
//template <>
//std::string RigidBody::fileExt<OURCVFH>(){
//  return "ourcvfh";
//}
//
//
//template <>
//std::string RigidBody::fileExt<ESF>(){
//  return "esf";
//}
//
//template <>
//std::string RigidBody::fileExt<GRSD>(){
//  return "grsd";
//}


template <>
PointCloud<VFHSignature308>::Ptr
RigidBody::computeDescriptor<VFH>() const
{
  // Object for storing the normals.
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  // Object for storing the VFH descriptor.
  pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptor(new pcl::PointCloud<pcl::VFHSignature308>);
  
  // Note: you should have performed preprocessing to cluster out the object
  // from the cloud, and save it to this individual file.
  
  // Estimate the normals.
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
  normalEstimation.setInputCloud(point_cloud);
  normalEstimation.setRadiusSearch(0.03);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
  normalEstimation.setSearchMethod(kdtree);
  normalEstimation.compute(*normals);
  
  // VFH estimation object.
  pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
  vfh.setInputCloud(point_cloud);
  vfh.setInputNormals(normals);
  vfh.setSearchMethod(kdtree);
  // Optionally, we can normalize the bins of the resulting histogram,
  // using the total number of points.
  vfh.setNormalizeBins(true);
  // Also, we can normalize the SDC with the maximum size found between
  // the centroid and any of the cluster's points.
  vfh.setNormalizeDistance(false);
  
  vfh.compute(*descriptor);
  
  return descriptor;
}



template <>
PointCloud<VFHSignature308>::Ptr
RigidBody::computeDescriptor<CVFH>() const
{
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  // Object for storing the CVFH descriptors.
  pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptor(new pcl::PointCloud<pcl::VFHSignature308>);
  
  // Note: you should have performed preprocessing to cluster out the object
  // from the cloud, and save it to this individual file.
  
  
  // Estimate the normals.
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
  normalEstimation.setInputCloud(point_cloud);
  normalEstimation.setRadiusSearch(0.03);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
  normalEstimation.setSearchMethod(kdtree);
  normalEstimation.compute(*normals);
  
  // CVFH estimation object.
  pcl::CVFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> cvfh;
  cvfh.setInputCloud(point_cloud);
  cvfh.setInputNormals(normals);
  cvfh.setSearchMethod(kdtree);
  // Set the maximum allowable deviation of the normals,
  // for the region segmentation step.
  cvfh.setEPSAngleThreshold(5.0 / 180.0 * M_PI); // 5 degrees.
                                                 // Set the curvature threshold (maximum disparity between curvatures),
                                                 // for the region segmentation step.
  cvfh.setCurvatureThreshold(1.0);
  // Set to true to normalize the bins of the resulting histogram,
  // using the total number of points. Note: enabling it will make CVFH
  // invariant to scale just like VFH, but the authors encourage the opposite.
  cvfh.setNormalizeBins(false);
  
  cvfh.compute(*descriptor);
  return descriptor;
}

template <>
PointCloud<VFHSignature308>::Ptr
RigidBody::computeDescriptor<OURCVFH>() const
{
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  
  pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptor(new pcl::PointCloud<pcl::VFHSignature308>);
  
  
  
  // Estimate the normals.
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
  normalEstimation.setInputCloud(point_cloud);
  normalEstimation.setRadiusSearch(0.03);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
  normalEstimation.setSearchMethod(kdtree);
  normalEstimation.compute(*normals);
  
  
  // OUR-CVFH estimation object.
  pcl::OURCVFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> ourcvfh;
  ourcvfh.setInputCloud(point_cloud);
  ourcvfh.setInputNormals(normals);
  ourcvfh.setSearchMethod(kdtree);
  ourcvfh.setEPSAngleThreshold(5.0 / 180.0 * M_PI); // 5 degrees.
  ourcvfh.setCurvatureThreshold(1.0);
  ourcvfh.setNormalizeBins(false);
  ourcvfh.setAxisRatio(0.8);
  
  ourcvfh.compute(*descriptor);
  return descriptor;
}

template <>
PointCloud<ESFSignature640>::Ptr
RigidBody::computeDescriptor<ESF>() const
{
  pcl::PointCloud<ESFSignature640>::Ptr descriptor(new PointCloud<ESFSignature640>);
  
  pcl::ESFEstimation<PointXYZ, ESFSignature640> esf;
  esf.setInputCloud(point_cloud);
  
  esf.compute(*descriptor);
  return descriptor;
}


template <>
PointCloud<GRSDSignature21>::Ptr
RigidBody::computeDescriptor<GRSD>() const
{
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  
  pcl::PointCloud<pcl::GRSDSignature21>::Ptr descriptors(new pcl::PointCloud<pcl::GRSDSignature21>());
  
  // Estimate the normals.
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
  normalEstimation.setInputCloud(point_cloud);
  normalEstimation.setRadiusSearch(0.03);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
  normalEstimation.setSearchMethod(kdtree);
  normalEstimation.compute(*normals);
  
  GRSDEstimation<pcl::PointXYZ, pcl::Normal, pcl::GRSDSignature21> grsd;
  grsd.setInputCloud(RigidBody::point_cloud);
  grsd.setInputNormals(normals);
  grsd.setSearchMethod(kdtree);
  // Search radius, to look for neighbors. Note: the value given here has to be
  // larger than the radius used to estimate the normals.
  grsd.setRadiusSearch(0.05);
  
  grsd.compute(*descriptors);
  return descriptors;

};





