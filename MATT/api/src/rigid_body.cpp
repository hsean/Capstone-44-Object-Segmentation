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

BoundingBox RigidBody::getBoundingBox() const{
  return BoundingBox(this->point_cloud);
}

RigidBody::RigidBody(Cloud3D::Ptr cloud,
                     float normal_search_radius) :
point_cloud(cloud),
normal_search_radius(normal_search_radius),
normal_cloud(new PointCloud<Normal>){
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
  normalEstimation.setInputCloud(point_cloud);
  normalEstimation.setRadiusSearch(normal_search_radius);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
  normalEstimation.setSearchMethod(kdtree);
  normalEstimation.compute(*normal_cloud);
}


template <>
PointCloud<VFHSignature308>::Ptr
RigidBody::computeDescriptor<VFH>() const
{
  boost::posix_time::ptime time_before_execution;
  boost::posix_time::ptime time_after_execution;
  boost::posix_time::time_duration difference;
  time_before_execution = boost::posix_time::microsec_clock::local_time();
  
  // Object for storing the VFH descriptor.
  pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptor(new pcl::PointCloud<pcl::VFHSignature308>);
  
  // Note: you should have performed preprocessing to cluster out the object
  // from the cloud, and save it to this individual file.
  
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
  
  // VFH estimation object.
  pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
  vfh.setInputCloud(point_cloud);
  vfh.setInputNormals(normal_cloud);
  vfh.setSearchMethod(kdtree);
  // Optionally, we can normalize the bins of the resulting histogram,
  // using the total number of points.
  vfh.setNormalizeBins(true);
  // Also, we can normalize the SDC with the maximum size found between
  // the centroid and any of the cluster's points.
  vfh.setNormalizeDistance(false);
  
  vfh.compute(*descriptor);
  time_after_execution = boost::posix_time::microsec_clock::local_time();
  difference = time_after_execution - time_before_execution;
  std::cout << "GRSD time = " << std::setw(5) << difference.total_milliseconds() << " ms" << std::endl;

  return descriptor;
}



template <>
PointCloud<VFHSignature308>::Ptr
RigidBody::computeDescriptor<CVFH>() const
{
  pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptor(new pcl::PointCloud<pcl::VFHSignature308>);
  
  // Note: you should have performed preprocessing to cluster out the object
  // from the cloud, and save it to this individual file.
  
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
  
  // CVFH estimation object.
  pcl::CVFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> cvfh;
  cvfh.setInputCloud(point_cloud);
  cvfh.setInputNormals(normal_cloud);
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
  pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptor(new pcl::PointCloud<pcl::VFHSignature308>);
  
  
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
  
  // OUR-CVFH estimation object.
  pcl::OURCVFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> ourcvfh;
  ourcvfh.setInputCloud(point_cloud);
  ourcvfh.setInputNormals(normal_cloud);
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
  boost::posix_time::ptime time_before_execution;
  boost::posix_time::ptime time_after_execution;
  boost::posix_time::time_duration difference;
  time_before_execution = boost::posix_time::microsec_clock::local_time();
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  
  pcl::PointCloud<pcl::GRSDSignature21>::Ptr descriptors(new pcl::PointCloud<pcl::GRSDSignature21>());
  
  // Estimate the normals.
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
  
  GRSDEstimation<pcl::PointXYZ, pcl::Normal, pcl::GRSDSignature21> grsd;
  grsd.setInputCloud(this->point_cloud);
  grsd.setInputNormals(this->normal_cloud);
  grsd.setSearchMethod(kdtree);
  // Search radius, to look for neighbors. Note: the value given here has to be
  
  // larger than the radius used to estimate the normals.
  grsd.setRadiusSearch(normal_search_radius + 0.02);
  
  grsd.compute(*descriptors);
  time_after_execution = boost::posix_time::microsec_clock::local_time();
  difference = time_after_execution - time_before_execution;
  std::cout << "GRSD time = " << std::setw(5) << difference.total_milliseconds() << " ms" << std::endl;

  return descriptors;

};





