//
//  easy_filters.cpp
//  PCLTestbench
//
//  Created by Matthew Whiteside on 2/17/16.
//  Copyright © 2016 capstone44. All rights reserved.
//

#include "c44_filters.hpp"

////////////////////////////////////////////////////////////////////////////////
namespace c44{
  namespace filters{
    
    void downsample(PointCloud<PointXYZ>::Ptr source,
                    PointCloud<PointXYZ>::Ptr result,
                    float voxelSize)
    {
      VoxelGrid<PointXYZ> sor;
      sor.setInputCloud (source);
      sor.setLeafSize (voxelSize, voxelSize, voxelSize);
      sor.filter (*result);
    }
    
    
    ////////////////////////////////////////////////////////////////////////////
    void removeStatOutliers(PointCloud<pcl::PointXYZ>::Ptr source,
                            PointCloud<pcl::PointXYZ>::Ptr result)
    {
      pcl::StatisticalOutlierRemoval<PointXYZ> sor;
      sor.setInputCloud (source);
      sor.setMeanK (50);
      sor.setStddevMulThresh (1.0);
      sor.filter (*result);
    }
    
    ////////////////////////////////////////////////////////////////////////////
    void cutoffZ(PointCloud<PointXYZ>::Ptr source,
                 PointCloud<PointXYZ>::Ptr result,
                 float maxZ)
    {
      PassThrough<PointXYZ> pass;
      pass.setInputCloud (source);
      pass.setFilterFieldName ("z");
      pass.setFilterLimits (-1 * maxZ, maxZ);
      pass.filter (*result);
    }
    
    
    ////////////////////////////////////////////////////////////////////////////
    void cutoffY(PointCloud<PointXYZ>::Ptr source,
                 PointCloud<PointXYZ>::Ptr result,
                 float maxY)
    {
      PassThrough<PointXYZ> pass;
      pass.setInputCloud (source);
      pass.setFilterFieldName ("y");
      pass.setFilterLimits (maxY, maxY + 1000.0);
      pass.filter (*result);
    }
  }
}
