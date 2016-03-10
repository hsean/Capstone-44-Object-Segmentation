//
//  easy_filters.hpp
//
//  Created on 2/17/16.
//

#ifndef easy_filters_hpp
#define easy_filters_hpp

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/common.h>

namespace c44
{
  namespace filters{
    using namespace pcl;
    //Sean's code
    void cutoffZ(PointCloud<PointXYZ>::Ptr source,
                  PointCloud<PointXYZ>::Ptr result,
                  float maxZ = 1.5);
    void cutoffY(PointCloud<PointXYZ>::Ptr source,
                  PointCloud<PointXYZ>::Ptr result,
                  float maxY);
    void downsample(PointCloud<PointXYZ>::Ptr source,
                    PointCloud<PointXYZ>::Ptr result,
                    float voxelSize = 0.01);
    void removeStatOutliers(PointCloud<pcl::PointXYZ>::Ptr source,
                PointCloud<PointXYZ>::Ptr result);
  }
}


#endif /* easy_filters_hpp */
