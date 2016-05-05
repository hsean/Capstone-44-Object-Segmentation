//
//  histogram_utils.h
//  Capstone44
//
//  Created by Matthew Whiteside on 5/1/16.
//
//

#ifndef histogram_utils_h
#define histogram_utils_h
#include <stdio.h>
#include <boost/filesystem.hpp>
#include <vector>
#include <histogram_utils.h>
#include <sstream>
#include <pcl/common/common.h>
//#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <rigid_body.h>

//using namespace c44;


typedef std::pair<std::string, std::vector<float> > vfh_model;

using namespace pcl;

//forward declaration
template<HistogramType histogram_t>
void loadHistograms (const boost::filesystem::path &base_dir,
                     std::vector<vfh_model> &models);



template<HistogramType histogram_t>
bool loadHist (const boost::filesystem::path &path, vfh_model &vfh)
{
  int field_idx;
  // Load the file as a PCD
  
  
  try
  {
    pcl::PCLPointCloud2 cloud;
    int version;
    Eigen::Vector4f origin;
    Eigen::Quaternionf orientation;
    pcl::PCDReader r;
    int type; unsigned int idx;
    r.readHeader (path.string (), cloud, origin, orientation, version, type, idx);
    
    field_idx = pcl::getFieldIndex (cloud,
                                    c44::RigidBodyWithHistogram<histogram_t>::fieldName);
    if (field_idx == -1)
      return (false);
    if ((int)cloud.width * cloud.height != 1)
      return (false);
  }
  catch (const pcl::InvalidConversionException&)
  {
    return (false);
  }
  
  // Treat the VFH signature as a single Point Cloud
  pcl::PointCloud<typename c44::RigidBodyWithHistogram<histogram_t>::signature_t> point;
  pcl::io::loadPCDFile (path.string (), point);
  vfh.second.resize (c44::RigidBodyWithHistogram<histogram_t>::descriptorSize());
  
  std::vector <pcl::PCLPointField> fields;
  pcl::getFieldIndex (point,
                      c44::RigidBodyWithHistogram<histogram_t>::fieldName,
                      fields);
  
  for (size_t i = 0; i < fields[field_idx].count; ++i)
  {
    vfh.second[i] = point.points[0].histogram[i];
  }
  vfh.first = path.string ();
  return (true);
}


template<HistogramType histogram_t>
void loadHistograms (const boost::filesystem::path &base_dir,
                     std::vector<vfh_model> &models)
{
  std::string extension = c44::RigidBodyWithHistogram<histogram_t>::fileExt;
  if (!boost::filesystem::exists (base_dir) && !boost::filesystem::is_directory (base_dir))
    return;
  
  for (boost::filesystem::directory_iterator it (base_dir); it != boost::filesystem::directory_iterator (); ++it)
  {
    if (boost::filesystem::is_directory (it->status ()))
    {
      std::stringstream ss;
      ss << it->path ();
      pcl::console::print_highlight ("Loading %s (%lu models loaded so far).\n", ss.str ().c_str (), (unsigned long)models.size ());
      loadHistograms<histogram_t>(it->path (), models);
    }
    auto currFileExt = boost::filesystem::extension (it->path ());
    if (boost::filesystem::is_regular_file (it->status ()) && currFileExt == extension)
    {
      vfh_model m;
      if (loadHist<histogram_t>(base_dir / it->path ().filename (), m))
        models.push_back (m);
    } else{
      pcl::console::print_highlight ("what.....");
    }
  }
}


#endif /* histogram_utils_h */
