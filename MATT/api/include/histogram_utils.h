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
#include <sstream>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <rigid_body.h>
#include <c44_types.h>


namespace c44{

  template<EstimationMethod E>
  std::string fileExt();

  template<EstimationMethod E>
  std::string fieldName();

  template<EstimationMethod E>
  bool loadHist (const boost::filesystem::path &path, histogram_t& hist)
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
      
      field_idx = pcl::getFieldIndex (cloud,fieldName<E>());
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
    pcl::PointCloud<typename fusion::result_of::value_at<c44::type_vec, mpl::int_<E>>::type> point;
    pcl::io::loadPCDFile (path.string (), point);
    hist.second.resize (fusion::result_of::value_at<c44::type_vec, mpl::int_<E>>::type::descriptorSize());
    
    std::vector <pcl::PCLPointField> fields;
    pcl::getFieldIndex (point,
                        fieldName<E>(),
                        fields);
    
    for (size_t i = 0; i < fields[field_idx].count; ++i)
    {
      hist.second[i] = point.points[0].histogram[i];
    }
    hist.first = path.string ();
    return (true);
  }

  template<EstimationMethod E>
  void loadHistograms (const boost::filesystem::path &base_dir,
                       std::vector<histogram_t> &models)
  {
    std::string extension = fileExt<E>();
    if (!boost::filesystem::exists (base_dir) && !boost::filesystem::is_directory (base_dir))
      return;
    
    for (boost::filesystem::directory_iterator it (base_dir); it != boost::filesystem::directory_iterator (); ++it)
    {
      if (boost::filesystem::is_directory (it->status ()))
      {
        std::stringstream ss;
        ss << it->path ();
        pcl::console::print_highlight ("Loading %s (%lu models loaded so far).\n", ss.str ().c_str (), (unsigned long)models.size ());
        loadHistograms<E>(it->path (), models);
      }
      auto currFileExt = boost::filesystem::extension (it->path ());
      if (boost::filesystem::is_regular_file (it->status ()) && currFileExt == extension)
      {
        c44::histogram_t hist;
        if (loadHist<E>(base_dir / it->path ().filename (), hist))
          models.push_back (hist);
      } else{
        pcl::console::print_highlight ("what.....");
      }
    }
  }
}
#endif /* histogram_utils_h */
