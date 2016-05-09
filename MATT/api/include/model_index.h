

#ifndef ModelIndex_h
#define ModelIndex_h
#include <stdio.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <string>
#include <rigid_body.h>
#include <flann/flann.h>
#include <flann/io/hdf5.h>
#include <c44_types.h>


namespace c44{
  
  template<EstimationMethod E, typename distance_metric_t>
  class ModelIndex{
    flann::Index<distance_metric_t>* index = nullptr;
    std::vector<histogram_t> models;
  public:
    
    
    ModelIndex(const std::string& model_src_dir){
      
      loadHistograms<E>(model_src_dir, models);
      if (models.size() == 0){
        pcl::console::print_error("no model files found");
        exit(-1);
      } else{
        flann::Matrix<float> flann_data (
                                         new float[models.size () * models[0].second.size ()],
                                         models.size (), models[0].second.size ()
                                         );
        
        for (size_t i = 0; i < flann_data.rows; ++i)
          for (size_t j = 0; j < flann_data.cols; ++j)
            flann_data[i][j] = models[i].second[j];
        
        
        index = new flann::Index<distance_metric_t>(flann_data, flann::LinearIndexParams ());
        index->buildIndex();
      }
    }

    
    
    void match(const histogram_t &model,
                   int k,
                   flann::Matrix<int> &indices,
                   flann::Matrix<float> &distances)
    {
      // Query point
      flann::Matrix<float> p = flann::Matrix<float>(new float[model.second.size ()],
                                                    1, model.second.size ());
      memcpy (&p.ptr ()[0], &model.second[0], p.cols * p.rows * sizeof (float));
      
      indices = flann::Matrix<int>(new int[k], 1, k);
      distances = flann::Matrix<float>(new float[k], 1, k);
      index->knnSearch (p, indices,
                        distances, k,
                        flann::SearchParams (512));
      delete[] p.ptr ();
      
      
    }
    
    
    histogram_t& getModelAtIndex(size_t index) {
      return models[index];
    }
  };
}

#endif