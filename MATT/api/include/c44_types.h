

#ifndef c44_types_h
#define c44_types_h

namespace c44{
  typedef std::pair<std::string, std::vector<float> > histogram_t;
  typedef typename flann::ChiSquareDistance<float> default_metric;
  //typedef typename flann::L1<float> default_metric;
  static const EstimationMethod dflt_est_method = EstimationMethod::GRSD;
  typedef typename fusion::result_of::value_at<c44::type_vec, mpl::int_<dflt_est_method>>::type dflt_hist_type;
}

#endif