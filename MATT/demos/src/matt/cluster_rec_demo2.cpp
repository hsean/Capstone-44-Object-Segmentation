
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <iostream>

#include <segmentation_pipeline.h>
#include <segmentation_pipeline.hpp>


using namespace pcl;

typedef GRSDSignature21 histogram_t;

int
main (int argc, char** argv)
{
  int k = 6;
  
  double thresh = DBL_MAX;     // No threshold, disabled by default
  
  std::string extension (".pcd");
  transform (extension.begin (), extension.end (), extension.begin (), (int(*)(int))tolower);
  
  c44::SegmentationPipeline<GRSDSignature21>::init(argv[1]);
  
  if (argc < 2)
  {
    pcl::console::print_error
    ("Need at least three parameters! Syntax is: %s <query_vfh_model.pcd> [options] {kdtree.idx} {training_data.h5} {training_data.list}\n", argv[0]);
    pcl::console::print_info ("    where [options] are:  -k      = number of nearest neighbors to search for in the tree (default: ");
    pcl::console::print_value ("%d", k); pcl::console::print_info (")\n");
    pcl::console::print_info ("                          -thresh = maximum distance threshold for a model to be considered VALID (default: ");
    pcl::console::print_value ("%f", thresh); pcl::console::print_info (")\n\n");
    return (-1);
  }
  
  
  transform (extension.begin (), extension.end (), extension.begin (), (int(*)(int))tolower);
  
  // Load the test histogram
  std::vector<int> pcd_indices = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
  vfh_model histogram;
  
  float voxelSize = 0.0025;
  float sampleSize = 100 - 2 * 25;
  
  float stdDev = 1.0;
  float iterationDivisor = 1 + 1.0;
  //float iterationDivisor = 1.0;
  boost::posix_time::time_duration runTime;
  auto startTime = boost::posix_time::microsec_clock::local_time();

  
  
  int hand = 0;
  c44::RigidBodyWithHistogram<histogram_t>* hand_ptr = nullptr;
  pcl::console::parse_argument(argc, argv, "-hand", hand);

  
  PointCloud<PointXYZ>::Ptr hand_scene(new PointCloud<PointXYZ>);
  PCLPointCloud2::Ptr hand_scene_2d(new PCLPointCloud2);

  Eigen::Vector4f origin;
  Eigen::Quaternionf orientation;
  int version;
  pcl::PCDReader pcd;
  if (pcd.read (argv[pcd_indices.at (0)], *hand_scene_2d, origin, orientation, version) < 0)
    return (-1);
  fromPCLPointCloud2 (*hand_scene_2d, *hand_scene);

  c44::SegmentationPipeline<histogram_t> pipeline(hand_scene,
                                voxelSize,
                                sampleSize,
                                stdDev,
                                iterationDivisor);
  
  if (pipeline.performSegmentation()){
    hand_ptr = new c44::RigidBodyWithHistogram<histogram_t>(pipeline.getObjects()[0].point_cloud);
    histogram.first = "hand";
    auto data = hand_ptr->computeDescriptor()->points[0].histogram;
    for (unsigned i = 0; i < histogram_t::descriptorSize(); i++){
      histogram.second.push_back(data[i]);
    }
    
  } else{
    return (-1);
  }
  
  
  pcl::console::parse_argument (argc, argv, "-thresh", thresh);
  // Search for the k closest matches
  pcl::console::parse_argument (argc, argv, "-k", k);
  pcl::console::print_highlight ("Using "); pcl::console::print_value ("%d", k); pcl::console::print_info (" nearest neighbors.\n");
  
  
  flann::Matrix<int> k_indices;
  flann::Matrix<float> k_distances;
  
  pipeline.findModel(histogram, k, k_indices, k_distances);
  
  // Output the results on screen
  pcl::console::print_highlight ("The closest %d neighbors for %s are:\n", k, argv[pcd_indices[0]]);
  for (int i = 0; i < k; ++i)
    pcl::console::print_info ("    %d - %s (%d) with a distance of: %f\n",
                              i, SegmentationPipeline<histogram_t>::getModels().at (k_indices[0][i]).first.c_str (), k_indices[0][i], k_distances[0][i]);
  
  // Load the results
  int n_cells = k;
  if (hand){
    n_cells *= 2;
  }
  pcl::visualization::PCLVisualizer p (argc, argv, "VFH Cluster Classifier");
  int y_s = (int)floor (sqrt ((double)n_cells));
  int x_s = y_s + (int)ceil ((n_cells / (double)y_s) - y_s);
  double x_step = (double)(1 / (double)x_s);
  double y_step = (double)(1 / (double)y_s);
  pcl::console::print_highlight ("Preparing to load ");
  pcl::console::print_value ("%d", k);
  pcl::console::print_info (" files (");
  pcl::console::print_value ("%d", x_s);
  pcl::console::print_info ("x");
  pcl::console::print_value ("%d", y_s);
  pcl::console::print_info (" / ");
  pcl::console::print_value ("%f", x_step);
  pcl::console::print_info ("x");
  pcl::console::print_value ("%f", y_step);
  pcl::console::print_info (")\n");
  
  int viewport = 0, l = 0, m = 0;
  for (int i = 0; i < k; ++i)
  {
    std::string cloud_name = SegmentationPipeline<histogram_t>::getModels().at (k_indices[0][i]).first;
    boost::replace_last (cloud_name, "_vfh", "");
    
    p.createViewPort (l * x_step, m * y_step, (l + 1) * x_step, (m + 1) * y_step, viewport);
    l++;
    if (l >= x_s)
    {
      l = 0;
      m++;
    }
    
    pcl::PCLPointCloud2 cloud;
    pcl::console::print_highlight (stderr, "Loading "); pcl::console::print_value (stderr, "%s ", cloud_name.c_str ());
    if (pcl::io::loadPCDFile (cloud_name, cloud) == -1)
      break;
    
    // Convert from blob to PointCloud
    pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
    pcl::fromPCLPointCloud2 (cloud, cloud_xyz);
    
    if (cloud_xyz.points.size () == 0)
      break;
    
    pcl::console::print_info ("[done, ");
    pcl::console::print_value ("%d", (int)cloud_xyz.points.size ());
    pcl::console::print_info (" points]\n");
    pcl::console::print_info ("Available dimensions: ");
    pcl::console::print_value ("%s\n", pcl::getFieldsList (cloud).c_str ());
    
    // Demean the cloud
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid (cloud_xyz, centroid);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_demean (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::demeanPointCloud<pcl::PointXYZ> (cloud_xyz, centroid, *cloud_xyz_demean);
    // Add to renderer*
    p.addPointCloud (cloud_xyz_demean, cloud_name, viewport);
    std::stringstream ss;
    ss << k_distances[0][i];
    if (k_distances[0][i] > thresh)
    {
      p.addText (ss.str (), 20, 30, 1, 0, 0, ss.str (), viewport);  // display the text with red
      
      // Create a red line
      pcl::PointXYZ min_p, max_p;
      pcl::getMinMax3D (*cloud_xyz_demean, min_p, max_p);
      std::stringstream line_name;
      line_name << "line_" << i;
      p.addLine (min_p, max_p, 1, 0, 0, line_name.str (), viewport);
      p.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, line_name.str (), viewport);
    }
    else
      p.addText (ss.str (), 20, 30, 0, 1, 0, ss.str (), viewport);
    
    // Increase the font size for the score*
    p.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_FONT_SIZE, 18, ss.str (), viewport);
    
    // Add the cluster name
    p.addText (cloud_name, 20, 10, cloud_name, viewport);

    
    viewport++;
    p.createViewPort (l * x_step, m * y_step, (l + 1) * x_step, (m + 1) * y_step, viewport);
    l++;
    if (l >= x_s)
    {
      l = 0;
      m++;
    }
    
    ss.clear();
    ss << "hand " << viewport;
    p.addPointCloud (hand_ptr->point_cloud, ss.str(),  viewport);
    
  }
  // Add coordianate systems to all viewports
  p.addCoordinateSystem (0.1, "global", 0);
  
  p.spin ();
  return (0);
}