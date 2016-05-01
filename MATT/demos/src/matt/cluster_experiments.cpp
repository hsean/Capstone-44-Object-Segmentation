/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */
// PCL
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <cfloat>
#include <pcl/visualization/eigen.h>
//#include <pcl/visualization/vtk.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/visualization/histogram_visualizer.h>
#if VTK_MAJOR_VERSION>=6 || (VTK_MAJOR_VERSION==5 && VTK_MINOR_VERSION>6)
#include <pcl/visualization/pcl_plotter.h>
#endif
#include <pcl/visualization/point_picking_event.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/search/kdtree.h>
#include <vtkPolyDataReader.h>
#include <vtkTransform.h>
#include <pcl/common/angles.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>


#include "segmentation_pipeline.h"
#include "desc_genner.h"

using namespace pcl::console;

using namespace Eigen;
using namespace c44;
typedef pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2> ColorHandler;
typedef ColorHandler::Ptr ColorHandlerPtr;
typedef ColorHandler::ConstPtr ColorHandlerConstPtr;

typedef pcl::visualization::PointCloudGeometryHandler<pcl::PCLPointCloud2> GeometryHandler;
typedef GeometryHandler::Ptr GeometryHandlerPtr;
typedef GeometryHandler::ConstPtr GeometryHandlerConstPtr;

typedef pcl::Histogram<90> CRH90;

#define NORMALS_SCALE 0.01f
#define PC_SCALE 0.001f

bool
isValidFieldName (const std::string &field)
{
  if (field == "_")
    return (false);
  
  if ((field == "vp_x") || (field == "vx") || (field == "vp_y") || (field == "vy") || (field == "vp_z") || (field == "vz"))
    return (false);
  return (true);
}

bool
isMultiDimensionalFeatureField (const pcl::PCLPointField &field)
{
  if (field.count > 1)
    return (true);
  return (false);
}

bool
isOnly2DImage (const pcl::PCLPointField &field)
{
  if (field.name == "rgba" || field.name == "rgb")
    return (true);
  return (false);
}


bool
loadCloud (const char* filename, PCLPointCloud2 &cloud)
{
  TicToc tt;
  print_highlight ("Loading "); print_value ("%s ", filename);
  
  pcl::PLYReader reader;
  tt.tic ();
  std::string _filename(filename);
  if (reader.read (_filename, cloud) < 0)
    return (false);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
  print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList (cloud).c_str ());
  
  return (true);
}


void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s <file_name 1..N>.<pcd or vtk> <options>\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     -bc r,g,b                = background color\n");
  print_info ("                     -fc r,g,b                = foreground color\n");
  print_info ("                     -ps X                    = point size ("); print_value ("1..64"); print_info (") \n");
  print_info ("                     -opaque X                = rendered point cloud opacity ("); print_value ("0..1"); print_info (")\n");
  print_info ("                     -shading X               = rendered surface shading ("); print_value ("'flat' (default), 'gouraud', 'phong'"); print_info (")\n");
  print_info ("                     -position x,y,z          = absolute point cloud position in metres\n");
  print_info ("                     -orientation r,p,y       = absolute point cloud orientation (roll, pitch, yaw) in radians\n");
  
  print_info ("                     -ax "); print_value ("n"); print_info ("                    = enable on-screen display of ");
  print_color (stdout, TT_BRIGHT, TT_RED, "X"); print_color (stdout, TT_BRIGHT, TT_GREEN, "Y"); print_color (stdout, TT_BRIGHT, TT_BLUE, "Z");
  print_info (" axes and scale them to "); print_value ("n\n");
  print_info ("                     -ax_pos X,Y,Z            = if axes are enabled, set their X,Y,Z position in space (default "); print_value ("0,0,0"); print_info (")\n");
  
  print_info ("\n");
  print_info ("                     -cam (*)                 = use given camera settings as initial view\n");
  print_info (stderr, " (*) [Clipping Range / Focal Point / Position / ViewUp / Distance / Field of View Y / Window Size / Window Pos] or use a <filename.cam> that contains the same information.\n");
  
  print_info ("\n");
  print_info ("                     -multiview 0/1           = enable/disable auto-multi viewport rendering (default "); print_value ("disabled"); print_info (")\n");
  print_info ("\n");
  
  print_info ("\n");
  print_info ("                     -normals 0/X             = disable/enable the display of every Xth point's surface normal as lines (default "); print_value ("disabled"); print_info (")\n");
  print_info ("                     -normals_scale X         = resize the normal unit vector size to X (default "); print_value ("0.02"); print_info (")\n");
  print_info ("\n");
  print_info ("                     -pc 0/X                  = disable/enable the display of every Xth point's principal curvatures as lines (default "); print_value ("disabled"); print_info (")\n");
  print_info ("                     -pc_scale X              = resize the principal curvatures vectors size to X (default "); print_value ("0.02"); print_info (")\n");
  print_info ("\n");
  print_info ("                     -immediate_rendering 0/1 = use immediate mode rendering to draw the data (default: "); print_value ("disabled"); print_info (")\n");
  print_info ("                                                Note: the use of immediate rendering will enable the visualization of larger datasets at the expense of extra RAM.\n");
  print_info ("                                                See http://en.wikipedia.org/wiki/Immediate_mode for more information.\n");
  print_info ("                     -vbo_rendering 0/1       = use OpenGL 1.4+ Vertex Buffer Objects for rendering (default: "); print_value ("disabled"); print_info (")\n");
  print_info ("                                                Note: the use of VBOs will enable the visualization of larger datasets at the expense of extra RAM.\n");
  print_info ("                                                See http://en.wikipedia.org/wiki/Vertex_Buffer_Object for more information.\n");
  print_info ("\n");
  print_info ("                     -use_point_picking       = enable the usage of picking points on screen (default "); print_value ("disabled"); print_info (")\n");
  print_info ("\n");
  print_info ("                     -optimal_label_colors    = maps existing labels to the optimal sequential glasbey colors, label_ids will not be mapped to fixed colors (default "); print_value ("disabled"); print_info (")\n");
  print_info ("\n");
  
  print_info ("\n(Note: for multiple .pcd files, provide multiple -{fc,ps,opaque,position,orientation} parameters; they will be automatically assigned to the right file)\n");
}

// Global visualizer object
#if VTK_MAJOR_VERSION>=6 || (VTK_MAJOR_VERSION==5 && VTK_MINOR_VERSION>6)
pcl::visualization::PCLPlotter ph_global;
#endif
boost::shared_ptr<pcl::visualization::PCLVisualizer> p;
std::vector<boost::shared_ptr<pcl::visualization::ImageViewer> > imgs;
pcl::search::KdTree<pcl::PointXYZ> _search;
pcl::PCLPointCloud2::Ptr cloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr xyzcloud;

void
pp_callback (const pcl::visualization::PointPickingEvent& event, void* cookie)
{
  int idx = event.getPointIndex ();
  if (idx == -1)
    return;
  
  if (!cloud)
  {
    cloud = *reinterpret_cast<pcl::PCLPointCloud2::Ptr*> (cookie);
    xyzcloud.reset (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2 (*cloud, *xyzcloud);
    _search.setInputCloud (xyzcloud);
  }
  // Return the correct index in the cloud instead of the index on the screen
  std::vector<int> indices (1);
  std::vector<float> distances (1);
  
  // Because VTK/OpenGL stores data without NaN, we lose the 1-1 correspondence, so we must search for the real point
  pcl::PointXYZ picked_pt;
  event.getPoint (picked_pt.x, picked_pt.y, picked_pt.z);
  _search.nearestKSearch (picked_pt, 1, indices, distances);
  
  PCL_INFO ("Point index picked: %d (real: %d) - [%f, %f, %f]\n", idx, indices[0], picked_pt.x, picked_pt.y, picked_pt.z);
  
  idx = indices[0];
  // If two points were selected, draw an arrow between them
  pcl::PointXYZ p1, p2;
  if (event.getPoints (p1.x, p1.y, p1.z, p2.x, p2.y, p2.z) && p)
  {
    std::stringstream ss;
    ss << p1 << p2;
    p->addArrow<pcl::PointXYZ, pcl::PointXYZ> (p1, p2, 1.0, 1.0, 1.0, ss.str ());
    return;
  }
  
  // Else, if a single point has been selected
  std::stringstream ss;
  ss << idx;
  // Get the cloud's fields
  for (size_t i = 0; i < cloud->fields.size (); ++i)
  {
    if (!isMultiDimensionalFeatureField (cloud->fields[i]))
      continue;
    PCL_INFO ("Multidimensional field found: %s\n", cloud->fields[i].name.c_str ());
#if VTK_MAJOR_VERSION>=6 || (VTK_MAJOR_VERSION==5 && VTK_MINOR_VERSION>6)
    ph_global.addFeatureHistogram (*cloud, cloud->fields[i].name, idx, ss.str ());
    ph_global.renderOnce ();
#endif
  }
  if (p)
  {
    pcl::PointXYZ pos;
    event.getPoint (pos.x, pos.y, pos.z);
    p->addText3D<pcl::PointXYZ> (ss.str (), pos, 0.0005, 1.0, 1.0, 1.0, ss.str ());
  }
  
}

/* ---[ */
int
main (int argc, char** argv)
{
  srand (static_cast<unsigned int> (time (0)));
  SegmentationPipeline::init();
  print_info ("The viewer window provides interactive commands; for help, press 'h' or 'H' from within the window.\n");
  
  if (argc < 2)
  {
    printHelp (argc, argv);
    return (-1);
  }
  
  bool debug = false;
  pcl::console::parse_argument (argc, argv, "-debug", debug);
  if (debug)
    pcl::console::setVerbosityLevel (pcl::console::L_DEBUG);
  
  // Parse the command line arguments for .pcd files
  std::vector<int> p_file_indices   = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
  std::vector<int> vtk_file_indices = pcl::console::parse_file_extension_argument (argc, argv, ".vtk");
  
  if (p_file_indices.size () == 0 && vtk_file_indices.size () == 0)
  {
    print_error ("No .PCD or .VTK file given. Nothing to visualize.\n");
    return (-1);
  }
  
  // Command line parsing
  double bcolor[3] = {0, 0, 0};
  pcl::console::parse_3x_arguments (argc, argv, "-bc", bcolor[0], bcolor[1], bcolor[2]);
  
  std::vector<double> fcolor_r, fcolor_b, fcolor_g;
  bool fcolorparam = pcl::console::parse_multiple_3x_arguments (argc, argv, "-fc", fcolor_r, fcolor_g, fcolor_b);
  
  std::vector<double> pose_x, pose_y, pose_z, pose_roll, pose_pitch, pose_yaw;
  bool poseparam = pcl::console::parse_multiple_3x_arguments (argc, argv, "-position", pose_x, pose_y, pose_z);
  poseparam &= pcl::console::parse_multiple_3x_arguments (argc, argv, "-orientation", pose_roll, pose_pitch, pose_yaw);
  
  std::vector<int> psize;
  pcl::console::parse_multiple_arguments (argc, argv, "-ps", psize);
  
  std::vector<double> opaque;
  pcl::console::parse_multiple_arguments (argc, argv, "-opaque", opaque);
  
  std::vector<std::string> shadings;
  pcl::console::parse_multiple_arguments (argc, argv, "-shading", shadings);
  
  int mview = 0;
  pcl::console::parse_argument (argc, argv, "-multiview", mview);
  
  int normals = 0;
  pcl::console::parse_argument (argc, argv, "-normals", normals);
  float normals_scale = NORMALS_SCALE;
  pcl::console::parse_argument (argc, argv, "-normals_scale", normals_scale);
  
  int pc = 0;
  pcl::console::parse_argument (argc, argv, "-pc", pc);
  float pc_scale = PC_SCALE;
  pcl::console::parse_argument (argc, argv, "-pc_scale", pc_scale);
  
  bool use_vbos = false;
  pcl::console::parse_argument (argc, argv, "-vbo_rendering", use_vbos);
  if (use_vbos)
    print_highlight ("Vertex Buffer Object (VBO) visualization enabled.\n");
  
  bool use_pp   = pcl::console::find_switch (argc, argv, "-use_point_picking");
  if (use_pp)
    print_highlight ("Point picking enabled.\n");
  
  bool use_optimal_l_colors = pcl::console::find_switch (argc, argv, "-optimal_label_colors");
  if (use_optimal_l_colors)
    print_highlight ("Optimal glasbey colors are being assigned to existing labels.\nNote: No static mapping between label ids and colors\n");
  
  // If VBOs are not enabled, then try to use immediate rendering
  bool use_immediate_rendering = false;
  if (!use_vbos)
  {
    pcl::console::parse_argument (argc, argv, "-immediate_rendering", use_immediate_rendering);
    if (use_immediate_rendering)
      print_highlight ("Using immediate mode rendering.\n");
  }
  const unsigned numRows = 1;
  const unsigned numCols = 2;
  const unsigned numIterations = numRows * numCols;
  // Multiview enabled?
  int y_s = 0, x_s = 0;
  double x_step = 0, y_step = 0;
  if (mview)
  {
    print_highlight ("Multi-viewport rendering enabled.\n");
    
    y_s = static_cast<int>(floor (sqrt (static_cast<float>(numIterations))));
    x_s = y_s + static_cast<int>(ceil (double (numIterations) / double (y_s) - y_s));
    
    if (p_file_indices.size () != 0)
    {
      print_highlight ("Preparing to load "); print_value ("%d", p_file_indices.size ()); print_info (" pcd files.\n");
    }
    
    if (vtk_file_indices.size () != 0)
    {
      print_highlight ("Preparing to load "); print_value ("%d", vtk_file_indices.size ()); print_info (" vtk files.\n");
    }
    
    x_step = static_cast<double>(1.0 / static_cast<double>(x_s));
    y_step = static_cast<double>(1.0 / static_cast<double>(y_s));
    print_value ("%d", x_s);    print_info ("x"); print_value ("%d", y_s);
    print_info (" / ");      print_value ("%f", x_step); print_info ("x"); print_value ("%f", y_step);
    print_info (")\n");
  }
  
  // Fix invalid multiple arguments
  if (psize.size () != p_file_indices.size () && psize.size () > 0)
    for (size_t i = psize.size (); i < p_file_indices.size (); ++i)
      psize.push_back (1);
  if (opaque.size () != p_file_indices.size () && opaque.size () > 0)
    for (size_t i = opaque.size (); i < p_file_indices.size (); ++i)
      opaque.push_back (1.0);
  
  if (shadings.size () != p_file_indices.size () && shadings.size () > 0)
    for (size_t i = shadings.size (); i < p_file_indices.size (); ++i)
      shadings.push_back ("flat");
  
  // Create the PCLVisualizer object
#if VTK_MAJOR_VERSION>=6 || (VTK_MAJOR_VERSION==5 && VTK_MINOR_VERSION>6)
  boost::shared_ptr<pcl::visualization::PCLPlotter> ph;
#endif
  // Using min_p, max_p to set the global Y min/max range for the histogram
  float min_p = FLT_MAX; float max_p = -FLT_MAX;
  
  int k = 0, l = 0, viewport = 0;
  // Load the data files
  pcl::PCDReader pcd;
  pcl::console::TicToc tt;
  ColorHandlerPtr color_handler;
  GeometryHandlerPtr geometry_handler;
  
  
  
  pcl::PCLPointCloud2::Ptr cloud, _hand_cloud2d;
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud, hand_cloud;
  
  cloud.reset (new pcl::PCLPointCloud2);
  _hand_cloud2d.reset (new pcl::PCLPointCloud2);
  hand_cloud.reset (new pcl::PointCloud<PointXYZ>);
  
  xyzCloud.reset(new pcl::PointCloud<PointXYZ>);
  Eigen::Vector4f origin;
  Eigen::Quaternionf orientation;
  int version;
  
  if (pcd.read (argv[p_file_indices.at (1)], *cloud, origin, orientation, version) < 0)
    return (-1);
//  if (pcd.read ("hand_cluster.pcd", *cloud, origin, orientation, version) < 0)
//    return (-1);
//  if (pcd.read ("hand_cluster.pcd", *_hand_cloud2d, origin, orientation, version) < 0)
//    return (-1);
  
  if (!loadCloud (argv[3], *_hand_cloud2d))
    return (-1);
  
  tt.tic ();
  
  print_highlight (stderr, "Loading "); print_value (stderr, "%s ", argv[p_file_indices.at (0)]);
  
  std::stringstream cloud_name;
  
  cloud_name << argv[p_file_indices.at (0)];
  
  // Create the PCLVisualizer object here on the first encountered XYZ file
  if (!p)
  {
    p.reset (new pcl::visualization::PCLVisualizer (argc, argv, "PCD viewer"));
    if (use_pp)   // Only enable the point picking callback if the command line parameter is enabled
      p->registerPointPickingCallback (&pp_callback, static_cast<void*> (&cloud));
    
    // Set whether or not we should be using the vtkVertexBufferObjectMapper
    p->setUseVbos (use_vbos);
    
    if (!p->cameraParamsSet () && !p->cameraFileLoaded ())
    {
      Eigen::Matrix3f rotation;
      rotation = orientation;
      p->setCameraPosition (origin [0]                  , origin [1]                  , origin [2],
                            origin [0] + rotation (0, 2), origin [1] + rotation (1, 2), origin [2] + rotation (2, 2),
                            rotation (0, 1),              rotation (1, 1),              rotation (2, 1));
    }
  }

  
  if (cloud->width * cloud->height == 0)
  {
    print_error ("[error: no points found!]\n");
    return (-1);
  }
  
  // If no color was given, get random colors
  double red, blue, green;
  red = 200.0;
  blue = 0.0;
  green = 0.0;
  color_handler.reset (new pcl::visualization::PointCloudColorHandlerCustom<pcl::PCLPointCloud2> (cloud, red, blue, green));
  
  // Add the dataset with a XYZ and a random handler
  geometry_handler.reset (new pcl::visualization::PointCloudGeometryHandlerXYZ<pcl::PCLPointCloud2> (cloud));
  // Add the cloud to the renderer
  //p->addPointCloud<pcl::PointXYZ> (cloud_xyz, geometry_handler, color_handler, cloud_name.str (), viewport);
  pcl::fromPCLPointCloud2 (*cloud, *xyzCloud);
  pcl::fromPCLPointCloud2 (*_hand_cloud2d, *hand_cloud);
  *xyzCloud += *hand_cloud;

  float voxelSize = 0.0025;
  float sampleSize = 100 - 2 * 25;
  
  float stdDev = 1.0;
  float iterationDivisor = 1 + 1.0;
  //float iterationDivisor = 1.0;
  boost::posix_time::time_duration runTime;
  auto startTime = boost::posix_time::microsec_clock::local_time();
  
  SegmentationPipeline pipeline(xyzCloud,
                                voxelSize,
                                sampleSize,
                                stdDev,
                                iterationDivisor);
  if (pipeline.performSegmentation()){
    runTime = boost::posix_time::microsec_clock::local_time() - startTime;
    
    auto objects = pipeline.getObjects();
    for (int i = 0; i < objects.size(); i++){
      auto desc = objects[i].computeVFHDescriptor();
      
      cloud_name << " cluster " << i;
      
      int r,g,b;
      if (i == 0){
        r = 255, g = 0, b = 0;
        //pcl::io::savePCDFileASCII ("hand_cluster.pcd", *cluster);
      } else if (i == 1){
        r = 0, g = 255, b = 0;
      } else if (i == 2){
        r = 0, g = 0, b = 255;
      } else {
        g = 255, b = 255, r = 0;
      }
      
      visualization::PointCloudColorHandlerCustom<PointXYZ>
      _color_handler(objects[i].point_cloud, r, g, b);
      p->addPointCloud (objects[i].point_cloud,
                        _color_handler,
                        cloud_name.str(), viewport);
      
      
    }

    
    for (int i = 0; i < pipeline.graspableObjects.size(); i++){
      auto obj = pipeline.graspableObjects[i];
      
      const auto bbox = obj.getBoundingBox();
      auto centroid = bbox.centroid;
      
      
      Eigen::Translation3f translation(centroid.x(),
                                       centroid.y(),
                                       centroid.z());
      Eigen::AngleAxisf rotation(bbox.rotational_matrix_OBB);
      
      
      std::stringstream boxName;
      
      boxName << "oriented bounding box" ;
      
      
      MomentOfInertiaEstimation<PointXYZ> feature_extractor;
      feature_extractor.setInputCloud (pipeline.getConvexHull());
      feature_extractor.compute();
      
      pcl::PointXYZ min_point_OBB,
      max_point_OBB,
      position_OBB;
      
      Eigen::Matrix3f rotational_matrix_OBB;
      
      
      feature_extractor.getOBB (min_point_OBB,
                                max_point_OBB,
                                position_OBB,
                                rotational_matrix_OBB);
      
      Vector3f position(position_OBB.x,
                        position_OBB.y,
                        position_OBB.z);
      
      Quaternionf hullOrientation(rotational_matrix_OBB);
      boxName.clear();
      boxName << "convex hull";
      p->addCube(position,
                 hullOrientation,
                 max_point_OBB.x - min_point_OBB.x,
                 max_point_OBB.y - min_point_OBB.y,
                 max_point_OBB.z - min_point_OBB.z,
                 boxName.str(),
                 viewport);
      
      
      
    }
  } else {
    print_error("could not find plane, even at max accuracy. aborting");
    return -1;
  }
  
  
  cloud.reset (new pcl::PCLPointCloud2);
  xyzCloud.reset(new pcl::PointCloud<PointXYZ>);
  



  if (p)
    p->setBackgroundColor (bcolor[0], bcolor[1], bcolor[2]);
  // Read axes settings
  double axes  = 0.0;
  pcl::console::parse_argument (argc, argv, "-ax", axes);
  if (axes != 0.0 && p)
  {
    float ax_x = 0.0, ax_y = 0.0, ax_z = 0.0;
    pcl::console::parse_3x_arguments (argc, argv, "-ax_pos", ax_x, ax_y, ax_z);
    // Draw XYZ axes if command-line enabled
    p->addCoordinateSystem (axes, ax_x, ax_y, ax_z, "global");
  }

  // Clean up the memory used by the binary blob
  // Note: avoid resetting the cloud, otherwise the PointPicking callback will fail
  if (!use_pp)   // Only enable the point picking callback if the command line parameter is enabled
  {
    cloud.reset ();
    xyzcloud.reset ();
  }

  // If we have been given images, create our own loop so that we can spin each individually
  if (!imgs.empty ())
  {
    bool stopped = false;
    do
    {
  #if VTK_MAJOR_VERSION>=6 || (VTK_MAJOR_VERSION==5 && VTK_MINOR_VERSION>6)
      if (ph) ph->spinOnce ();
  #endif
        
        for (int i = 0; i < int (imgs.size ()); ++i)
        {
          if (imgs[i]->wasStopped ())
          {
            stopped = true;
            break;
          }
          imgs[i]->spinOnce ();
        }
      
      if (p)
      {
        if (p->wasStopped ())
        {
          stopped = true;
          break;
        }
        p->spinOnce ();
      }
      boost::this_thread::sleep (boost::posix_time::microseconds (100));
    }
    while (!stopped);
  }
  else
  {
    // If no images, continue
  #if VTK_MAJOR_VERSION>=6 || (VTK_MAJOR_VERSION==5 && VTK_MINOR_VERSION>6)
    if (ph)
    {
      //print_highlight ("Setting the global Y range for all histograms to: "); print_value ("%f -> %f\n", min_p, max_p);
      //ph->setGlobalYRange (min_p, max_p);
      //ph->updateWindowPositions ();
      if (p)
        p->spin ();
        else
          ph->spin ();
          }
    else
  #endif
      if (p)
        p->spin ();
  }
}
/* ]--- */
