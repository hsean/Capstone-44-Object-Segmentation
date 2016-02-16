// c44_pcd_viewer.cpp
// a slightly simplified/customized version of the pcl_viewer.cpp example code
// from the PCL repo.

// PCL
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <cfloat>
#include <pcl/visualization/eigen.h>
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
//#include <pcl/filters/voxel_grid.h>
#include "segmentation_pipeline.hpp"

using namespace pcl::console;
using namespace C44;

typedef pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2> ColorHandler;
typedef ColorHandler::Ptr ColorHandlerPtr;
typedef ColorHandler::ConstPtr ColorHandlerConstPtr;

typedef pcl::visualization::PointCloudGeometryHandler<pcl::PCLPointCloud2> GeometryHandler;
typedef GeometryHandler::Ptr GeometryHandlerPtr;
typedef GeometryHandler::ConstPtr GeometryHandlerConstPtr;

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



// Global visualizer object
#if VTK_MAJOR_VERSION>=6 || (VTK_MAJOR_VERSION==5 && VTK_MINOR_VERSION>6)
pcl::visualization::PCLPlotter ph_global;
#endif
boost::shared_ptr<pcl::visualization::PCLVisualizer> p;
std::vector<boost::shared_ptr<pcl::visualization::ImageViewer> > imgs;
pcl::search::KdTree<pcl::PointXYZ> search_;
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
		search_.setInputCloud (xyzcloud);
	}
	// Return the correct index in the cloud instead of the index on the screen
	std::vector<int> indices (1);
	std::vector<float> distances (1);
	
	// Because VTK/OpenGL stores data without NaN, we lose the 1-1 correspondence, so we must search for the real point
	pcl::PointXYZ picked_pt;
	event.getPoint (picked_pt.x, picked_pt.y, picked_pt.z);
	search_.nearestKSearch (picked_pt, 1, indices, distances);
	
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
	print_info ("The viewer window provides interactive commands; for help, press 'h' or 'H' from within the window.\n");
	//doStuff(argv[1]);

	
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
	
	// Multiview enabled?
	int y_s = 0, x_s = 0;
	double x_step = 0, y_step = 0;
	
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
	// Us ing min_p, max_p to set the global Y min/max range for the histogram
	float min_p = FLT_MAX; float max_p = -FLT_MAX;
	
	int k = 0, l = 0, viewport = 0;

	pcl::PCDReader pcd;
	pcl::console::TicToc tt;
	ColorHandlerPtr color_handler;
	GeometryHandlerPtr geometry_handler;
	

  PCLPointCloud2::Ptr cloud;
  Cloud3D::Ptr xyzCloud;
  
	
	// Go through PCD files
	for (size_t i = 0; i < p_file_indices.size (); ++i)
	{
		tt.tic ();
    cloud.reset (new PCLPointCloud2);
    xyzCloud.reset(new Cloud3D);

		Eigen::Vector4f origin;
		Eigen::Quaternionf orientation;
		int version;
		
		print_highlight (stderr, "Loading "); print_value (stderr, "%s ", argv[p_file_indices.at (i)]);
		
		if (pcd.read (argv[p_file_indices.at (i)], *cloud, origin, orientation, version) < 0)
			return (-1);
    fromPCLPointCloud2(*cloud, *xyzCloud);
		

		
		// Calculate transform if available.
		if (pose_x.size () > i && pose_y.size () > i && pose_z.size () > i &&
				pose_roll.size () > i && pose_pitch.size () > i && pose_yaw.size () > i)
		{
			Eigen::Affine3f pose =
			Eigen::Translation3f (Eigen::Vector3f (pose_x[i], pose_y[i], pose_z[i])) *
			Eigen::AngleAxisf (pose_yaw[i],   Eigen::Vector3f::UnitZ ()) *
			Eigen::AngleAxisf (pose_pitch[i], Eigen::Vector3f::UnitY ()) *
			Eigen::AngleAxisf (pose_roll[i],  Eigen::Vector3f::UnitX ());
			orientation = pose.rotation () * orientation;
			origin.block<3, 1> (0, 0) = (pose * Eigen::Translation3f (origin.block<3, 1> (0, 0))).translation ();
		}
		
		std::stringstream cloud_name;
		

		

		
		cloud_name << argv[p_file_indices.at (i)] << "-" << i;
		
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
    
    
		SegmentationPipeline pipeline(xyzCloud);
    if (pipeline.performSegmentation()){
      for (int i = 0; i < pipeline.graspableObjects.size(); i++){
        auto obj = pipeline.graspableObjects[i];
        
        
        const auto bbox = obj.getBoundingBox();
        
        Vector3f position(bbox.position_OBB.x, bbox.
                          position_OBB.y,
                          bbox.position_OBB.z);
        Quaternionf objOrientation(bbox.rotational_matrix_OBB);
        
//        p->addCube(position,
//                   objOrientation,
//                   bbox.max_point_OBB.x - bbox.min_point_OBB.x,
//                   bbox.max_point_OBB.y - bbox.min_point_OBB.y,
//                   bbox.max_point_OBB.z - bbox.min_point_OBB.z, "OBB");
        //
        std::stringstream objName;
        objName << "cylinder " << i;
        p->addCylinder(obj.coefficients,objName.str(),viewport);
        
      }
    } else {
      print_error("could not find plane. aborting");
      return -1;      
    }
		
		// Multiview enabled?
		if (mview)
		{
			p->createViewPort (k * x_step, l * y_step, (k + 1) * x_step, (l + 1) * y_step, viewport);
			k++;
			if (k >= x_s)
			{
				k = 0;
				l++;
			}
		}
		
		if (cloud->width * cloud->height == 0)
		{
			print_error ("[error: no points found!]\n");
			return (-1);
		}
		
		// If no color was given, get random colors
		if (fcolorparam)
		{
			if (fcolor_r.size () > i && fcolor_g.size () > i && fcolor_b.size () > i)
				color_handler.reset (new pcl::visualization::PointCloudColorHandlerCustom<pcl::PCLPointCloud2> (cloud, fcolor_r[i], fcolor_g[i], fcolor_b[i]));
			else
				color_handler.reset (new pcl::visualization::PointCloudColorHandlerRandom<pcl::PCLPointCloud2> (cloud));
		}
		else
			color_handler.reset (new pcl::visualization::PointCloudColorHandlerRandom<pcl::PCLPointCloud2> (cloud));
		
		// Add the dataset with a XYZ and a random handler
		geometry_handler.reset (new pcl::visualization::PointCloudGeometryHandlerXYZ<pcl::PCLPointCloud2> (cloud));
		p->addPointCloud (cloud, geometry_handler, color_handler, origin, orientation, cloud_name.str (), viewport);
		
		if (mview)
			// Add text with file name
			p->addText (argv[p_file_indices.at (i)], 5, 5, 10, 1.0, 1.0, 1.0, "text_" + std::string (argv[p_file_indices.at (i)]), viewport);
		

    const auto filteredCloud = pipeline.getNoiseFreeCloud();
		
		// Add every dimension as a possible color
		if (!fcolorparam)
		{
			int rgb_idx = 0;
			int label_idx = 0;
			for (size_t f = 0; f < cloud->fields.size (); ++f)
			{
				if (cloud->fields[f].name == "rgb" || cloud->fields[f].name == "rgba")
				{
					rgb_idx = f + 1;
					color_handler.reset (new pcl::visualization::PointCloudColorHandlerRGBField<pcl::PCLPointCloud2> (cloud));
				}
				else if (cloud->fields[f].name == "label")
				{
					label_idx = f + 1;
					color_handler.reset (new pcl::visualization::PointCloudColorHandlerLabelField<pcl::PCLPointCloud2> (cloud, !use_optimal_l_colors));
				}
				else
				{
					if (!isValidFieldName (cloud->fields[f].name))
						continue;
					color_handler.reset (new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PCLPointCloud2> (cloud, cloud->fields[f].name));
				}
				// Add the cloud to the renderer
        //p->addPointCloud<pcl::PointXYZ> (cloud_xyz, color_handler, cloud_name.str (), viewport);
        p->addPointCloud (cloud, color_handler, origin, orientation, cloud_name.str (), viewport);
			}
			// Set RGB color handler or label handler as default
			p->updateColorHandlerIndex (cloud_name.str (), (rgb_idx ? rgb_idx : label_idx));
		}
		
		// Additionally, add normals as a handler
		geometry_handler.reset (new pcl::visualization::PointCloudGeometryHandlerSurfaceNormal<pcl::PCLPointCloud2> (cloud));
		if (geometry_handler->isCapable ())
			//p->addPointCloud<pcl::PointXYZ> (cloud_xyz, geometry_handler, cloud_name.str (), viewport);
			p->addPointCloud (cloud, geometry_handler, origin, orientation, cloud_name.str (), viewport);
		
		if (use_immediate_rendering)
			// Set immediate mode rendering ON
			p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_IMMEDIATE_RENDERING, 1.0, cloud_name.str ());
		
		// Change the cloud rendered point size
		if (psize.size () > 0)
			p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, psize.at (i), cloud_name.str ());
		
		// Change the cloud rendered opacity
		if (opaque.size () > 0)
			p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, opaque.at (i), cloud_name.str ());
		
		// Reset camera viewpoint to center of cloud if camera parameters were not passed manually and this is the first loaded cloud
		if (i == 0 && !p->cameraParamsSet () && !p->cameraFileLoaded ())
		{
			p->resetCameraViewpoint (cloud_name.str ());
			p->resetCamera ();
		}
		
		print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%u", cloud->width * cloud->height); print_info (" points]\n");
		print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList (*cloud).c_str ());
		if (p->cameraFileLoaded ())
			print_info ("Camera parameters restored from %s.\n", p->getCameraFile ().c_str ());
		//delete cylinderCloud;
    
	}
	
	if (!mview && p)
	{
		std::string str;
		if (!p_file_indices.empty ())
			str = std::string (argv[p_file_indices.at (0)]);
		else if (!vtk_file_indices.empty ())
			str = std::string (argv[vtk_file_indices.at (0)]);
		
		for (size_t i = 1; i < p_file_indices.size (); ++i)
			str += ", " + std::string (argv[p_file_indices.at (i)]);
		
		for (size_t i = 1; i < vtk_file_indices.size (); ++i)
			str += ", " + std::string (argv[vtk_file_indices.at (i)]);
		
		p->addText (str, 5, 5, 10, 1.0, 1.0, 1.0, "text_allnames");
   
	}
	
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

			if (ph)
        ph->spinOnce ();
			
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
    else{
			if (p)
				p->spin ();
    }
	}
}
/* ]--- */
