#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/vfh.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>     //Visualization class 1
#include <pcl/visualization/pcl_visualizer.h>   // Visualization class 2
#include <pcl/visualization/histogram_visualizer.h> // histogram visualizer
#include <pcl/filters/voxel_grid.h>  //For voxel grid
#include <pcl/filters/passthrough.h> // For passthrough filter
#include <pcl/console/parse.h>       // To parse Console
#include <pcl/filters/statistical_outlier_removal.h>  // Statistical Filtering
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>  //Planar removal
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/fpfh.h>	//FPFH Features

// Predefined 
//#define PASSTHROUGH_FILTER
//#define VOXEL_FILTER
//#define OUTLIER_REMOVAL   
//#define NORMAL_COMPUTE
#define GENERAL_NORMAL_COMPUTE
//#define PLANE_REMOVAL
#define VFH_DESCRIPTOR
#define FPFH_FEATURES

std::string modelFileName;
//std::string sceneFileName;

// enable variables for time logging
 boost::posix_time::ptime time_before_execution;
 boost::posix_time::ptime time_after_execution;
 boost::posix_time::time_duration difference;

//-------------------------------------------------------
// Command line parser 
void 
parseCommandLine (int argc, char *argv[])
{
	// Model & Scene Filenames
	std::vector<int> filenames;
	filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");
	if(filenames.size() !=1)  // For  files set the value to 2
	{
		std::cout << "Filenames missing \n";
		std::cout << "./executable model.pcd scene.pdc \n";
		exit(-1);
	}

	// Store the filenames in temp variables
	modelFileName = argv[filenames[0]];
	//sceneFileName = argv[filenames[1]];
}


int main( int argc, char *argv[])
{ 
	//=================================================================
	// Create a cloud pointer
	// The file should be in the same folder
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud0 (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//=================================================================
 	// Parse Command Line
 	parseCommandLine (argc, argv);
 	// Load the file
	if(pcl::io::loadPCDFile(modelFileName, *cloud0) < 0)
	{
		std::cout << "Error Loading model cloud." <<std::endl;
		return(-1);
	}
	cloud = cloud0;
//================================================================================//
	 // It is very important to run pass through filter. it cuts off values 
	 // that are either inside or outside a given user range.
	 // I could not computer Normals directly from Voxal Filter.. As was suggested
	// in community post, this step was essential.
	 //POINT CLOUD---> Flitering---> Pass through filter   
      //a) 
//===============================================================================//
	#ifdef PASSTHROUGH_FILTER	
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass_filtered (new pcl::PointCloud<pcl::PointXYZ>);
		std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ").";

  	// we create the PassThrough filter object, and set its parameters.
	 	pcl::PassThrough<pcl::PointXYZ> pass;
 	 //	pass.setInputCloud (cloud_voxal_filtered);
	 	pass.setInputCloud (cloud);
 	
 		pass.setFilterFieldName ("z");
  		pass.setFilterLimits (-0.01, 0.7); 
 		/* For model arm
 		pass.setFilterFieldName ("x");
  		pass.setFilterLimits (-0.01, 1); */
 	   // pass.setFilterLimitsNegative (false);
 		pass.filter (*cloud_pass_filtered);

 		cloud = cloud_pass_filtered;   // Set the new cloud 
/*
		//visualize Filtered Data
		pcl::visualization::CloudViewer viewer("PCL Viewer");
		
		viewer.showCloud(cloud);
		while(!viewer.wasStopped(cloud))
		{
			
		}  */
    #endif
 	
//==========================================================================
      //POINT CLOUD---> Flitering    
      //a) Voxal Grid  
     //===============================================================================//
      // Create a cloud for voxel filter
    #ifdef VOXEL_FILTER
		std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ").";

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxal_filtered (new pcl::PointCloud<pcl::PointXYZ>);

		// a pcl::VoxelGrid filter is created with a leaf size of 1cm,
		// the input data is passed, and the output is computed and stored in cloud_filtered.
		pcl::VoxelGrid<pcl::PointXYZ>sor;
		sor.setInputCloud(cloud);
		sor.setLeafSize(0.01f, 0.01f, 0.01f);
		sor.filter(*cloud_voxal_filtered);

		std::cerr << "PointCloud after filtering: " << cloud_voxal_filtered->width *cloud_voxal_filtered->height 
      	 << " data points (" << pcl::getFieldsList (*cloud_voxal_filtered) << ").";
      	
      	// Set cloud = new cloud voxal filtered
      	 cloud = cloud_voxal_filtered;  // Indirect Reference
      	// Save to a file
    //	 pcl::io::savePCDFileASCII ("downsampledModel2.pcd",*cloud_voxal_filtered);
		
	
	/*
		//visualize Filtered Data
		pcl::visualization::CloudViewer viewer("PCL Viewer");
		
		viewer.showCloud(cloud);
		while(!viewer.wasStopped())
		{
			
		}  */

	#endif	

    //=============================================================
	// Remove the plane
	//===========================================================
	#ifdef PLANE_REMOVAL
			 // Create the segmentation object for the planar model and set all the parameters
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
		cloud_filtered = cloud;  // Set the values in these pointers equal
		pcl::PCDWriter writer;
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
		seg.setOptimizeCoefficients (true);
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setMaxIterations (100);
		seg.setDistanceThreshold (0.5);  // How much close they have to be to consider inlier (z-axis value) From output file

		int i=0, nr_points = (int) cloud_filtered->points.size ();
		std::cout << "PointCloud representing the planar component: " << cloud_filtered->points.size () << " data points." << std::endl;

		// When 30% of the cloud are still there
		while (cloud_filtered->points.size () > 0.1 * nr_points)
		{
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud (cloud_filtered);
		seg.segment (*inliers, *coefficients);
		if (inliers->indices.size () == 0)
		{
		  std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
		  break;
		}

		// Extract the planar inliers from the input cloud
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud (cloud_filtered);
		extract.setIndices (inliers);
		extract.setNegative (false);

		// Get the points associated with the planar surface
		extract.filter (*cloud_plane);
		std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

		// Remove the planar inliers, extract the rest
		extract.setNegative (true);
		extract.filter (*cloud_f);
		*cloud_filtered = *cloud_f;  // Removing the value
		} 
		/*
		// Following this is the cluster extraction part. It is very handy to see the size and nature of object
		  // Creating the KdTree object for the search method of the extraction
  		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  		tree->setInputCloud (cloud_filtered);

		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		ec.setClusterTolerance (0.02); // 2cm
		ec.setMinClusterSize (50);
		ec.setMaxClusterSize (25000);
		ec.setSearchMethod (tree);
		ec.setInputCloud (cloud_filtered);
		ec.extract (cluster_indices);

		int j = 0;
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
		{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
		  cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
		std::stringstream ss;
		ss << "cloud_cluster_" << j << ".pcd";
		writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
		j++;
		} */


		// pcl::io::savePCDFileASCII ("removedPlane2.pcd",*cloud_filtered);
		
	
	
		//visualize Filtered Data
		pcl::visualization::CloudViewer viewer("PCL Viewer");
		
		viewer.showCloud(cloud_filtered);
		while(!viewer.wasStopped())
		{
			
		} 
	#endif
     //================================================================================//
	 //POINT CLOUD---> Flitering---> Normals    
      //a) 
     //===============================================================================//
	#ifdef NORMAL_COMPUTE	
		//Create an object for the normal estimation and 
		//compute the normals
	//	pcl::io::loadPCDFile("milk_cartoon_downsampled.pcd", *cloud);
		std::cout << "Computing Normals" << std::endl;
  		
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

		// The AVERAGE_3D_GRADIENT mode creates 6 integral images to 
		//compute the normals using the cross-product of horizontal and
		//vertical 3D gradients and computes the normals using the cross-
		//product between these two gradients.

		pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

		time_before_execution = boost::posix_time::microsec_clock::local_time();  // time before
	     ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
	     ne.setMaxDepthChangeFactor(0.02f);
	     ne.setNormalSmoothingSize(10.0f);
	     ne.setInputCloud(cloud);   		// Load the cloud
	     ne.compute(*normals);				// Compute Normal
	
		// Time after calculation
		time_after_execution = boost::posix_time::microsec_clock::local_time(); // time after
		difference = time_after_execution - time_before_execution;  // get execution time
		std::cout << std::setw(5) << difference.total_milliseconds() << ": "
              << "Normal Computation "<< std::endl;

        /*
         //visualize normals
		pcl::visualization::PCLVisualizer viewer("PCL Viewer");
		viewer.setBackgroundColor(0.0,0.0,0.5);
		viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud,normals);
		while(!viewer.wasStopped())
		{
			viewer.spinOnce();
		}  */
    #endif
    //=====================================================================================//  
      //================================================================================//
	 // This normal estimation is for all images--not necessarily organized
     //===============================================================================//
	#ifdef GENERAL_NORMAL_COMPUTE	
		// Output dataset
		pcl::PointCloud <pcl::Normal>::Ptr gnormals (new pcl::PointCloud <pcl::Normal>);
		// Create the normal estimation class, and pass the input dataset to it
		pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> gne;

		// Timing calculation
		std::cout << "Computing General Normals" << std::endl;
  		time_before_execution = boost::posix_time::microsec_clock::local_time();  // time before

		gne.setInputCloud (cloud);
        // Create an empty kdtree representation, and pass it to the normal estimation object.
  		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  		pcl::search::Search<pcl::PointXYZ>::Ptr gtree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
  		
  		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  		normal_estimator.setSearchMethod (gtree);
  		normal_estimator.setInputCloud (cloud);
  		normal_estimator.setKSearch (50);
  		normal_estimator.compute (*gnormals);

		// Time after calculation
		time_after_execution = boost::posix_time::microsec_clock::local_time(); // time after
		difference = time_after_execution - time_before_execution;  // get execution time
		std::cout << std::setw(5) << difference.total_milliseconds() << ": "
              << " General Normal Computation "<< std::endl;

        
 	//	pcl::io::savePCDFileASCII ("gnormals.pcd",*gnormals);
     /*	
     	 //visualize normals
		pcl::visualization::PCLVisualizer viewer("PCL Viewer");
		viewer.setBackgroundColor(0.0,0.0,0.5);
		viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud,normals);
		while(!viewer.wasStopped())
		{
			viewer.spinOnce();
		}  */
    #endif
    //=====================================================================================//      //POINT CLOUD---> Flitering---> Normals----> VFH Extraction    
      //a) 
     //===============================================================================//
     #ifdef VFH_DESCRIPTOR
   		 // Now the normal is computed, we are ready to use VFH
    	 pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
    	 // Timing calculation
		 std::cout << "Computing VFH" << std::endl;
  		 time_before_execution = boost::posix_time::microsec_clock::local_time();  // time before
   		 vfh.setInputCloud(cloud);
     	 vfh.setInputNormals(gnormals);

    	 // Create an empty kdtree representation, and pass it to the FPFH
    	 //estimation object.
    	 //Its content will be filled inside the object, based on the given input dataset

    	// pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>());
    	 vfh.setSearchMethod(gtree);   // Using the last tree

     	//Output datasets
     	// Create an empty PointCloud representation, and pass it to vfh
     	 pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs(new pcl::PointCloud<pcl::VFHSignature308>());

   		 // Normalize bins
     	vfh.setNormalizeBins(true);
    	vfh.setNormalizeDistance(false);
    	// Compute the features
     	 vfh.compute(*vfhs);

     	 // Time after calculation
		time_after_execution = boost::posix_time::microsec_clock::local_time(); // time after
		difference = time_after_execution - time_before_execution;  // get execution time
		std::cout << std::setw(5) << difference.total_milliseconds() << ": "
              << " VFH Computation "<< std::endl;
     /*
     	//VFH Visualization.
		pcl::visualization::PCLHistogramVisualizer viewer;

		// Set the size of the descriptor beforehand
		viewer.addFeatureHistogram(*vfhs, 308);

		viewer.spin(); */
	#endif
	//===============================================================================//
	#ifdef FPFH_FEATURES
		// Create the FPFH estimation class, and pass the input dataset+normals to it
		pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
		// Timing calculation
		std::cout << "Computing FPFH" << std::endl;
  		time_before_execution = boost::posix_time::microsec_clock::local_time();  // time before

		fpfh.setInputCloud (cloud);
		fpfh.setInputNormals (gnormals);
		// alternatively, if cloud is of tpe PointNormal, do fpfh.setInputNormals (cloud);

		fpfh.setSearchMethod (gtree);  // Reuse the same KD-Tree

		// Output datasets
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());

		// Use all neighbors in a sphere of radius 1cm
		// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
		fpfh.setRadiusSearch (0.01);

		// Compute the features
		fpfh.compute (*fpfhs);

		// Time after calculation
		time_after_execution = boost::posix_time::microsec_clock::local_time(); // time after
		difference = time_after_execution - time_before_execution;  // get execution time
		std::cout << std::setw(5) << difference.total_milliseconds() << ": "
              << " FPFH Computation "<< std::endl;


         /*
        //VISUALIZE
		//VFH Visualization.
		pcl::visualization::PCLHistogramVisualizer viewer;

		// Set the size of the descriptor beforehand
		viewer.addFeatureHistogram(*fpfhs, 308);

		viewer.spin(); */

	#endif  

//====================================================================================================

	return 0;
}