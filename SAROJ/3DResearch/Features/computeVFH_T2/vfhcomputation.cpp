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

#define VOXEL_FILTER
//#define NORMAL_COMPUTE
#define PASSTHROUGH_FILTER
//#define VFH_DESCRIPTOR

int main()
{
	//Load Point Cloud from a file
	// The file should be in the same folder
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("milk_cartoon_all_small_clorox.pcd", *cloud);

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

		std::cerr << "PointCloud after filtering: " << cloud_voxal_filtered->width * cloud_voxal_filtered->height 
      	 << " data points (" << pcl::getFieldsList (*cloud_voxal_filtered) << ").";
      	
      	// Save to a file
      //	 pcl::io::savePCDFileASCII ("milk_cartoon_downsampled.pcd",*cloud_voxal_filtered);
	/*	 pcl::PCDWriter writer;
 		 writer.write ("milk_cartoon_downsampled.pcd", *cloud_filtered,Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);
*/
/*
		//visualize Filtered Data
		pcl::visualization::CloudViewer viewer("PCL Viewer");
		
		viewer.showCloud(cloud_filtered);
		while(!viewer.wasStopped())
		{
			
		}
*/
	#endif	

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

  	// we create the PassThrough filter object, and set its parameters.
	 	pcl::PassThrough<pcl::PointXYZ> pass;
 	 	pass.setInputCloud (cloud_voxal_filtered);
 		pass.setFilterFieldName ("z");
  		pass.setFilterLimits (0.0, 1.0);
 	    pass.setFilterLimitsNegative (true);
 		pass.filter (*cloud_pass_filtered);

    #endif
     //================================================================================//
	 //POINT CLOUD---> Flitering---> Normals    
      //a) 
     //===============================================================================//

	#ifdef NORMAL_COMPUTE	
		//Create an object for the normal estimation and 
		//compute the normals
	//	pcl::io::loadPCDFile("milk_cartoon_downsampled.pcd", *cloud);
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		// The AVERAGE_3D_GRADIENT mode creates 6 integral images to 
		//compute the normals using the cross-product of horizontal and
		//vertical 3D gradients and computes the normals using the cross-
		//product between these two gradients.

		pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
             ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
             ne.setMaxDepthChangeFactor(0.02f);
             ne.setNormalSmoothingSize(10.0f);
             ne.setInputCloud(cloud_pass_filtered);   		// Load the cloud
             ne.compute(*normals);				// Compute Normal
         //visualize normals
		pcl::visualization::PCLVisualizer viewer("PCL Viewer");
		viewer.setBackgroundColor(0.0,0.0,0.5);
		viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud,normals);
		while(!viewer.wasStopped())
		{
			viewer.spinOnce();
		} 
    #endif
    //=====================================================================================//  
      //POINT CLOUD---> Flitering---> Normals----> VFH Extraction    
      //a) 
     //===============================================================================//
     #ifdef VFH_DESCRIPTOR
   		 // Now the normal is computed, we are ready to use VFH
    	 pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
   		 vfh.setInputCloud(cloud);
     	 vfh.setInputNormals(normals);

    	 // Create an empty kdtree representation, and pass it to the FPFH
    	 //estimation object.
    	 //Its content will be filled inside the object, based on the given input dataset

    	 pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>());
    	 vfh.setSearchMethod(tree);

     	//Output datasets
     	// Create an empty PointCloud representation, and pass it to vfh
     	 pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs(new pcl::PointCloud<pcl::VFHSignature308>());

   		 // Normalize bins
     	vfh.setNormalizeBins(true);
    	vfh.setNormalizeDistance(false);
    	// Compute the features
     	 vfh.compute(*vfhs);
     /*
     	//VFH Visualization.
		pcl::visualization::PCLHistogramVisualizer viewer;

		// Set the size of the descriptor beforehand
		viewer.addFeatureHistogram(*vfhs, 308);

		viewer.spin(); */
	#endif
	return 0;
}