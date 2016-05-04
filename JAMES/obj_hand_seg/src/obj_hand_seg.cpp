/******************************************************************************
* Author: Sean Hendrickson, James Schiffer
* File: demo03.cpp
* Last Modified: 22 April 2016
* Description: Segments out a hand and objects from the predominate plane
*  in a loaded PCD file. It is desgined with porting to a ROS package in mind
*  and therefore follows the same infrastructure.
*
* TODO Implement Hand Detection from 3D Model!
******************************************************************************/
#include "c44_pipes.h"
#include "c44_geometry.hpp"
#include <string.h>

#define EUCLIDEAN_CLUSTER_TEST

#define LINUX
#define DEBUG
//#define DEBUG_PLANE
#define SAVE_PCD
#define Z_CUTOFF 2.5     // 1 meter from camera

// To make it easier to switch between PointXYZ and PointXYZRGB
typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud3D;

///////////////////////////////////////////////////////////////////////////////
/**
 * desc:  In ROS, we need a call back function that does the work. In an effort
 *          to reduce integration cost, please add the main pipeline for hand
 *          segmentation inside of here.
 * param: (in) msg_cloud - PointCloud PTR that represents the incoming message
 *        (in) filePath - the absolute or relative file to the pcd file -- Non-ROS only
 *
 */
bool ros_callback(
        const Cloud3D::Ptr msg_cloud,
        std::string filePath
        );

///////////////////////////////////////////////////////////////////////////////
/**
 * desc:  This function takes a file path and a target extension, and will
 *         return the file path sans the extension if the file extension
 *         matches the target.
 * param: (in) filePath - absolute or relative file path with extension
 *        (in) desiredExt - The target file extension
 *        (out) fileNoExt - The file path without the extension, "" if ext bad.
 */
std::string extractFileName(
        const std::string& filePath,
        const std::string& desiredExt
        );


int main(int argc, char** argv)
{
	// initialize variables
    Cloud3D::Ptr cloud_original (new Cloud3D);
    pcl::PCDReader reader;
    //	pcl::PCDWriter writer;
//	int result = 0;  // catch function return


	// read point cloud through command line
    if (argc != 2)
	{
        std::cout << "usage: " << argv[0] << " <filename>.pcd" << std::endl;
		return 0;
	}

    // read cloud and display its size
    std::string file_ame(argv[1]);
    std::cout << "Reading Point Cloud" << std::endl;
    reader.read(file_ame, *cloud_original);

#ifdef DEBUG
    std::cout << "size of original cloud: "
              << cloud_original->points.size() << " points" << std::endl;
#endif

    if(!ros_callback(cloud_original, file_ame))
    {
        std::cout << "Could not segment objects" << std::endl;
        return 0;
    }

	return 0;
}


bool ros_callback(
        const Cloud3D::Ptr cloud_original,
        std::string file_path
        )
{

    // enable variables for time logging
    boost::posix_time::ptime time_before_execution;
    boost::posix_time::ptime time_after_execution;
    boost::posix_time::time_duration difference;


    // SET UP VIZUALIZER - NON-ROS
    // setup viewer?
    std::string cloudName = "cloud original";
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_original);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud_original, rgb, cloudName);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, cloudName);
    viewer->initCameraParameters ();

    // get time before execution, cutting out the time it takes to open the file and initialize the viewer
    time_before_execution = boost::posix_time::microsec_clock::local_time();  // time before

    // delcare variables for storing the cloud object,
    Cloud3D::Ptr cloud_inside_prism (new Cloud3D);
    int result = 0;  // catch function return
    bool success = false;


    pcl::PCDWriter writer;

    // check for valid file
    std::string pcd_ext_str = ".pcd";
    std::cout << "recieved file path: " << file_path << std::endl;
    std::string file_no_ext = extractFileName(file_path, pcd_ext_str);
    if(file_no_ext == "")
    {
        std::cout << "usage: " << "hand_seg" << " <filename>" << std::endl;
        return success;
    }

    // pcl::fromROSMsg (*cloud_msg, *cloud_original);

    float leafSize = 0.02;
    // downsample source PCD image
    c44_pipes::voxelFilter(cloud_original, cloud_original, leafSize);  // call voxel filter

    // settings - should read these in through a configuration process - Maybe XML?
    int neighbors_to_analyze = 20;
    double std_deviation = 1.0;
    double min_angle = 0;
    double max_angle = 1.57;
    int ransac_iterations = 1000;
    double threshold_distance = 0.01;
    double min_object_distance = 0.02;
    double max_object_distance = 0.3;
    double cluster_tolerance = 0.02;
    int min_cluster_size = 10;
    int max_cluster_size = 1000;

    // seperate prism on top of plane into a new cloud (that hopefully contains the objects)
    result = c44_pipes::getPrism(
                cloud_original,
                cloud_inside_prism,
                min_angle,
                max_angle,
                ransac_iterations,
                threshold_distance,
                min_object_distance,
                max_object_distance
                );


    // check for errors from getPrism()
    if(result < 0)
    {
        std::cout << "No plane found in getPrism" << std::endl;
        //pub.publish(cloud_original);
    }
    else
    {
#ifdef SAVE_PCD
        std::stringstream obj_file_name;
        obj_file_name << file_no_ext << "_cloud_objects.pcd";
        writer.write<pcl::PointXYZRGB> (obj_file_name.str (), *cloud_inside_prism, false);
#endif

        // remove noise from objects PCD using gaussian filter
        c44_pipes::removeNoise(
                    cloud_inside_prism,
                    cloud_inside_prism,
                    neighbors_to_analyze,
                    std_deviation
                    );
        std::vector<pcl::PointIndices> cluster_indices;

        c44_pipes::clusterExtraction(
                    cloud_inside_prism,
                    &cluster_indices,
                    cluster_tolerance,
                    min_cluster_size,
                    max_cluster_size
                    );

        // Look through each cluster for an object and a hand
        int j = 0;
        std::vector<pcl::PointIndices>::const_iterator obj_clusters_index = cluster_indices.begin ();
        while(obj_clusters_index != cluster_indices.end ())
        {
            Cloud3D::Ptr cloud_cluster (new Cloud3D);
            c44_pipes::seperateClusters(
                        cluster_indices,
                        obj_clusters_index,
                        cloud_inside_prism,
                        cloud_cluster
                        );


            // increment it
            ++obj_clusters_index;     // vectors are non-trivial class members, so pre-increment has better performance than it++

            // make a bounding box around the object
            c44Geometry::BoundingBox obj_box(cloud_cluster);


            // get detail about object cluster
            Eigen::Quaternionf quat (obj_box.rotational_matrix_OBB);
            pcl::PointXYZ center (obj_box.centroid (0), obj_box.centroid (1), obj_box.centroid (2));


            // determine if the object can be picked up
            if(obj_box.width > 0.03
                    && obj_box.height > 0.05
                    && obj_box.width < 0.5
                    && obj_box.height < 0.5)
            {
                std::cout << "Found pick upable object at: {" \
                          << center.x << " , "
                          << center.y << " , "
                          << center.z <<  "}" << std::endl;
                // pub.publish(cloud_cluster);

                success = true;
                // write out cluster as a PCD file
                std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
                std::stringstream ss;
                ss << file_no_ext << "_found_obj_" << j << ".pcd";

#ifdef SAVE_PCD
                writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_cluster, false);
#endif
                viewer->addCube (obj_box.centroid,
                                 quat,
                                 obj_box.width,
                                 obj_box.height,
                                 obj_box.depth,
                                 ss.str()
                                 );
                //break;    In ROS, we will selecting the first obj found.
            }
/*TODO      else // if the object fits the dimensions of the hand (too large to pick up)
            {
                // this is where the global 3D object recognition pipline can go.

                // Description Stage
                // see http://robotica.unileon.es/mediawiki/index.php/PCL/OpenNI_tutorial_5:_3D_object_recognition_(pipeline)#Computing_descriptors_2
                // We need to choose the best fitting descriptor for our problem. This will require research and experimentation.

                // Matching Stage
                // see http://robotica.unileon.es/mediawiki/index.php/PCL/OpenNI_tutorial_5:_3D_object_recognition_(pipeline)#Matching_2
                //

                // Alignment Stage
                // see http://robotica.unileon.es/mediawiki/index.php/PCL/OpenNI_tutorial_5:_3D_object_recognition_(pipeline)#Pose_estimation_2

                // ICP Refinement
                // see http://robotica.unileon.es/mediawiki/index.php/PCL/OpenNI_tutorial_5:_3D_object_recognition_(pipeline)#Postprocessing

                // Hypothessis Verification stage
                // see http://pointclouds.org/documentation/tutorials/global_hypothesis_verification.php
                // The robotica tutorial is not finished, they have yet to fill out this section.
                // So we must make do with the pointclouds.org tutorial on this section.


            }
TODO*/
            j++;
        }
    }

    if(!success)
    {
        cout << "no objects found";
    }

    // get time after execution
    time_after_execution = boost::posix_time::microsec_clock::local_time();  // time after

    // print out execution time.
    difference = time_after_execution - time_before_execution;  // get execution time
    std::cout << std::setw(5) << difference.total_milliseconds() << " ms operation time for whole pipline." << std::endl;

    while (!viewer->wasStopped ()){
        viewer->spinOnce (100);
    }

    return success;
}


///////////////////////////////////////////////////////////////////////////////
/**
 * desc:  This function takes a file path and a target extension, and will
 *         return the file path sans the extension if the file extension
 *         matches the target.
 * param: (in) filePath - absolute or relative file path with extension
 *        (in) desiredExt - The target file extension
 *        (out) fileNoExt - The file path without the extension, "" if ext bad.
 */
std::string extractFileName(
        const std::string& file_path,
        const std::string& desired_ext
        )
{

   //char sep = '/';
   char period = '.';

   std::string file_no_ext = "";
   std::string just_file_ext = "";

   //size_t start = s.find_last_of(sep, s.length());
   size_t end = file_path.find_last_of(period);

   if (end != std::string::npos ) {
      file_no_ext = file_path.substr(0, end);
      just_file_ext = file_path.substr(end+1, file_path.length()-1);
   }
   else if(just_file_ext != desired_ext)
   {
       std::cout << "need file with " << desired_ext << "extension, gave " << just_file_ext << "\n";
       return "";
   }
   return file_no_ext;
}

