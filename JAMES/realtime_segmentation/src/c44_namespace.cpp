/**************************************************************************************************
 * Utility Function header for C44 group
 * Gives us a place to experiment and organize different functionality in a easy way.
 * Keep an eye out for potential objects.
 * ***********************************************************************************************/
#include "c44_namespace.hpp"

namespace c44
{
    //----------------------- REALSENSE RELATED ---------------------------------
    int GetRealsenseDevice(rs::device * dev)
    {
        // ==== RS SETUP ====
        // Turn on logging. We can separately enable logging to console or to file, and use different severity filters for each.
        rs::log_to_console(rs::log_severity::warn);
        //rs::log_to_file(rs::log_severity::debug, "librealsense.log");

        // Create a context object. This object owns the handles to all connected realsense devices.
        rs::context ctx;
        printf("There are %d connected RealSense devices.\n", ctx.get_device_count());
        if(ctx.get_device_count() == 0) return EXIT_FAILURE;

        // We will access only a single device, but it is trivial to extend to multiple devices
        dev = ctx.get_device(0);
        printf("\nUsing device 0, an %s\n", dev->get_name());
        printf("    Serial number: %s\n", dev->get_serial());
        printf("    Firmware version: %s\n", dev->get_firmware_version());

        // Configure depth and color to run with the device's preferred settings
        dev->enable_stream(rs::stream::depth, rs::preset::best_quality);
        dev->start();

        return 0;

    }

    void GrabRealsenseFrame(rs::device *dev, float maxDistanceMeters, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
    {
        //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr = cloud.Ptr;

        // Wait for new frame data
        dev->wait_for_frames();

        // Retrieve our images
        const uint16_t * depth_image = (const uint16_t *)dev->get_frame_data(rs::stream::depth);

        // Retrieve camera parameters for mapping between depth and color
        rs::intrinsics depth_intrin = dev->get_stream_intrinsics(rs::stream::depth);
        float scale = dev->get_depth_scale();

        // pcl cloud prep

        //cloud swap is the same as cloud reset?
        cloud.reset(new pcl::PointCloud<pcl::PointXYZ>(depth_intrin.width, depth_intrin.height));
        cloud->is_dense = false;
        cloud->points.resize(depth_intrin.width * depth_intrin.height);
        //float max_distance = 1.5;       // get rid of noisy data that is past 1.5 meters

        // convert to pcl -- MAKE THIS INTO FUNCTION
        for(int dy=0; dy<depth_intrin.height; ++dy)
        {
            for(int dx=0; dx<depth_intrin.width; ++dx)
            {
                // grab our index. this will leave a small number of stragglers untouched
                uint i = dy * depth_intrin.width + dx;

                // Retrieve the 16-bit depth value and map it into a depth in meters
                uint16_t depth_value = depth_image[i];
                float depth_in_meters = depth_value * scale;

                // Map from pixel coordinates in the depth image to real world co-ordinates
                rs::float2 depth_pixel = {(float)dx, (float)dy};
                rs::float3 depth_point = depth_intrin.deproject(depth_pixel, depth_in_meters);

                // Skip over pixels with a depth value of zero, which is used to indicate no data
                static const float nan = std::numeric_limits<float>::quiet_NaN();
                if(depth_value == 0 || depth_point.z > maxDistanceMeters)
                {
                    cloud->points[i].x = cloud->points[i].y = cloud->points[i].z = (float) nan;
                    continue;
                }
                else // write the valid data to the PCD format
                {
                    cloud->points[i].x = depth_point.x;    // seems to orientate better this way
                    cloud->points[i].y = -depth_point.y;
                    cloud->points[i].z = depth_point.z;
                }
            }
        }
        //pcl::io::savePCDFile("in_function.pcd", *cloud);

        //return cloud;
    }


    //--------------------- VIEWER RELATED --------------------------------------------

    void AddCloudToVisualizer(
                                pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                                boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
                                double red,
                                double green,
                                double blue,
                                std::string cloudName
                               )
    {
      // --------------------------------------------
      // -----Open 3D viewer and add point cloud-----
      // --------------------------------------------
      //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
      viewer->setBackgroundColor (0, 0, 0);
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, red, green, blue);
      viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, cloudName);
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, cloudName);
      //viewer->addCoordinateSystem (1.0);
      viewer->initCameraParameters ();
      //return (viewer);
    }

    void overwriteVis (
                        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
                        double red,
                        double green,
                        double blue
                      )
    {
      // --------------------------------------------
      // -----Open 3D viewer and add point cloud-----
      // --------------------------------------------
      //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
      //viewer->setBackgroundColor (0, 0, 0);
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, red, green, blue);
      viewer->updatePointCloud<pcl::PointXYZ> (cloud, single_color, "sample cloud");
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
      //viewer->addCoordinateSystem (0.1);
      viewer->initCameraParameters ();
      //return (viewer);
    }


// -------------------------- SEGMENTATION RELATED ------------------------------

    void DownsampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
    {
        // Create the filtering object: downsample the dataset using a leaf size of 1cm
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
        vg.setInputCloud (cloud);
        vg.setLeafSize (0.001f, 0.001f, 0.001f);
        vg.filter (*cloud);
        std::cout << "PointCloud after filtering has: " << cloud->points.size ()  << " data points." << std::endl; //*
    }

    void RemoveNansFromCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
    {
        // Build a passthrough filter to remove spurious NaNs
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud (cloud);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0, 1.5);
        pass.filter (*cloud);
        std::cerr << "PointCloud after filtering has: " << cloud->points.size () << " data points." << std::endl;
    }

    void EstimateCloudNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals)
    {
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
        // Estimate point normals
        ne.setSearchMethod (tree);
        ne.setInputCloud (cloud);
        ne.setKSearch (50);
        ne.compute (*cloud_normals);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr SegmentPlane(
                                                     pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                                     pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals,
                                                     pcl::ModelCoefficients::Ptr &coefficients_plane
                                                    )
    {
        pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
        pcl::PointIndices::Ptr inliers_indicies (new pcl::PointIndices);
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
        pcl::ExtractIndices<pcl::Normal> extract_normals;

        // Create the segmentation object for the planar model and set all the parameters
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
        seg.setNormalDistanceWeight (0.1);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (100);
        seg.setDistanceThreshold (0.03);
        seg.setInputCloud (cloud);
        seg.setInputNormals (cloud_normals);
        // Obtain the plane inliers and coefficients
        seg.segment (*inliers_indicies, *coefficients_plane);
        std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

        // Extract the planar inliers from the input cloud
        extract.setInputCloud (cloud);
        extract.setIndices (inliers_indicies);
        extract.setNegative (false);
        extract.filter (*cloud_plane);
        std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud);

        // remove corresponding plane normals
        extract_normals.setNegative (true);
        extract_normals.setInputCloud (cloud_normals);
        extract_normals.setIndices (inliers_indicies);
        extract_normals.filter (*cloud_normals);

        return cloud_plane;
    }


    pcl::PointCloud<pcl::PointXYZ>::Ptr ExtractObjects( pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                                         pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals,
                                                         pcl::ModelCoefficients::Ptr &coefficients_cylinder
                                                        )
    {
        pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
        pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder (new pcl::PointCloud<pcl::PointXYZ> ());
        pcl::ExtractIndices<pcl::Normal> extract_normals;
        //bool success = false;

        // Create the segmentation object for cylinder segmentation and set all the parameters
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_CYLINDER);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setNormalDistanceWeight (0.1);
        seg.setMaxIterations (1000);
        seg.setDistanceThreshold (0.05);
        seg.setRadiusLimits (0, 0.1);
        seg.setInputCloud (cloud);
        seg.setInputNormals (cloud_normals);

        // Obtain the cylinder inliers and coefficients
        seg.segment (*inliers_cylinder, *coefficients_cylinder);
        std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

        // Extract the cyclinder from the cloud
        extract.setInputCloud (cloud);
        extract.setIndices (inliers_cylinder);
        extract.setNegative (false);
        extract.filter (*cloud_cylinder);

        // and then remove them from the cloud
        extract.setNegative(true);
        extract.filter(*cloud);

        // remove corresponding cylinder normals
        extract_normals.setNegative (true);
        extract_normals.setInputCloud (cloud_normals);
        extract_normals.setIndices (inliers_cylinder);
        extract_normals.filter (*cloud_normals);

        return cloud_cylinder;
    }
}
