/******************************************************************************
* Author: Sean Hendrickson, James Schiffer
* File: c44_pipes.cpp
* Last Modified: 22 April 2016
* Description: This file contains implementations of PCL pipelines for use
*               in object and hand segmentation of 3D objects sitting on
*               a plane.
*              This is part of Portland State Universitie's 2015-2016 ECE
*              capstone projects, specifically team #44
******************************************************************************/
#include "c44_pipes.h"


using namespace c44_pipes;

///////////////////////////////////////////////////////////////////////////////
/**
 * desc:  This function cuts off all points outside a given range
 *        on the field passed the the field parameter.
 * param: (in) source_cloud - ptr to point cloud
 *        (out) filtered_cloud - ptr to cloud after filtering
 *        (in) axis - axis to filter values passed as 'x', 'y', or 'z'
 *        (in) min_point - kept points are >= minPoint
 *        (in) max_point - kept points are <= maxPoint
 */
void c44_pipes::passthroughFilter(
        Cloud3D::Ptr source_cloud,
        Cloud3D::Ptr filtered_cloud,
        const std::string& axis,
        float min_point,
        float max_point
        )
{
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(source_cloud);
    pass.setFilterFieldName(axis);
    pass.setFilterLimits(min_point, max_point);
    pass.filter(*filtered_cloud);
}


///////////////////////////////////////////////////////////////////////////////
/**
 * desc: This function uses a voxel filter to downsample a PCD file.
 * param: (in) source_cloud - ptr source_cloudto cloud to be filtered
 *        (out) filtered_cloud - ptr to cloud after filtering
 *        (in) leaf_size - size of voxel in cm^3. (ex. 0.01 = 1 cm)
 */
void c44_pipes::voxelFilter(
        Cloud3D::Ptr source_cloud,
        Cloud3D::Ptr filtered_cloud,
        float leaf_size
        )
{
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(source_cloud);
    sor.setLeafSize(leaf_size, leaf_size, leaf_size);
    sor.filter(*filtered_cloud);
}



///////////////////////////////////////////////////////////////////////////////
/**
 * desc: This function computes the average distance between each point in a PCD
 *       file and then uses the standard deviation to designate points as outliers.
 *       All outliers are then removed from the resultant point cloud.
 * param: (in) source_cloud - ptr to input cloud
 *        (out) filtered_cloud - ptr to cloud with outliers removed
 *        (in) neighbors_to_analyze - number of nearest neighbors to analyze (ex. 50)
 *        (in) std_deviation - standard deviation used to find outliers (ex. 1.0)
 */
void c44_pipes::removeNoise(
        Cloud3D::Ptr source_cloud,
        Cloud3D::Ptr filtered_cloud,
        int neighbors_to_analyze,
        double std_deviation
        )
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(source_cloud);
    sor.setMeanK(neighbors_to_analyze);
    sor.setStddevMulThresh(std_deviation);
    sor.filter(*filtered_cloud);
}


///////////////////////////////////////////////////////////////////////////////
/**
 * desc: This function takes a source point cloud and segments out a plane.
 *       The inliers of the plane, as well as all information needed to
 *       rebuild the plane is returned through the function parameters.
 * param: (in) source_cloud - ptr to input cloud
 *        (out) inliers_cloud - returns inliers of plane as pointcloud
 *        (out) outliers_cloud - returns outliers of plane as pointcloud
 *        (out) coefficients - returns the coefficients of the plane
 *        (out) indices - returns the indices of the plane
 *        (in) min_angle - minimum radius between plane and camera in radians (ex. 0)
 *        (in) max_angle - maximum radius between plane and camera in radians (ex. 1.57)
 *        (in) ransac_iterations - maximum amount of times RANSAC can estimate plane (ex. 1000)
 *        (in) threshold_distance - range of points considered inliers from plane model (ex 0.01 = 1cm)
 * pre-cond: all output parameters must be declared before function call
 * ret: -1 if a plane could not be found else
 *      -2 if source PCD is empty
 *       0 if succesfull
 */
int c44_pipes::getPlane(
        Cloud3D::Ptr source_cloud,
        Cloud3D::Ptr inliers_cloud,
        Cloud3D::Ptr outliers_cloud,
        pcl::ModelCoefficients::Ptr coefficients,
        pcl::PointIndices::Ptr plane_indices,
        double min_angle,
        double max_angle,
        int ransac_iterations,
        double threshold_distance
        )
{

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;

    // Optional
    seg.setOptimizeCoefficients(true);

    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(ransac_iterations);
    seg.setDistanceThreshold(threshold_distance);
    seg.setRadiusLimits(min_angle, max_angle);  // TODO: may need to use different function

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(source_cloud);
    seg.segment(*plane_indices, *coefficients);

    // check if a plane was found
    if (plane_indices->indices.size() == 0)
    {	// No plane was found
        return -1;
    }

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;

    // Extract the inliers
    extract.setInputCloud(source_cloud);
    extract.setIndices(plane_indices);
    extract.setNegative(false);
    extract.filter(*inliers_cloud);

    #ifdef DEBUG_PLANE
    std::cerr << "PointCloud representing the planar component: "
              << inliers_cloud->width * inliers_cloud->height << " data points."
              << std::endl;
    #endif

    // Create the filtering object
    extract.setNegative(true);
    extract.filter(*outliers_cloud);
    return 0;
}


///////////////////////////////////////////////////////////////////////////////
/**
 * desc: This function takes a source PCD image an segments out objects found
 *       on a plane.
 * param: (in) source_cloud - ptr to input cloud
 *        (out) objects_cloud - returns cloud with objects above plane
 *        (in) min_angle - minimum radius between plane and camera in radians (ex. 0)
 *        (in) max_angle - maximum radius between plane and camera in radians (ex. 1.57)
 *        (in) ransac_iterations - maximum amount of times RANSAC can estimate plane (ex. 1000)
 *        (in) threshold_distance - range of points considered inliers from plane model (ex 0.01 = 1cm)
 *        (in) min_object_distance - lowest point of objects above a plane (ex 0.01 = 1cm)
 *        (in) max_object_distance - highest point of objects above a plane (ex 0.2 = 20cm)
 * pre-cond: all output variables must be declared before function is called.
 * ret: -1 if a plane could not be found else
 *      -2 if source PCD is empty
 *       0 if succesfull
 */
int c44_pipes::getPrism(
        Cloud3D::Ptr source_cloud,
        Cloud3D::Ptr objects_cloud,
        double min_angle,
        double max_angle,
        int ransac_iterations,
        double threshold_distance,
        double min_object_distance,
        double max_object_distance
        )
{
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr plane_indices (new pcl::PointIndices);
    Cloud3D::Ptr inliers_cloud (new Cloud3D);

    // find plane parameters
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(ransac_iterations);
    seg.setDistanceThreshold(threshold_distance);
    seg.setRadiusLimits(min_angle, max_angle);  // TODO: may need to use different function
    seg.setInputCloud(source_cloud);
    seg.segment(*plane_indices, *coefficients);

    //std::cerr << "Plane Size: " << plane_indices->indices.size() << std::endl;

    // check if a plane was found
    if (plane_indices->indices.size() == 0)
    {	// No plane was found
        return -1;
    }
    else
    {
        // Create the filtering object
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;

        // Extract the inliers
        extract.setInputCloud(source_cloud);
        extract.setIndices(plane_indices);
        extract.filter(*inliers_cloud);

        // retrieve the convex hull
        Cloud3D::Ptr convexHull (new Cloud3D);
        pcl::ConvexHull<pcl::PointXYZRGB> chull;
        chull.setInputCloud (inliers_cloud);
        chull.setDimension(2);
        chull.reconstruct(*convexHull);

        // redundant check
        if (chull.getDimension() == 2)
        {
            // segment those points that are in the polygonal prism
            pcl::ExtractPolygonalPrismData<pcl::PointXYZRGB> prism;
            prism.setInputCloud(source_cloud);
            prism.setInputPlanarHull(convexHull);
            prism.setHeightLimits(min_object_distance, max_object_distance);
            pcl::PointIndices::Ptr objectIndices (new pcl::PointIndices);
            prism.segment (*objectIndices);

            // get all points retrieved by the hull
            extract.setIndices(objectIndices);
            extract.filter(*objects_cloud);

            // check if any objects were found
            if(0 == objects_cloud->points.size())
            {	// no objects found
                return -1;
            }
        }
        else
        {	// the chosen hull is not planar
            return -1;
        }
    }
    return 0;
}


///////////////////////////////////////////////////////////////////////////////
/**
 * desc: This function uses a "Euclidean Cluster Extraction" algorithm to pair
 *       separate data in a Point Cloud into separate objects.
 * param: (in) source_cloud - ptr to input cloud
 *        (out) cluster_indicies - ojects that holds each cluster as an element of
 *                                 a vector. cluster_indices[0] is cluster 1, and
 *                                 cluster_indices[1] is cluster 2, and so on.
 *        (in) cluster_tolerance - The maximum distance between points in the
 *                                same cluster (e.g. 0.02 = 2cm)
 *        (in) min_cluster_size - minimum number of points in a single cluster
 *        (in) max_cluster_size - minimum number of points in a single cluster
 * pre-cond: cluster_indicies needs to be declared before function call
 */
void c44_pipes::clusterExtraction(
        Cloud3D::Ptr source_cloud,
        std::vector<pcl::PointIndices>* cluster_indices,
        double cluster_tolerance,
        int min_cluster_size,
        int max_cluster_size
        )
{
    // create the kdTree object for the search method of extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (source_cloud);
    //pcl::PCDWriter writer;

    // store each cluster as a vector of indices
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (cluster_tolerance);
    ec.setMinClusterSize (min_cluster_size);
    ec.setMaxClusterSize (max_cluster_size);
    ec.setSearchMethod (tree);
    ec.setInputCloud (source_cloud);
    ec.extract (*cluster_indices);
}


///////////////////////////////////////////////////////////////////////////////
/**
 * desc:
 * Seperates a cluster discovered by Euclidean Extraction from the
 * cloud of clusters. Not sure why we are not using indicies extraction for this.
 * #TODO - see if manual indicies extraction was actually neccessary or some
 *          weird tutorial related thing
 *
 * To separate each cluster out of the vector<PointIndices> we have to iterate
 * through cluster_indices, create a new PointCloud for each entry
 * and write all points of the current cluster in the PointCloud.
 * param: (inout) it - Iterator holding reference to current cluster to seperate
 *        (in) cloud_of_objs - point cloud of se
 *        (out) cloud_cluster - point cloud of seperated object
 */
bool c44_pipes::seperateClusters(
        std::vector<pcl::PointIndices> cluster_indices,
        std::vector<pcl::PointIndices>::const_iterator obj_clusters_index,
        Cloud3D::Ptr cloud_of_objs,
        Cloud3D::Ptr cloud_cluster
        ){
    bool last_cluster = false;

    if(obj_clusters_index != cluster_indices.end ())
    {
        //cloud_cluster = new Cloud3D::Ptr();
        for (std::vector<int>::const_iterator pit = obj_clusters_index->indices.begin (); pit != obj_clusters_index->indices.end (); ++pit)
        {
            cloud_cluster->points.push_back (cloud_of_objs->points[*pit]); //*
        }

        // manually set the parameters
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        // Set ROS specific parameters
        //cloud_cluster->header.frame_id = "/camera_depth_frame";
        //cloud_cluster->header.stamp = ros::Time::now().toSec();

    }
    else
    {
        // all clusters processed
        last_cluster = true;
    }

    return last_cluster;
}

