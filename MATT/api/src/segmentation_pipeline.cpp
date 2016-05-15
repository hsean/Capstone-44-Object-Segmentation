

#include <segmentation_pipeline.h>
#include <pcl/segmentation/extract_clusters.h>



using namespace c44;

const float SegmentationPipeline::normal_search_radius = 0.03;

SegmentationPipeline::SegmentationPipeline(Cloud3D::Ptr rawCloud,
                                           float voxelSize,
                                           float sampleSize,
                                           float stdDev,
                                           float iterationDivisor) :

        normals(new PointCloud<Normal>),
//objectCloud(new Cloud3D),
        denoisedCloud(rawCloud),
        cloudMinusPrism(new Cloud3D),
        iterationDivisor(iterationDivisor),
        convexHull (new Cloud3D),
        planeCloud(new Cloud3D),
        sampleSize(sampleSize),
        stdDev(stdDev)
{
  

    passthroughFilter(denoisedCloud,
                      denoisedCloud,
                      "z", 0.0, 1.5);

    if (voxelSize > 0) {
      voxelFilter(denoisedCloud, denoisedCloud, voxelSize);
    }

    NormalEstimation<PointXYZ, Normal> ne;

    std::cerr << "PointCloud after filtering has: " <<
    denoisedCloud->points.size ()      <<
    " data points."                    <<
    std::endl;

    search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>());
    //ne.setRadiusSearch (SegmentationPipeline::normal_search_radius);
    ne.setSearchMethod(tree);
    ne.setInputCloud(denoisedCloud);
    ne.setKSearch(25);
    ne.compute(*normals);


}


bool SegmentationPipeline::performSegmentation(float cluster_tolerance){
    if (extractPrism()){
      clusterize(cluster_tolerance);
        //this is to support extracting multiple objects
//      int max = 3;
//      bool stillFindingStuff = true;
//      do {
//          stillFindingStuff = extractGraspableObject(SACMODEL_CYLINDER);
//      } while (stillFindingStuff && graspableObjects.size() < max);
      return true;
    } else {
      return false;
    }
}



bool SegmentationPipeline::extractPlane()
{
    planeCoefficients = ModelCoefficients::Ptr(new ModelCoefficients);
    PointIndices::Ptr plane_indices(new PointIndices);


    ExtractIndices<PointXYZ> extractor;

    // Create the segmentation object for the planar
    // model and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType(SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight (0.1);

    seg.setMethodType(SAC_RANSAC);
    int plane_iterations = int(ceil(float(100)/iterationDivisor));
    seg.setMaxIterations(plane_iterations);
    seg.setDistanceThreshold (0.03);
    seg.setInputCloud (denoisedCloud);
    seg.setInputNormals (normals);
    // Obtain the plane inliers and coefficients
    seg.segment(*plane_indices, *planeCoefficients);
    std::cerr << "Plane coefficients: " << *planeCoefficients << std::endl;

    // Extract the planar inliers from the input cloud
    extractor.setInputCloud (denoisedCloud);
    extractor.setIndices (plane_indices);

    extractor.filter (*planeCloud);
    return planeCloud->points.size() > 0;
}

bool SegmentationPipeline::extractPrism()
{

    if (extractPlane()){

        pcl::ConvexHull<pcl::PointXYZ> chull;
        chull.setInputCloud (planeCloud);
        chull.setDimension(2);
        chull.reconstruct(*convexHull);


        //segment out those points that are in the polygonal prism
        pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prismExtractor;
        prismExtractor.setInputCloud(denoisedCloud);
        prismExtractor.setInputPlanarHull(convexHull);
        prismExtractor.setHeightLimits(0.02, 0.2);
        pcl::PointIndices::Ptr objectIndices (new pcl::PointIndices);
        prismExtractor.segment (*objectIndices);


        //remove the convex-hull's points from the main cloud
        ExtractIndices<PointXYZ> extractor;
        extractor.setInputCloud(denoisedCloud);
        extractor.setIndices(objectIndices);
        extractor.filter(*cloudMinusPrism);

        ExtractIndices<Normal> normalExtractor;
        normalExtractor.setInputCloud (normals);
        normalExtractor.setIndices(objectIndices);
        normalExtractor.filter (*normals);

        return cloudMinusPrism->points.size() > 0;
    } else{
        return false;
    }
}


/*
bool SegmentationPipeline::extractGraspableObject(SacModel model)
{
  
  ModelCoefficients::Ptr objectCoefficients(new ModelCoefficients);
  ExtractIndices<PointXYZ> extractor;
  ExtractIndices<Normal> normalExtractor;

  PointIndices::Ptr inliers(new PointIndices);
  PointCloud<Normal>::Ptr objectNormals(new PointCloud<Normal>);
  Cloud3D::Ptr cylinderCloud(new Cloud3D);
  Cloud3D::Ptr denoisedObjectCloud(objectCloud);

  removeNoise(denoisedObjectCloud,
              denoisedObjectCloud,
              sampleSize, stdDev);
  clusterize();
  
  
  normals.reset(new PointCloud<Normal>);

  NormalEstimation<PointXYZ, Normal> ne;

  std::cerr << "PointCloud after filtering has: " <<
  denoisedCloud->points.size ()      <<
  " data points."                    <<
  std::endl;

  search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>());
  ne.setSearchMethod(tree);
  ne.setInputCloud(objectCloud);
  ne.setKSearch(25);
  ne.compute(*normals);


  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (model);
  seg.setMethodType (SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  int objIterations = int(ceil(float(10000)/iterationDivisor));
  seg.setMaxIterations(objIterations);
  seg.setDistanceThreshold (0.05);
  seg.setRadiusLimits (0.01, 0.12);
  seg.setInputCloud (objectCloud);
  seg.setInputNormals (normals);

  // Obtain the cylinder inliers and coefficients
  seg.segment (*inliers, *objectCoefficients);
  std::cerr << "Object coefficients: " << *objectCoefficients << std::endl;

  extractor.setInputCloud (objectCloud);
  extractor.setIndices (inliers);
  extractor.setNegative (false);
  normalExtractor.setIndices(inliers);
  extractor.filter(*cylinderCloud);
  normalExtractor.filter(*objectNormals);
  if (cylinderCloud->size() == 0){
      return false;
  } else {
      //subtract out the object just found from the original input cloud
      extractor.setInputCloud (objectCloud);
      extractor.setNegative (true);
      extractor.setIndices (inliers);
      extractor.filter (*objectCloud);
      GraspableObject obj(*objectCoefficients,cylinderCloud,objectNormals);
      graspableObjects.push_back(obj);
      return true;
  }


}
*/

void SegmentationPipeline::clusterize(float cluster_tolerance){
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  objectCloud = Cloud3D::Ptr(cloudMinusPrism);
  tree->setInputCloud (objectCloud);
  
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (cluster_tolerance);
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (125000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (objectCloud);
  ec.extract (cluster_indices);
  
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    
    PointCloud<Normal>::Ptr normal_cluster(new PointCloud<Normal>);
    
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
      cloud_cluster->points.push_back (objectCloud->points[*pit]); //*
      normal_cluster->points.push_back(normals->points[*pit]);
    }
    cloud_cluster->width = cloud_cluster->points.size ();
    
    cloud_cluster->height = 1;
    
    cloud_cluster->is_dense = true;
    
    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";
    
    auto rb = RigidBody(cloud_cluster,normal_cluster,normal_search_radius);
    objects.push_back(rb);
    
    j++;
  }
  
  
}



