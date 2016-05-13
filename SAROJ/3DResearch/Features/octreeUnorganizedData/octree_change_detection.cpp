#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <iostream>
#include <vector>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>

std::string modelFileName ;
std::string sceneFileName;

// Command line parser 
void 
parseCommandLine (int argc, char *argv[])
{
//Model & scene filenames
  std::vector<int> filenames;
  filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
  if (filenames.size () != 2)
  {
    std::cout << "Filenames missing.\n";
    exit (-1);
  }

   // Store the filenames in temp variables
  modelFileName = argv[filenames[0]];
  sceneFileName = argv[filenames[1]];
}



int
main (int argc, char** argv)
{


  //=================================================================
  // Create a cloud pointer
  // The file should be in the same folder
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZ>);
//=================================================================
  // Parse Command Line
  parseCommandLine (argc, argv);
  // Load the file
  if(pcl::io::loadPCDFile(modelFileName, *cloudA) < 0)
  {
    std::cout << "Error Loading model cloud." <<std::endl;
    return(-1);
  }

  if (pcl::io::loadPCDFile (sceneFileName, *cloudB) < 0)
  {
    std::cout << "Error loading scene cloud." << std::endl;
    return (-1);
  }

  // Octree resolution - side length of octree voxels
  float resolution = 32.0f;

  // Instantiate octree-based point cloud change detection class
  pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree (resolution);

  // Add points from cloudA to octree
  octree.setInputCloud (cloudA);
  octree.addPointsFromInputCloud ();

  // Switch octree buffers: This resets octree but keeps previous tree structure in memory.
  octree.switchBuffers ();

  // Add points from cloudB to octree
  octree.setInputCloud (cloudB);
  octree.addPointsFromInputCloud ();

  std::vector<int> newPointIdxVector;

  // Get vector of point indices from octree voxels which did not exist in previous buffer
  octree.getPointIndicesFromNewVoxels (newPointIdxVector);

  // Output points
  std::cout << "Output from getPointIndicesFromNewVoxels:" << std::endl;
  for (size_t i = 0; i < newPointIdxVector.size (); ++i)
    std::cout << i << "# Index:" << newPointIdxVector[i]
              << "  Point:" << cloudB->points[newPointIdxVector[i]].x << " "
              << cloudB->points[newPointIdxVector[i]].y << " "
              << cloudB->points[newPointIdxVector[i]].z << std::endl;

}