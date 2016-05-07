#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>

int main()
{
	//Load Point Cloud from a file
	// The file should be in the same folder
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("milk_cartoon_all_small_clorox.pcd", *cloud);

	//Create an object for the normal estimation and 
	//compute the normals
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	
	// The AVERAGE_3D_GRADIENT mode creates 6 integral images to 
	//compute the normals using the cross-product of horizontal and
	//vertical 3D gradients and computes the normals using the cross-
	//product between these two gradients.

	pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
             ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
             ne.setMaxDepthChangeFactor(0.02f);
             ne.setNormalSmoothingSize(10.0f);
             ne.setInputCloud(cloud);   		// Load the cloud
             ne.compute(*normals);				// Compute Normal
	//visualize normals
	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	viewer.setBackgroundColor(0.0,0.0,0.5);
	viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud,normals);

	while(!viewer.wasStopped())
	{
		viewer.spinOnce();
	}

	return 0;
}