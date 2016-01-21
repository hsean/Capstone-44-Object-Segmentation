/* AUTHOR: Sean Hendrickson
 * FILE: pcl_visualizer.cpp
 * DATE: 1/21/2016
 * DESC: Uses the PCL Visualizer class to display a passed PCD file.
 *       Based on pcl_vicualizer_demo by Geoffrey Biggs
 */
 
#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
using std::cout;
using std::endl;

/* Help
*/
void printUsage (const char* progName)
printUsage (const char* progName)
{
  std::cout << "\n\nUsage: "<<progName<<" [options]\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
	        << "-f [file]    Simple visualization of file\n"
            << "\n\n";
}

int main(int argc, char *argv[])
{
	// create point cloud object
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	
	// check that enough arguments are passed
	if(argc != 2)
	{
		cout << "[option] <filename> not found" << endl;
	}
	else
	{   
		if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud) == -1) //* load the file
		{
			PCL_ERROR ("Couldn't read file %s\n", argv[1]);
			return (-1);
		}
		
		
	}
	return 0;
}
