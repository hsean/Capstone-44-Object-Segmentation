/************************************************************************
 * Realsense PCD Grabber
 * James Schiffer - Capstone Group 44 - PSU/Intel
 * V0.01
 *
 * Purpose:
 * Grab data from the Realsense camera and then convert/display/save
 * as the PCL PCD format.
 *
 * Requirements:
 * PCL + librealsense + ubuntu 14.04 + cmake and inhereited dependencies
 *
 * Usage:
 * After all requirements are installed, please use the cmake file to
 * compile. Tested using qtcreator by opening the cmake file as a project.
 *
 *
 * *********************************************************************/

/* First include the librealsense C header file */
#include <librealsense/rs.hpp>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include "pcl/io/pcd_io.h"
#include "pcl/io/file_io.h"
#include "pcl/visualization/cloud_viewer.h"
#include "pcl/point_types.h"
#include <string>

/*  IMPLMENT THIS LATER
template<typename T> inline void
ConvertPoint(const float3, T& tgt)
{


}
*/

int main()
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
    rs::device * dev = ctx.get_device(0);
    printf("\nUsing device 0, an %s\n", dev->get_name());
    printf("    Serial number: %s\n", dev->get_serial());
    printf("    Firmware version: %s\n", dev->get_firmware_version());

    // Configure depth and color to run with the device's preferred settings
    dev->enable_stream(rs::stream::depth, rs::preset::best_quality);
    dev->start();

    // ==== PCL SETUP ====
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::visualization::CloudViewer viewer("Realsense Cloud Viewer");
    //viewer.setBackgroundColor(1.0, 0.5, 1.0);
    int numshots = 0;

    // Take 10 shots
    while(numshots < 10) // !viewer.wasStopped()) put this back after fixing visualizer
    {
        // Wait for new frame data
        dev->wait_for_frames();

        // Retrieve our images
        const uint16_t * depth_image = (const uint16_t *)dev->get_frame_data(rs::stream::depth);

        // Retrieve camera parameters for mapping between depth and color
        rs::intrinsics depth_intrin = dev->get_stream_intrinsics(rs::stream::depth);
        float scale = dev->get_depth_scale();

        // pcl cloud prep
        cloud.reset(new pcl::PointCloud<pcl::PointXYZ>(depth_intrin.width, depth_intrin.height));
        cloud->is_dense = false;
        cloud->points.resize(depth_intrin.width * depth_intrin.height);
        float max_distance = 1.5;       // get rid of noisy data that is past 3.5 meters

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
                if(depth_value == 0 || depth_point.z > max_distance)
                {
                    cloud->points[i].x = cloud->points[i].y = cloud->points[i].z = (float) nan;
                    continue;
                }
                else // write the valid data to the PCD format
                {
                    cloud->points[i].x = -depth_point.x;    // seems to orientate better this way
                    cloud->points[i].y = depth_point.y;
                    cloud->points[i].z = depth_point.z;
                }

            }
        }

        // display - DOES NOT WORK WITH REALSENSE - FIGURE OUT WHY
        // or just try implmenting pclvisualizer
        viewer.showCloud(cloud);
        numshots++;
        std::string astring = std::to_string(numshots);
        // need to derefernce pointer to pass - don't know why
        pcl::io::savePCDFile("west_tabletest" + astring + ".pcd", *cloud);
    }

    return EXIT_SUCCESS;

}
