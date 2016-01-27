# Building PCL on Windows 10 using MSVC 2015

These instructions include how to install, setup, and run PCL on Windows 10 using pre-built binaries.  This guide will assume one is using Microsoft's Visual Studio 2015 on a 64-bit machine.  Note: this guide is a essentially paraphrasing the linked document.

### Step 0: Install PCL   
go to the provided link and install the all-in-one installer for your verson of windows and msvc. [link here](http://unanancyowen.com/?p=1255&lang=en).  Install PCL at the recommended location.

### Step 1: Set environment variables   
go to control panel->system->advanced system settings
click on the Environment Variables button and enter the following information
```
VARIABLE       VALUE
---------------------------------------------
PCL_ROOT       C:\Program Files\PCL 1.7.2
Path           ;%PCL_ROOT%\bin
               ;%PCL_ROOT%\3rdParty\FLANN
               ;%PCL_ROOT%\3rdParty\VTK\bin
```  

### Step 2: Set up properties for Visual Studio and Dependencies
Links to each property sheet is provided below   
[PCL Property Sheet](https://gist.github.com/UnaNancyOwen/d2c380603a4642525b5c#file-pcl-props)   
[OpenNI2 Property Sheet](https://gist.github.com/UnaNancyOwen/2b58b3bb0f11d0c8fc38#file-openni2-props)   
[Kinect for Windows SDK v1.x Property Sheet](https://gist.github.com/UnaNancyOwen/2c5f2bb71b823283a296#file-kinectsdk-props)   
[Kinect for Windows SDK v2.x Property Sheet](https://gist.github.com/UnaNancyOwen/905fea1ea1832833a22d#file-kinectsdk2-props)   
NOTE: Kinect for XBOX ONE is v2.0.

Download each file above somewhere on your computer.  I saved mine to Desktop\PCL_VS2015Properties

### Step 3: Start Visual Studio 2015
Create a new empty c++ project.  For example PCL_DEMO.
Now, with the solution selected, go to view->property pages
If this option is greyed out, make sure PCL_DEMO is selected in the solution explorer.

### Step 4: Add PCL Properties
One can extract the properties based on ones system from the PCL Properties file.
For simplicity, the properties for Win 64-bit VS2015 will be displayed below
```
C/C++ > General > Additional Include Directories:
$(PCL_ROOT)\include\pcl-1.7;
$(PCL_ROOT)\3rdParty\Boost\include\boost-1_57;
$(PCL_ROOT)\3rdParty\Eigen\eigen3;
$(PCL_ROOT)\3rdParty\FLANN\include;
$(PCL_ROOT)\3rdParty\QHull\include;
$(PCL_ROOT)\3rdParty\VTK\include\vtk-6.2;

Linker > General > Additional Library Directories:
$(PCL_ROOT)\lib;
$(PCL_ROOT)\lib\$(Configuration);
$(PCL_ROOT)\3rdParty\Boost\lib;
$(PCL_ROOT)\3rdParty\FLANN\lib;
$(PCL_ROOT)\3rdParty\QHull\lib;
$(PCL_ROOT)\3rdParty\VTK\lib;

Linker > Input > Additional Dependencies:
pcl_common_debug.lib;
pcl_features_debug.lib;
pcl_filters_debug.lib;
pcl_io_debug.lib;
pcl_io_ply_debug.lib;
pcl_kdtree_debug.lib;
pcl_keypoints_debug.lib;
pcl_octree_debug.lib;
pcl_outofcore_debug.lib;
pcl_people_debug.lib;
pcl_recognition_debug.lib;
pcl_registration_debug.lib;
pcl_sample_consensus_debug.lib;
pcl_search_debug.lib;
pcl_segmentation_debug.lib;
pcl_surface_debug.lib;
pcl_tracking_debug.lib;
pcl_visualization_debug.lib;
libboost_atomic-vc140-mt-gd-1_57.lib;
libboost_chrono-vc140-mt-gd-1_57.lib;
libboost_container-vc140-mt-gd-1_57.lib;
libboost_context-vc140-mt-gd-1_57.lib;
libboost_coroutine-vc140-mt-gd-1_57.lib;
libboost_date_time-vc140-mt-gd-1_57.lib;
libboost_exception-vc140-mt-gd-1_57.lib;
libboost_filesystem-vc140-mt-gd-1_57.lib;
libboost_graph-vc140-mt-gd-1_57.lib;
libboost_iostreams-vc140-mt-gd-1_57.lib;
libboost_locale-vc140-mt-gd-1_57.lib;
libboost_log-vc140-mt-gd-1_57.lib;
libboost_log_setup-vc140-mt-gd-1_57.lib;
libboost_math_c99-vc140-mt-gd-1_57.lib;
libboost_math_c99f-vc140-mt-gd-1_57.lib;
libboost_math_c99l-vc140-mt-gd-1_57.lib;
libboost_math_tr1-vc140-mt-gd-1_57.lib;
libboost_math_tr1f-vc140-mt-gd-1_57.lib;
libboost_math_tr1l-vc140-mt-gd-1_57.lib;
libboost_mpi-vc140-mt-gd-1_57.lib;
libboost_prg_exec_monitor-vc140-mt-gd-1_57.lib;
libboost_program_options-vc140-mt-gd-1_57.lib;
libboost_random-vc140-mt-gd-1_57.lib;
libboost_regex-vc140-mt-gd-1_57.lib;
libboost_serialization-vc140-mt-gd-1_57.lib;
libboost_signals-vc140-mt-gd-1_57.lib;
libboost_system-vc140-mt-gd-1_57.lib;
libboost_test_exec_monitor-vc140-mt-gd-1_57.lib;
libboost_thread-vc140-mt-gd-1_57.lib;
libboost_timer-vc140-mt-gd-1_57.lib;
libboost_unit_test_framework-vc140-mt-gd-1_57.lib;
libboost_wave-vc140-mt-gd-1_57.lib;
libboost_wserialization-vc140-mt-gd-1_57.lib;
flann_cpp_s-gd.lib;qhullstatic_d.lib;
vtkalglib-6.2-gd.lib;
vtkChartsCore-6.2-gd.lib;
vtkCommonColor-6.2-gd.lib;
vtkCommonComputationalGeometry-6.2-gd.lib;
vtkCommonCore-6.2-gd.lib;
vtkCommonDataModel-6.2-gd.lib;
vtkCommonExecutionModel-6.2-gd.lib;
vtkCommonMath-6.2-gd.lib;
vtkCommonMisc-6.2-gd.lib;
vtkCommonSystem-6.2-gd.lib;
vtkCommonTransforms-6.2-gd.lib;
vtkDICOMParser-6.2-gd.lib;
vtkDomainsChemistry-6.2-gd.lib;
vtkexoIIc-6.2-gd.lib;
vtkexpat-6.2-gd.lib;
vtkFiltersAMR-6.2-gd.lib;
vtkFiltersCore-6.2-gd.lib;
vtkFiltersExtraction-6.2-gd.lib;
vtkFiltersFlowPaths-6.2-gd.lib;
vtkFiltersGeneral-6.2-gd.lib;
vtkFiltersGeneric-6.2-gd.lib;
vtkFiltersGeometry-6.2-gd.lib;
vtkFiltersHybrid-6.2-gd.lib;
vtkFiltersHyperTree-6.2-gd.lib;
vtkFiltersImaging-6.2-gd.lib;
vtkFiltersModeling-6.2-gd.lib;
vtkFiltersParallel-6.2-gd.lib;
vtkFiltersParallelImaging-6.2-gd.lib;
vtkFiltersProgrammable-6.2-gd.lib;
vtkFiltersSelection-6.2-gd.lib;
vtkFiltersSMP-6.2-gd.lib;
vtkFiltersSources-6.2-gd.lib;
vtkFiltersStatistics-6.2-gd.lib;
vtkFiltersTexture-6.2-gd.lib;
vtkFiltersVerdict-6.2-gd.lib;
vtkfreetype-6.2-gd.lib;
vtkftgl-6.2-gd.lib;
vtkGeovisCore-6.2-gd.lib;
vtkgl2ps-6.2-gd.lib;
vtkhdf5-6.2-gd.lib;
vtkhdf5_hl-6.2-gd.lib;
vtkImagingColor-6.2-gd.lib;
vtkImagingCore-6.2-gd.lib;
vtkImagingFourier-6.2-gd.lib;
vtkImagingGeneral-6.2-gd.lib;
vtkImagingHybrid-6.2-gd.lib;
vtkImagingMath-6.2-gd.lib;
vtkImagingMorphological-6.2-gd.lib;
vtkImagingSources-6.2-gd.lib;
vtkImagingStatistics-6.2-gd.lib;
vtkImagingStencil-6.2-gd.lib;
vtkInfovisCore-6.2-gd.lib;
vtkInfovisLayout-6.2-gd.lib;
vtkInteractionImage-6.2-gd.lib;
vtkInteractionStyle-6.2-gd.lib;
vtkInteractionWidgets-6.2-gd.lib;
vtkIOAMR-6.2-gd.lib;
vtkIOCore-6.2-gd.lib;
vtkIOEnSight-6.2-gd.lib;
vtkIOExodus-6.2-gd.lib;
vtkIOExport-6.2-gd.lib;
vtkIOGeometry-6.2-gd.lib;
vtkIOImage-6.2-gd.lib;
vtkIOImport-6.2-gd.lib;
vtkIOInfovis-6.2-gd.lib;
vtkIOLegacy-6.2-gd.lib;
vtkIOLSDyna-6.2-gd.lib;
vtkIOMINC-6.2-gd.lib;
vtkIOMovie-6.2-gd.lib;
vtkIONetCDF-6.2-gd.lib;
vtkIOParallel-6.2-gd.lib;
vtkIOParallelXML-6.2-gd.lib;
vtkIOPLY-6.2-gd.lib;
vtkIOSQL-6.2-gd.lib;
vtkIOVideo-6.2-gd.lib;
vtkIOXML-6.2-gd.lib;
vtkIOXMLParser-6.2-gd.lib;
vtkjpeg-6.2-gd.lib;
vtkjsoncpp-6.2-gd.lib;
vtklibxml2-6.2-gd.lib;
vtkmetaio-6.2-gd.lib;
vtkNetCDF-6.2-gd.lib;
vtkNetCDF_cxx-6.2-gd.lib;
vtkoggtheora-6.2-gd.lib;
vtkParallelCore-6.2-gd.lib;
vtkpng-6.2-gd.lib;
vtkproj4-6.2-gd.lib;
vtkRenderingAnnotation-6.2-gd.lib;
vtkRenderingContext2D-6.2-gd.lib;
vtkRenderingContextOpenGL-6.2-gd.lib;
vtkRenderingCore-6.2-gd.lib;
vtkRenderingFreeType-6.2-gd.lib;
vtkRenderingFreeTypeOpenGL-6.2-gd.lib;
vtkRenderingGL2PS-6.2-gd.lib;
vtkRenderingImage-6.2-gd.lib;
vtkRenderingLabel-6.2-gd.lib;
vtkRenderingLIC-6.2-gd.lib;
vtkRenderingLOD-6.2-gd.lib;
vtkRenderingOpenGL-6.2-gd.lib;
vtkRenderingVolume-6.2-gd.lib;
vtkRenderingVolumeOpenGL-6.2-gd.lib;
vtksqlite-6.2-gd.lib;
vtksys-6.2-gd.lib;
vtktiff-6.2-gd.lib;
vtkverdict-6.2-gd.lib;
vtkViewsContext2D-6.2-gd.lib;
vtkViewsCore-6.2-gd.lib;
vtkViewsInfovis-6.2-gd.lib;
vtkzlib-6.2-gd.lib;
```

It is useful to note that only <pcl_common_debug.lib> and <pcl_io_debug.lib> are needed to run the pcd_write example on the PCL website.  

### Step 5: Test 
now place the following code in main.cpp
```
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int
  main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // Fill in the cloud data
  cloud.width    = 5;
  cloud.height   = 1;
  cloud.is_dense = false;
  cloud.points.resize (cloud.width * cloud.height);

  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
  std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;

  for (size_t i = 0; i < cloud.points.size (); ++i)
    std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;

  return (0);
}
```
