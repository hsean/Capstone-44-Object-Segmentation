# Building and Running PCL

These instructions should work on most unix like environments.  I tested on a mac.  I also tested on ubuntu, but the machine was too old, so the compiler used up all the memory and crashed the machine.  But the build script ran at least :)

### Step 0: Install prerequisites
QHull, Eigen, and Boost are required dependencies.  If you want to build the GUI/visualizer apps, you'll need VTK as well, but that can be skipped for now.


On Debian like systems (Ubuntu, Mint), for example, you would do the following:

```
sudo apt-get install libeigen3-dev
sudo apt-get install libboost-all-dev
sudo apt-get install libqhull-dev
```


### Step 1: clone the PCL source code
From [here](https://github.com/PointCloudLibrary/pcl), and then cd into wherever you cloned it.

### Step 2: checkout the tag for the latest stable release
```
git checkout tags/pcl-1.7.2
```

This is what I did. You may have luck building and running with the latest code from the master branch, but it seemed like running off the latest stable release was a reasonable thing to do.

### Step 3: Install cmake
```
sudo apt-get install cmake
```

### Step 4: Modify the CMakeLists.txt to create a new executable for testing
In your pcl sourcedir, there is a "segmentation" directory.  Within that directory, find the file "CMakeLists.txt", and replace it with the one [here](https://github.com/hsean/Capstone-44-Object-Segmentation/blob/master/pcl-build-notes/CMakeLists.txt).

### Step 5: generate build configuration
Create a build directory.  This is to prevent the source directory from becoming polluted with intermediate build files.

Then cd into your new build directory, and then run:

```
ccmake /path/back/to/your/pcl/source/dir
```

This will launch a command line "gui". Once that has come up,

In the gui, type 'c', for configure, then wait for it to finish.  Then type 'g' for generate, and wait some more.  Then when it's done, type 'e' to exit.

### Step 6: run the build
You should now have a Makefile at the root of your pcl-build directory. Run 'make', and then go have some coffee.  It will probably take 30-45 minutes to build.

### Step 7: Run it 
If your build was successful, you should have an executable called "cylinder_segmentation" at the top level of your pcl build directory.

Drop [this sample data file](https://github.com/hsean/Capstone-44-Object-Segmentation/blob/master/pcl-build-notes/table_scene_mug_stereo_textured.pcd) into the root of your pcl build directory, and run the cylinder_segmentation:

```
./cylinder_segmentation 
```

This should print out some cylinder coefficients. 


