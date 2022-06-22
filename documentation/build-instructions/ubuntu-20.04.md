It turns out that it is incredibly difficult to find versions of Qt, VTK and PCL that work well together these days.

On Ubuntu 20.04 I managed to get QtKittiVisualizer running with the default Qt version of the OS.

However I had to compile VTK manually and even had to patch PCL before compiling.

In order to save you some time and trouble, here is what you need to do in order to produce a successful build on Ubunut 20.04.

Install `cmake`
===============

`sudo apt install cmake`

And checked the version:

`cmake --version`

`cmake version 3.16.3`


Install Qt Creator to have an IDE and Qt available.
===================================================

`sudo apt install qtcreator`

Check Qt version:

`qtdiag`

`Qt 5.12.8 (x86_64-little_endian-lp64 shared (dynamic) release build; by GCC 9.3.0) on "xcb" 
OS: Ubuntu 20.04.4 LTS [linux version 5.13.0-51-generic]`


Then build the required libraries from scratch:


Build VTK
=========

`sudo apt install build-essential cmake mesa-common-dev mesa-utils freeglut3-dev cmake-curses-gui qtdeclarative5-dev qtquickcontrols2-5-dev`

Seems like ninja is not available on Ubuntu 20.04 but recommended for install in the VTK docs.

`cd`

`git clone -b v9.1.0 --depth 1 https://gitlab.kitware.com/vtk/vtk vtk-9.1.0`

`mkdir vtk-9.1.0-build && cd $_`

`ccmake ../vtk-9.1.0`

Set VTK_GROUP_ENABLE_Qt to YES and exit ccmake by pressing e

`cmake ../vtk-9.1.0`

`make all`

Note:
QVTKOpenGLWidget was renamed to QVTKOpenGLStereoWidget in this version of VTK.


Build PCL
=========

`sudo apt install libbenchmark-dev`

Here I also needed to install several other packages like libboost etc. I don't have the exact names since the PC crashed one time compiling the PCL, which takes an incredible amount of RAM and time.

`cd`

`git clone -b pcl-1.12.1 --depth 1 https://github.com/PointCloudLibrary/pcl pcl-1.12.1`

`mkdir pcl-1.12.1-build && cd $_`

`cmake ../pcl-1.12.1 -DVTK_DIR=$HOME/vtk-9.1.0-build`

Apply the `link-required-vtk-libraries-for-libpcl_io.patch` for pcl from the QtKittiVisualizer source code. The patch is located in the `patches/pcl-1.12.1` directory.

It adds VTK::FiltersCore and VTK::FiltersGeneral to line 350 of io/CMakeLists

`make all`

Build QtKittiVisualizer
=======================

`cd`

`git clone https://github.com/MarkMuth/QtKittiVisualizer`

`mkdir QtKittiVisualizer-build && cd $_`

`cmake ../QtKittiVisualizer`

`make all`

Now you should have the `qt-kitti-visualizer` binary in your home folder. Make sure to download at least one dataset and extract the data to the right location. Please refer to `KittiConfig.cpp` to see were the datasets should be located or change that file as needed and recompile.

Then start QtKittiVisualizer:

./qt-kitti-visulizer --dataset=5

The visualization might be hidden. Try to drag the splitter from the very right side of the window to see the render window.
