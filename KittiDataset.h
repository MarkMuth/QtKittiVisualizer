/*

Copyright 2016 Mark Muth

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

*/

#ifndef KITTIDATASET_H
#define KITTIDATASET_H

#include <string>

#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "KittiConfig.h"

#include "kitti-devkit-raw/tracklets.h"

typedef pcl::PointXYZI KittiPoint;
typedef pcl::PointCloud<KittiPoint> KittiPointCloud;
typedef Tracklets::tTracklet KittiTracklet;

class KittiDataset
{

public:

    KittiDataset();
    KittiDataset(int dataset);
    int getNumberOfFrames();
    KittiPointCloud::Ptr getPointCloud(int frameId);
    KittiPointCloud::Ptr getTrackletPointCloud(KittiPointCloud::Ptr& pointCloud, const KittiTracklet& tracklet, int frameId);
    Tracklets& getTracklets();

    static int getLabel(const char* labelString);
    static void getColor(const char* labelString, int& r, int& g, int& b);
    static void getColor(int label, int& r, int& g, int& b);
    static std::string getLabelString(int label);

private:

    int _dataset;
    int _number_of_frames;
    /** Counts the number of files with an ".bin" extension in the point cloud path */
    void initNumberOfFrames();

    Tracklets _tracklets;
    void initTracklets();
};

#endif // KITTIDATASET_H
