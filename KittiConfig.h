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

#ifndef KITTICONFIG_H
#define KITTICONFIG_H

#include <string>
#include <vector>

#include <boost/filesystem/path.hpp>

/**
 * @brief The KittiConfig class
 *
 * Define where the KITTI data sets are stored. Templated strings provide for a
 * high flexibility in storing and accessing your files.
 *
 * The predefined values assume the following filesystem hierarchy:
 *
 * /QtKittiVisualizer (source)
 * /QtKittiVisualizer-build (working directory)
 * /KittiData
 *   /raw
 *     /%|04|_sync (datasets, e.g. 0001_sync)
 *       /velodyne_points
 *         /data
 *           /%|010|.bin (point clouds, e.g. 0000000000.bin)
 *       /tracklet_labels.xml (tracklets)
 *
 * You can change the predefined values to your needs in KittiConfig.cpp.
 */
class KittiConfig
{

public:

    static boost::filesystem::path getPointCloudPath(int dataset);
    static boost::filesystem::path getPointCloudPath(int dataset,int frameId);
    static boost::filesystem::path getTrackletsPath(int dataset);

    /** Contains the numbers of data sets available from your data set folder */
    static const std::vector<int> availableDatasets;
    static int getDatasetNumber(int index);
    static int getDatasetIndex(int number);

private:


    // Following variables describe the filesystem hierarchy of the KITTI dataset
    static std::string data_directory;
    static std::string raw_data_directory;
    static std::string dataset_folder_template;
    static std::string point_cloud_directory;
    static std::string point_cloud_file_template;
    static std::string tracklets_directory;
    static std::string tracklets_file_name;

    static std::vector<int> initAvailableDatasets();
};

#endif // KITTICONFIG_H
