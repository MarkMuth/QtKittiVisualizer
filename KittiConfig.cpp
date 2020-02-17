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

#include "KittiConfig.h"

#include <string>
#include <vector>
#include <iostream>

#include <boost/filesystem/path.hpp>
#include <boost/format.hpp>

//std::string KittiConfig::data_directory = "../KittiData";
std::string KittiConfig::data_directory = "D:\\KITTI\\2011_09_26";
std::string KittiConfig::raw_data_directory = "";
std::string KittiConfig::dataset_folder_template = "%|04|_sync";
std::string KittiConfig::point_cloud_directory = "velodyne_points\\data";
std::string KittiConfig::point_cloud_file_template = "%|010|.bin";
std::string KittiConfig::tracklets_directory = ".";
std::string KittiConfig::tracklets_file_name = "tracklet_labels.xml";

const std::vector<int> KittiConfig::availableDatasets = KittiConfig::initAvailableDatasets();

boost::filesystem::path KittiConfig::getPointCloudPath(int dataset)
{
    return boost::filesystem::path(data_directory)
            / raw_data_directory
            / (boost::format(dataset_folder_template) % dataset).str()
            / point_cloud_directory
            ;
}

boost::filesystem::path KittiConfig::getPointCloudPath(int dataset, int frameId)
{
    return getPointCloudPath(dataset)
            / (boost::format(point_cloud_file_template) % frameId).str()
            ;
}

boost::filesystem::path KittiConfig::getTrackletsPath(int dataset)
{
    return boost::filesystem::path(data_directory)
            / raw_data_directory
            / (boost::format(dataset_folder_template) % dataset).str()
            / tracklets_directory
            / tracklets_file_name
            ;
}

int KittiConfig::getDatasetNumber(int index)
{
    if (index >= 0 && index < availableDatasets.size())
    {
        return availableDatasets.at(index);
    }
    std::cerr << "No such data set index: " << index << std::endl;
    return 0;
}

int KittiConfig::getDatasetIndex(int number)
{
    for (int i = 0; i < availableDatasets.size(); ++i)
    {
        if (availableDatasets.at(i) == number)
        {
            return i;
        }
    }
    std::cerr << "No such data set number: " << number << std::endl;
    return 0;
}

std::vector<int> KittiConfig::initAvailableDatasets()
{
    std::vector<int> datasets;
    datasets.push_back(1);
    datasets.push_back(2);
    datasets.push_back(5);
    datasets.push_back(9);
    datasets.push_back(11);
    datasets.push_back(13);
    datasets.push_back(14);
    datasets.push_back(17);
    datasets.push_back(18);
    datasets.push_back(48);
    datasets.push_back(51);
    datasets.push_back(56);
    datasets.push_back(57);
    datasets.push_back(59);
    datasets.push_back(60);
    datasets.push_back(84);
    datasets.push_back(91);
    datasets.push_back(93);
    return datasets;
}
