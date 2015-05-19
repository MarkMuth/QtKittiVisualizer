#include "KittiDataset.h"

#include <string>
#include <vector>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/format.hpp>

#include <pcl/filters/crop_box.h>
#include <pcl/common/io.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/search/kdtree.h>

#include <eigen3/Eigen/Core>

KittiDataset::KittiDataset(int dataset) :
    _dataset(dataset),
    _number_of_frames(0)
{
    if (!boost::filesystem::exists(KittiConfig::getPointCloudPath(_dataset)))
    {
        std::cerr << "Error in KittiDataset: Data set path "
                  << (KittiConfig::getPointCloudPath(_dataset)).string()
                  << " does not exist!" << std::endl;
        return;
    }
    if (!boost::filesystem::exists(KittiConfig::getPointCloudPath(_dataset, 0)))
    {
        std::cerr << "Error in KittiDataset: No point cloud was found at "
                  << (KittiConfig::getPointCloudPath(_dataset, 0)).string()
                  << std::endl;
        return;
    }
    if (!boost::filesystem::exists(KittiConfig::getTrackletsPath(_dataset)))
    {
        std::cerr << "Error in KittiDataset: No tracklets were found at "
                  << (KittiConfig::getTrackletsPath(_dataset)).string()
                  << std::endl;
        return;
    }

    initNumberOfFrames();
    initTracklets();
}

int KittiDataset::getNumberOfFrames()
{
    return _number_of_frames;
}

KittiPointCloud::Ptr KittiDataset::getPointCloud(int frameId)
{
    KittiPointCloud::Ptr cloud(new KittiPointCloud);
    std::fstream file(KittiConfig::getPointCloudPath(_dataset, frameId).c_str(), std::ios::in | std::ios::binary);
    if(file.good()){
        file.seekg(0, std::ios::beg);
        int i;
        for (i = 0; file.good() && !file.eof(); i++) {
            KittiPoint point;
            file.read((char *) &point.x, 3*sizeof(float));
            file.read((char *) &point.intensity, sizeof(float));
            cloud->push_back(point);
        }
        file.close();
    }
    return cloud;
}

KittiPointCloud::Ptr KittiDataset::getTrackletPointCloud(KittiPointCloud::Ptr& pointCloud, const KittiTracklet& tracklet, int frameId)
{
    int pose_number = frameId - tracklet.first_frame;
    Tracklets::tPose tpose = tracklet.poses.at(pose_number);

    Eigen::Vector4f minPoint(-tracklet.l / 2.0f, -tracklet.w / 2.0, -tracklet.h / 2.0, 1.0f);
    Eigen::Vector4f maxPoint( tracklet.l / 2.0f,  tracklet.w / 2.0,  tracklet.h / 2.0, 1.0f);
    Eigen::Vector3f boxTranslation((float) tpose.tx, (float) tpose.ty, (float) tpose.tz + tracklet.h / 2.0f);
    Eigen::Vector3f boxRotation((float) tpose.rx, (float) tpose.ry, (float) tpose.rz);

    KittiPointCloud::Ptr trackletPointCloud(new KittiPointCloud());
    pcl::CropBox<KittiPoint> cropFilter;
    cropFilter.setInputCloud(pointCloud);
    cropFilter.setMin(minPoint);
    cropFilter.setMax(maxPoint);
    cropFilter.setTranslation(boxTranslation);
    cropFilter.setRotation(boxRotation);
    cropFilter.filter(*trackletPointCloud);

    return trackletPointCloud;
}

Tracklets& KittiDataset::getTracklets()
{
    return _tracklets;
}

int KittiDataset::getLabel(const char* labelString)
{
    if (strcmp(labelString, "Car") == 0)
    {
        return 0;
    }
    if (strcmp(labelString, "Van") == 0)
    {
        return 1;
    }
    if (strcmp(labelString, "Truck") == 0)
    {
        return 2;
    }
    if (strcmp(labelString, "Pedestrian") == 0)
    {
        return 3;
    }
    if (strcmp(labelString, "Person (sitting)") == 0)
    {
        return 4;
    }
    if (strcmp(labelString, "Cyclist") == 0)
    {
        return 5;
    }
    if (strcmp(labelString, "Tram") == 0)
    {
        return 6;
    }
    if (strcmp(labelString, "Misc") == 0)
    {
        return 7;
    }
    std::cerr << "Error in KittiDataset:getLabel(): Not a valid label string: "
              << labelString << std::endl;
    return -1;
}

void KittiDataset::getColor(const char* labelString, int& r, int& g, int& b)
{
    if (strcmp(labelString, "Car") == 0)
    {
        r = 255; g = 0; b = 0;
        return;
    }
    if (strcmp(labelString, "Van") == 0)
    {
        r = 191; g = 0; b = 0;
        return;
    }
    if (strcmp(labelString, "Truck") == 0)
    {
        r = 127; g = 0; b = 0;
        return;
    }
    if (strcmp(labelString, "Pedestrian") == 0)
    {
        r = 0; g = 255; b = 0;
        return;
    }
    if (strcmp(labelString, "Person (sitting)") == 0)
    {
        r = 0; g = 128; b = 0;
        return;
    }
    if (strcmp(labelString, "Cyclist") == 0)
    {
        r = 0; g = 0; b = 255;
        return;
    }
    if (strcmp(labelString, "Tram") == 0)
    {
        r = 0; g = 255; b = 255;
        return;
    }
    if (strcmp(labelString, "Misc") == 0)
    {
        r = 255; g = 255; b = 0;
        return;
    }
    std::cerr << "Error in KittiDataset:getColor(): Not a valid label string: "
              << labelString << std::endl;
    r = 128; g = 128; b = 128;
}

void KittiDataset::getColor(int label, int& r, int& g, int& b)
{
    if (label == 0)
    {
        r = 255; g = 0; b = 0;
        return;
    }
    if (label == 1)
    {
        r = 191; g = 0; b = 0;
        return;
    }
    if (label == 2)
    {
        r = 127; g = 0; b = 0;
        return;
    }
    if (label == 3)
    {
        r = 0; g = 255; b = 0;
        return;
    }
    if (label == 4)
    {
        r = 0; g = 128; b = 0;
        return;
    }
    if (label == 5)
    {
        r = 0; g = 0; b = 255;
        return;
    }
    if (label == 6)
    {
        r = 0; g = 255; b = 255;
        return;
    }
    if (label == 7)
    {
        r = 255; g = 255; b = 0;
        return;
    }
    std::cerr << "Error in KittiDataset:getColor(): Not a valid label: "
              << label << std::endl;
    r = 128; g = 128; b = 128;
}

std::string KittiDataset::getLabelString(int label)
{
    if (label == 0)
    {
        return "Car";
    }
    if (label == 1)
    {
        return "Van";
    }
    if (label == 2)
    {
        return "Truck";
    }
    if (label == 3)
    {
        return "Pedestrian";
    }
    if (label == 4)
    {
        return "Person (sitting)";
    }
    if (label == 5)
    {
        return "Cyclist";
    }
    if (label == 6)
    {
        return "Tram";
    }
    if (label == 7)
    {
        return "Misc";
    }
    std::cerr << "Error in KittiDataset:getLabelString(): Not a valid label: "
              << label << std::endl;
    return "Unknown";
}

void KittiDataset::initNumberOfFrames()
{
    boost::filesystem::directory_iterator dit(KittiConfig::getPointCloudPath(_dataset));
    boost::filesystem::directory_iterator eit;

    while(dit != eit)
    {
        if(boost::filesystem::is_regular_file(*dit) && dit->path().extension() == ".bin")
        {
            _number_of_frames++;
        }
        ++dit;
    }
}

void KittiDataset::initTracklets()
{
    boost::filesystem::path trackletsPath = KittiConfig::getTrackletsPath(_dataset);
    _tracklets.loadFromFile(trackletsPath.string());
}

//#undef DEBUG_OUTPUT_ENABLED
