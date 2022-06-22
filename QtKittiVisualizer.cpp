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

#include "QtKittiVisualizer.h"
#include "ui_QtKittiVisualizer.h"

#include <string>

#include <QCheckBox>
#include <QLabel>
#include <QMainWindow>
#include <QSlider>
#include <QWidget>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <eigen3/Eigen/Core>

#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>

#include <KittiConfig.h>
#include <KittiDataset.h>

#include <kitti-devkit-raw/tracklets.h>

typedef pcl::visualization::PointCloudColorHandlerCustom<KittiPoint> KittiPointCloudColorHandlerCustom;

KittiVisualizerQt::KittiVisualizerQt(QWidget *parent, int argc, char** argv)
    : QMainWindow(parent)
    , ui(new Ui::KittiVisualizerQt)
    , dataset_index(0)
    , frame_index(0)
    , tracklet_index(0)
    , pclVisualizer(nullptr)
    , pointCloudVisible(true)
    , trackletBoundingBoxesVisible(true)
    , trackletPointsVisible(true)
    , trackletInCenterVisible(true)
{
    int invalidOptions = parseCommandLineOptions(argc, argv);
    if (invalidOptions)
    {
        exit(invalidOptions);
    }

    // Set up user interface
    ui->setupUi(this);

    vtkNew<vtkGenericOpenGLRenderWindow> renderWindow;
    vtkNew<vtkRenderer> renderer;
    renderWindow->AddRenderer(renderer);

    pclVisualizer.reset(
        new pcl::visualization::PCLVisualizer(
             renderer, renderWindow,
             "pclVisualizer", false));

    ui->qvtkWidget_pclViewer->setRenderWindow(
        pclVisualizer->getRenderWindow());
    pclVisualizer->setupInteractor(
        ui->qvtkWidget_pclViewer->GetInteractor(),
        ui->qvtkWidget_pclViewer->GetRenderWindow());
    pclVisualizer->setBackgroundColor(0, 0, 0);
    pclVisualizer->addCoordinateSystem(1.0);
    pclVisualizer->registerKeyboardCallback(
        &KittiVisualizerQt::keyboardEventOccurred, *this, 0);
    this->setWindowTitle("Qt KITTI Visualizer");
    ui->qvtkWidget_pclViewer->update();

    // Init the viewer with the first point cloud and corresponding tracklets
    dataset = new KittiDataset(KittiConfig::availableDatasets.at(dataset_index));
    loadPointCloud();
    if (pointCloudVisible)
        showPointCloud();
    loadAvailableTracklets();
    if (trackletBoundingBoxesVisible)
        showTrackletBoxes();
    loadTrackletPoints();
    if (trackletPointsVisible)
        showTrackletPoints();
    if (trackletInCenterVisible)
        showTrackletInCenter();

    ui->slider_dataSet->setRange(0, KittiConfig::availableDatasets.size() - 1);
    ui->slider_dataSet->setValue(dataset_index);
    ui->slider_frame->setRange(0, dataset->getNumberOfFrames() - 1);
    ui->slider_frame->setValue(frame_index);
    if (availableTracklets.size() != 0)
        ui->slider_tracklet->setRange(0, availableTracklets.size() - 1);
    else
        ui->slider_tracklet->setRange(0, 0);
    ui->slider_tracklet->setValue(tracklet_index);

    updateDatasetLabel();
    updateFrameLabel();
    updateTrackletLabel();

    // Connect signals and slots
    connect(ui->slider_dataSet,  SIGNAL (valueChanged(int)), this, SLOT (newDatasetRequested(int)));
    connect(ui->slider_frame,    SIGNAL (valueChanged(int)), this, SLOT (newFrameRequested(int)));
    connect(ui->slider_tracklet, SIGNAL (valueChanged(int)), this, SLOT (newTrackletRequested(int)));
    connect(ui->checkBox_showFramePointCloud,       SIGNAL (toggled(bool)), this, SLOT (showFramePointCloudToggled(bool)));
    connect(ui->checkBox_showTrackletBoundingBoxes, SIGNAL (toggled(bool)), this, SLOT (showTrackletBoundingBoxesToggled(bool)));
    connect(ui->checkBox_showTrackletPointClouds,   SIGNAL (toggled(bool)), this, SLOT (showTrackletPointCloudsToggled(bool)));
    connect(ui->checkBox_showTrackletInCenter,      SIGNAL (toggled(bool)), this, SLOT (showTrackletInCenterToggled(bool)));
}

KittiVisualizerQt::~KittiVisualizerQt()
{
    delete dataset;
    delete ui;
}

int KittiVisualizerQt::parseCommandLineOptions(int argc, char** argv)
{
    // Declare the supported options.
    boost::program_options::options_description desc("Program options");
    desc.add_options()
        ("help", "Produce this help message.")
        ("dataset", boost::program_options::value<int>(), "Set the number of the KITTI data set to be used.")
    ;

    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }

    if (vm.count("dataset")) {
        dataset_index = vm["dataset"].as<int>();
        std::cout << "Using data set " << dataset_index << "." << std::endl;
        dataset_index = KittiConfig::getDatasetIndex(dataset_index);
    } else {
        dataset_index = 0;
        std::cout << "Data set was not specified." << std::endl;
        std::cout << "Using data set " << KittiConfig::getDatasetNumber(dataset_index) << "." << std::endl;
    }
    return 0;
}

bool KittiVisualizerQt::loadNextFrame()
{
    newFrameRequested(frame_index + 1);
    return false;
}

bool KittiVisualizerQt::loadPreviousFrame()
{
    newFrameRequested(frame_index - 1);
    return false;
}

void KittiVisualizerQt::getTrackletColor(const KittiTracklet& tracklet, int &r, int& g, int& b)
{
    KittiDataset::getColor(tracklet.objectType.c_str(), r, g, b);
}

void KittiVisualizerQt::showFramePointCloudToggled(bool value)
{
    pointCloudVisible = value;
    if (pointCloudVisible)
    {
        showPointCloud();
    }
    else
    {
        hidePointCloud();
    }
    ui->qvtkWidget_pclViewer->update();
}

void KittiVisualizerQt::newDatasetRequested(int value)
{
    if (dataset_index == value)
        return;

    if (trackletInCenterVisible)
        hideTrackletInCenter();
    if (trackletPointsVisible)
        hideTrackletPoints();
    clearTrackletPoints();
    if (trackletBoundingBoxesVisible)
        hideTrackletBoxes();
    clearAvailableTracklets();
    if (pointCloudVisible)
        hidePointCloud();

    dataset_index = value;
    if (dataset_index >= KittiConfig::availableDatasets.size())
        dataset_index = KittiConfig::availableDatasets.size() - 1;
    if (dataset_index < 0)
        dataset_index = 0;

    delete dataset;
    dataset = new KittiDataset(KittiConfig::availableDatasets.at(dataset_index));

    if (frame_index >= dataset->getNumberOfFrames())
        frame_index = dataset->getNumberOfFrames() - 1;

    loadPointCloud();
    if (pointCloudVisible)
        showPointCloud();
    loadAvailableTracklets();
    if (trackletBoundingBoxesVisible)
        showTrackletBoxes();
    loadTrackletPoints();
    if (trackletPointsVisible)
        showTrackletPoints();
    if (tracklet_index >= availableTracklets.size())
        tracklet_index = availableTracklets.size() - 1;
    if (tracklet_index < 0)
        tracklet_index = 0;
    if (trackletInCenterVisible)
        showTrackletInCenter();

    ui->slider_frame->setRange(0, dataset->getNumberOfFrames() - 1);
    ui->slider_frame->setValue(frame_index);
    if (availableTracklets.size() != 0)
        ui->slider_tracklet->setRange(0, availableTracklets.size() - 1);
    else
        ui->slider_tracklet->setRange(0, 0);
    ui->slider_tracklet->setValue(tracklet_index);

    updateDatasetLabel();
    updateFrameLabel();
    updateTrackletLabel();
    ui->qvtkWidget_pclViewer->update();
}

void KittiVisualizerQt::newFrameRequested(int value)
{
    if (frame_index == value)
        return;

    if (trackletInCenterVisible)
        hideTrackletInCenter();
    if (trackletPointsVisible)
        hideTrackletPoints();
    clearTrackletPoints();
    if (trackletBoundingBoxesVisible)
        hideTrackletBoxes();
    clearAvailableTracklets();
    if (pointCloudVisible)
        hidePointCloud();

    frame_index = value;
    if (frame_index >= dataset->getNumberOfFrames())
        frame_index = dataset->getNumberOfFrames() - 1;
    if (frame_index < 0)
        frame_index = 0;

    loadPointCloud();
    if (pointCloudVisible)
        showPointCloud();
    loadAvailableTracklets();
    if (trackletBoundingBoxesVisible)
        showTrackletBoxes();
    loadTrackletPoints();
    if (trackletPointsVisible)
        showTrackletPoints();
    if (tracklet_index >= availableTracklets.size())
        tracklet_index = availableTracklets.size() - 1;
    if (tracklet_index < 0)
        tracklet_index = 0;
    if (trackletInCenterVisible)
        showTrackletInCenter();

    if (availableTracklets.size() != 0)
        ui->slider_tracklet->setRange(0, availableTracklets.size() - 1);
    else
        ui->slider_tracklet->setRange(0, 0);
    ui->slider_tracklet->setValue(tracklet_index);

    updateFrameLabel();
    updateTrackletLabel();
    ui->qvtkWidget_pclViewer->update();
}

void KittiVisualizerQt::newTrackletRequested(int value)
{
    if (tracklet_index == value)
        return;

    if (trackletInCenterVisible)
        hideTrackletInCenter();

    tracklet_index = value;
    if (tracklet_index >= availableTracklets.size())
        tracklet_index = availableTracklets.size() - 1;
    if (tracklet_index < 0)
        tracklet_index = 0;
    if (trackletInCenterVisible)
        showTrackletInCenter();

    updateTrackletLabel();
    ui->qvtkWidget_pclViewer->update();
}

void KittiVisualizerQt::showTrackletBoundingBoxesToggled(bool value)
{
    trackletBoundingBoxesVisible = value;
    if (trackletBoundingBoxesVisible)
    {
        showTrackletBoxes();
    }
    else
    {
        hideTrackletBoxes();
    }
    ui->qvtkWidget_pclViewer->update();
}

void KittiVisualizerQt::showTrackletPointCloudsToggled(bool value)
{
    trackletPointsVisible = value;
    if (trackletPointsVisible)
    {
        showTrackletPoints();
    }
    else
    {
        hideTrackletPoints();
    }
    ui->qvtkWidget_pclViewer->update();
}

void KittiVisualizerQt::showTrackletInCenterToggled(bool value)
{
    trackletInCenterVisible = value;
    if (trackletInCenterVisible)
    {
        showTrackletInCenter();
    }
    else
    {
        hideTrackletInCenter();
    }
    ui->qvtkWidget_pclViewer->update();
}

void KittiVisualizerQt::loadPointCloud()
{
    pointCloud = dataset->getPointCloud(frame_index);
}

void KittiVisualizerQt::showPointCloud()
{
    KittiPointCloudColorHandlerCustom colorHandler(pointCloud, 255, 255, 255);
    pclVisualizer->addPointCloud<KittiPoint>(pointCloud, colorHandler, "point_cloud");
}

void KittiVisualizerQt::hidePointCloud()
{
    pclVisualizer->removePointCloud("point_cloud");
}

void KittiVisualizerQt::loadAvailableTracklets()
{
    Tracklets& tracklets = dataset->getTracklets();
    int tracklet_id;
    int number_of_tracklets = tracklets.numberOfTracklets();
    for (tracklet_id = 0; tracklet_id < number_of_tracklets; tracklet_id++)
    {
        KittiTracklet* tracklet = tracklets.getTracklet(tracklet_id);
        if (tracklet->first_frame <= frame_index && tracklet->lastFrame() >= frame_index)
        {
            availableTracklets.push_back(*tracklet);
        }
    }
}

void KittiVisualizerQt::clearAvailableTracklets()
{
    availableTracklets.clear();
}

void KittiVisualizerQt::updateDatasetLabel()
{
    std::stringstream text;
    text << "Data set: "
         << dataset_index + 1 << " of " << KittiConfig::availableDatasets.size()
         << " [" << KittiConfig::getDatasetNumber(dataset_index) << "]"
         << std::endl;
    ui->label_dataSet->setText(text.str().c_str());
}

void KittiVisualizerQt::updateFrameLabel()
{
    std::stringstream text;
    text << "Frame: "
         << frame_index + 1 << " of " << dataset->getNumberOfFrames()
         << std::endl;
    ui->label_frame->setText(text.str().c_str());

}

void KittiVisualizerQt::updateTrackletLabel()
{
    if (availableTracklets.size())
    {
        std::stringstream text;
        KittiTracklet tracklet = availableTracklets.at(tracklet_index);
        text << "Tracklet: "
             << tracklet_index + 1 << " of " << availableTracklets.size()
             << " (\"" << tracklet.objectType
             << "\", " << croppedTrackletPointClouds.at(tracklet_index).get()->size()
             << " points)"
             << std::endl;
        ui->label_tracklet->setText(text.str().c_str());
    }
    else
    {
        std::stringstream text;
        text << "Tracklet: "
             << "0 of 0"
             << std::endl;
        ui->label_tracklet->setText(text.str().c_str());
    }
}

void KittiVisualizerQt::showTrackletBoxes()
{
    double boxHeight = 0.0d;
    double boxWidth = 0.0d;
    double boxLength = 0.0d;
    int pose_number = 0;

    for (int i = 0; i < availableTracklets.size(); ++i)
    {
        // Create the bounding box
        const KittiTracklet& tracklet = availableTracklets.at(i);

        boxHeight = tracklet.h;
        boxWidth = tracklet.w;
        boxLength = tracklet.l;
        pose_number = frame_index - tracklet.first_frame;
        const Tracklets::tPose& tpose = tracklet.poses.at(pose_number);
        Eigen::Vector3f boxTranslation;
        boxTranslation[0] = (float) tpose.tx;
        boxTranslation[1] = (float) tpose.ty;
        boxTranslation[2] = (float) tpose.tz + (float) boxHeight / 2.0f;
        Eigen::Quaternionf boxRotation = Eigen::Quaternionf(Eigen::AngleAxisf((float) tpose.rz, Eigen::Vector3f::UnitZ()));

        // Add the bounding box to the visualizer
        std::string viewer_id = "tracklet_box_" + i;
        pclVisualizer->addCube(boxTranslation, boxRotation, boxLength, boxWidth, boxHeight, viewer_id);
    }
}

void KittiVisualizerQt::hideTrackletBoxes()
{
    for (int i = 0; i < availableTracklets.size(); ++i)
    {
        std::string viewer_id = "tracklet_box_" + i;
        pclVisualizer->removeShape(viewer_id);
    }
}

void KittiVisualizerQt::loadTrackletPoints()
{
    for (int i = 0; i < availableTracklets.size(); ++i)
    {
        // Create the tracklet point cloud
        const KittiTracklet& tracklet = availableTracklets.at(i);
        pcl::PointCloud<KittiPoint>::Ptr trackletPointCloud = dataset->getTrackletPointCloud(pointCloud, tracklet, frame_index);
        pcl::PointCloud<KittiPoint>::Ptr trackletPointCloudTransformed(new pcl::PointCloud<KittiPoint>);

        Eigen::Vector3f transformOffset;
        transformOffset[0] = 0.0f;
        transformOffset[1] = 0.0f;
        transformOffset[2] = 6.0f;
        pcl::transformPointCloud(*trackletPointCloud, *trackletPointCloudTransformed, transformOffset, Eigen::Quaternionf::Identity());

        // Store the tracklet point cloud
        croppedTrackletPointClouds.push_back(trackletPointCloudTransformed);
    }
}

void KittiVisualizerQt::showTrackletPoints()
{
    for (int i = 0; i < availableTracklets.size(); ++i)
    {
        // Create a color handler for the tracklet point cloud
        const KittiTracklet& tracklet = availableTracklets.at(i);
        int r, g, b;
        getTrackletColor(tracklet, r, g, b);
        KittiPointCloudColorHandlerCustom colorHandler(croppedTrackletPointClouds.at(i), r, g, b);

        // Add tracklet point cloud to the visualizer
        std::string viewer_id = "cropped_tracklet_" + i;
        pclVisualizer->addPointCloud<KittiPoint>(croppedTrackletPointClouds.at(i), colorHandler, viewer_id);
    }
}

void KittiVisualizerQt::hideTrackletPoints()
{
    for (int i = 0; i < availableTracklets.size(); ++i)
    {
        std::string viewer_id = "cropped_tracklet_" + i;
        pclVisualizer->removeShape(viewer_id);
    }
}

void KittiVisualizerQt::clearTrackletPoints()
{
    croppedTrackletPointClouds.clear();
}

void KittiVisualizerQt::showTrackletInCenter()
{
    if (availableTracklets.size())
    {
        // Create the centered tracklet point cloud
        const KittiTracklet& tracklet = availableTracklets.at(tracklet_index);
        pcl::PointCloud<KittiPoint>::Ptr cloudOut = dataset->getTrackletPointCloud(pointCloud, tracklet, frame_index);
        pcl::PointCloud<KittiPoint>::Ptr cloudOutTransformed(new pcl::PointCloud<KittiPoint>);

        int pose_number = frame_index - tracklet.first_frame;
        Tracklets::tPose tpose = tracklet.poses.at(pose_number);

        Eigen::Vector3f transformOffset((float) -(tpose.tx), (float) -(tpose.ty), (float) -(tpose.tz + tracklet.h / 2.0));

        Eigen::AngleAxisf angleAxisZ(-tpose.rz, Eigen::Vector3f::UnitZ());
        Eigen::Quaternionf transformRotation(angleAxisZ);
        pcl::transformPointCloud(*cloudOut, *cloudOutTransformed, transformOffset, Eigen::Quaternionf::Identity());
        pcl::transformPointCloud(*cloudOutTransformed, *cloudOut, Eigen::Vector3f::Zero(), transformRotation);

        // Create color handler for the centered tracklet point cloud
        KittiPointCloudColorHandlerCustom colorHandler(cloudOut, 0, 255, 0);

        // Add the centered tracklet point cloud to the visualizer
        pclVisualizer->addPointCloud<KittiPoint>(cloudOut, colorHandler, "centered_tracklet");
    }
}

void KittiVisualizerQt::hideTrackletInCenter()
{
    if (availableTracklets.size())
    {
        pclVisualizer->removeShape("centered_tracklet");
    }
}

void KittiVisualizerQt::setFrameNumber(int frameNumber)
{
    frame_index = frameNumber;
}

void KittiVisualizerQt::keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                                             void* viewer_void)
{
    if (event.getKeyCode() == 0 && event.keyDown())
    {
        if (event.getKeySym() == "Left")
        {
            loadPreviousFrame();
        }
        else if (event.getKeySym() == "Right")
        {
            loadNextFrame();
        }
    }
}
