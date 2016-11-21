/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */
#pragma once

// PCL includes
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/visualization/common/common.h>
// VTK includes
#include <vtkSmartPointer.h>
#include <vtkDataArray.h>
#include <vtkFloatArray.h>
#include <vtkUnsignedCharArray.h>

template <typename PointT>
class PointCloudIntensityHandler : public pcl::visualization::PointCloudColorHandler<PointT>
{
    typedef typename pcl::visualization::PointCloudColorHandler<PointT>::PointCloud PointCloud;
    typedef typename pcl::visualization::PointCloudColorHandler<PointT>::PointCloud::Ptr PointCloudPtr;
    typedef typename pcl::visualization::PointCloudColorHandler<PointT>::PointCloud::ConstPtr PointCloudConstPtr;

    public:
    typedef boost::shared_ptr<PointCloudIntensityHandler<PointT> > Ptr;
    typedef boost::shared_ptr<const PointCloudIntensityHandler<PointT> > ConstPtr;

    /** \brief Constructor. */
    PointCloudIntensityHandler () :
        PointCloudIntensityHandler<PointT> ()
    {
        capable_ = true;
    }

    /** \brief Constructor. */
    PointCloudIntensityHandler (const PointCloudConstPtr &cloud) :
        pcl::visualization::PointCloudColorHandler<PointT> (cloud)
    {
        capable_ = true;
    }

    /** \brief Abstract getName method. */
    virtual std::string
        getName () const { return ("PointCloudIntensityHandler"); }

    /** \brief Get the name of the field used. */
    virtual std::string
        getFieldName () const { return (""); }

    /** \brief Obtain the actual color for the input dataset as vtk scalars.
     * \param[out] scalars the output scalars containing the color for the dataset
     * \return true if the operation was successful (the handler is capable and 
     * the input cloud was given as a valid pointer), false otherwise
     */
    virtual bool
        getColor (vtkSmartPointer<vtkDataArray> &scalars) const;

    protected:
    // Members derived from the base class
    using pcl::visualization::PointCloudColorHandler<PointT>::cloud_;
    using pcl::visualization::PointCloudColorHandler<PointT>::capable_;
};

#include <point_cloud_intensity_handler.hpp>
