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

#include <set>
#include <map>

#include <pcl/pcl_macros.h>
#include <pcl/common/colors.h>

template <typename PointT> bool
PointCloudIntensityHandler<PointT>::getColor (vtkSmartPointer<vtkDataArray> &scalars) const
{
  if (!capable_ || !cloud_)
    return (false);

  if (!scalars)
    scalars = vtkSmartPointer<vtkUnsignedCharArray>::New ();
  scalars->SetNumberOfComponents (3);
  
  const vtkIdType nr_points = cloud_->points.size ();
  reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->SetNumberOfTuples (nr_points);

  // Get a random color
  unsigned char* colors = new unsigned char[nr_points * 3];

  // Color every point
  for (vtkIdType cp = 0; cp < nr_points; ++cp)
  {
    const unsigned char intensity = static_cast<unsigned char> (cloud_->points[cp].intensity * 255);
    colors[cp * 3 + 0] = intensity;
    colors[cp * 3 + 1] = intensity;
    colors[cp * 3 + 2] = intensity;
  }
  reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->SetArray (colors, 3 * nr_points, 0, vtkUnsignedCharArray::VTK_DATA_ARRAY_DELETE);
  return (true);
}
