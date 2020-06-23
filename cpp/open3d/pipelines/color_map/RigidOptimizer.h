// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#pragma once

#include <memory>
#include <vector>

#include "open3d/camera/PinholeCameraTrajectory.h"
#include "open3d/geometry/Image.h"
#include "open3d/geometry/RGBDImage.h"
#include "open3d/geometry/TriangleMesh.h"

namespace open3d {
namespace pipelines {
namespace color_map {

class RigidOptimizer {
public:
    RigidOptimizer(
            const std::shared_ptr<geometry::TriangleMesh>& mesh,
            const std::vector<std::shared_ptr<geometry::RGBDImage>>&
                    images_rgbd,
            const std::shared_ptr<camera::PinholeCameraTrajectory>&
                    camera_trajectory,
            const std::vector<std::shared_ptr<geometry::Image>>& images_gray,
            const std::vector<std::shared_ptr<geometry::Image>>& images_dx,
            const std::vector<std::shared_ptr<geometry::Image>>& images_dy,
            const std::vector<std::shared_ptr<geometry::Image>>& images_color,
            const std::vector<std::shared_ptr<geometry::Image>>& images_depth)
        : mesh_(mesh),
          images_rgbd_(images_rgbd),
          camera_trajectory_(camera_trajectory),
          images_gray_(images_gray),
          images_dx_(images_dx),
          images_dy_(images_dy),
          images_color_(images_color),
          images_depth_(images_depth) {}

    void Run(int maximum_iteration = 0,
             double maximum_allowable_depth = 2.5,
             double depth_threshold_for_visibility_check = 0.03,
             double depth_threshold_for_discontinuity_check = 0.1,
             double half_dilation_kernel_size_for_discontinuity_map = 3,
             int image_boundary_margin = 10,
             int invisible_vertex_color_knn = 3);

protected:
    std::shared_ptr<geometry::TriangleMesh> mesh_;
    std::vector<std::shared_ptr<geometry::RGBDImage>> images_rgbd_;
    std::shared_ptr<camera::PinholeCameraTrajectory> camera_trajectory_;
    std::vector<std::shared_ptr<geometry::Image>> images_gray_;
    std::vector<std::shared_ptr<geometry::Image>> images_dx_;
    std::vector<std::shared_ptr<geometry::Image>> images_dy_;
    std::vector<std::shared_ptr<geometry::Image>> images_color_;
    std::vector<std::shared_ptr<geometry::Image>> images_depth_;
};

}  // namespace color_map
}  // namespace pipelines
}  // namespace open3d
