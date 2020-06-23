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

#include "open3d/pipelines/color_map/ColorMapOptimizer.h"

#include "open3d/camera/PinholeCameraTrajectory.h"
#include "open3d/geometry/Image.h"
#include "open3d/geometry/KDTreeFlann.h"
#include "open3d/geometry/RGBDImage.h"
#include "open3d/geometry/TriangleMesh.h"
#include "open3d/io/ClassIO/ImageWarpingFieldIO.h"
#include "open3d/io/ClassIO/PinholeCameraTrajectoryIO.h"
#include "open3d/pipelines/color_map/JacobianHelper.h"
#include "open3d/pipelines/color_map/NonRigidOptimizer.h"
#include "open3d/pipelines/color_map/RigidOptimizer.h"
#include "open3d/pipelines/color_map/TriangleMeshAndImageUtilities.h"
#include "open3d/utility/Console.h"
#include "open3d/utility/Eigen.h"

namespace open3d {
namespace pipelines {
namespace color_map {

void ColorMapOptimizer::CreateGradientImages() {
    utility::LogDebug("[ColorMapOptimization] :: CreateGradientImages");
    for (size_t i = 0; i < images_rgbd_.size(); i++) {
        auto gray_image = images_rgbd_[i]->color_.CreateFloatImage();
        auto gray_image_filtered =
                gray_image->Filter(geometry::Image::FilterType::Gaussian3);
        images_gray_.push_back(gray_image_filtered);
        images_dx_.push_back(gray_image_filtered->Filter(
                geometry::Image::FilterType::Sobel3Dx));
        images_dy_.push_back(gray_image_filtered->Filter(
                geometry::Image::FilterType::Sobel3Dy));
        auto color = std::make_shared<geometry::Image>(images_rgbd_[i]->color_);
        auto depth = std::make_shared<geometry::Image>(images_rgbd_[i]->depth_);
        images_color_.push_back(color);
        images_depth_.push_back(depth);
    }
}

ColorMapOptimizer::ColorMapOptimizer(
        const geometry::TriangleMesh& mesh,
        const std::vector<std::shared_ptr<geometry::RGBDImage>>& images_rgbd,
        const camera::PinholeCameraTrajectory& camera_trajectory)
    : mesh_(std::make_shared<geometry::TriangleMesh>(mesh)),
      images_rgbd_(images_rgbd),
      camera_trajectory_(std::make_shared<camera::PinholeCameraTrajectory>(
              camera_trajectory)) {
    // Fills images_gray_, images_dx_, images_dy_, images_color_, images_depth_
    CreateGradientImages();
}

void ColorMapOptimizer::RunRigidOptimization(
        int maximum_iteration,
        double maximum_allowable_depth,
        double depth_threshold_for_visibility_check,
        double depth_threshold_for_discontinuity_check,
        double half_dilation_kernel_size_for_discontinuity_map,
        int image_boundary_margin,
        int invisible_vertex_color_knn) {
    RigidOptimizer optimizer(mesh_, images_rgbd_, camera_trajectory_,
                             images_gray_, images_dx_, images_dy_,
                             images_color_, images_depth_);

    RigidOptimizerOption option;
    option.maximum_iteration_ = maximum_iteration;
    option.maximum_allowable_depth_ = maximum_allowable_depth;
    option.depth_threshold_for_visibility_check_ =
            depth_threshold_for_visibility_check;
    option.depth_threshold_for_discontinuity_check_ =
            depth_threshold_for_discontinuity_check;
    option.half_dilation_kernel_size_for_discontinuity_map_ =
            half_dilation_kernel_size_for_discontinuity_map;
    option.image_boundary_margin_ = image_boundary_margin;
    option.invisible_vertex_color_knn_ = invisible_vertex_color_knn;

    optimizer.Run(option);
}

void ColorMapOptimizer::RunNonRigidOptimization(
        int number_of_vertical_anchors,
        double non_rigid_anchor_point_weight,
        int maximum_iteration,
        double maximum_allowable_depth,
        double depth_threshold_for_visibility_check,
        double depth_threshold_for_discontinuity_check,
        double half_dilation_kernel_size_for_discontinuity_map,
        int image_boundary_margin,
        int invisible_vertex_color_knn) {
    NonRigidOptimizer optimizer(mesh_, images_rgbd_, camera_trajectory_,
                                images_gray_, images_dx_, images_dy_,
                                images_color_, images_depth_);
    optimizer.Run(number_of_vertical_anchors, non_rigid_anchor_point_weight,
                  maximum_iteration, maximum_allowable_depth,
                  depth_threshold_for_visibility_check,
                  depth_threshold_for_discontinuity_check,
                  half_dilation_kernel_size_for_discontinuity_map,
                  image_boundary_margin, invisible_vertex_color_knn);
}

}  // namespace color_map
}  // namespace pipelines
}  // namespace open3d
