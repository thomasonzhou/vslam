#pragma once
#include "calib/calib.h"
#include <sophus/se3.hpp>

namespace geometry {

Eigen::Vector2d
reprojection_error(const Eigen::Vector3d &point3d_cam2,
                   const Eigen::Vector2d &point2d_img2,
                   const calib::PinholeCameraIntrinsics &intrinsics2);

double sum_of_squares_cost(
    const std::vector<Eigen::Vector3d,
                      Eigen::aligned_allocator<Eigen::Vector3d>> &points3d_cam1,
    const std::vector<Eigen::Vector2d,
                      Eigen::aligned_allocator<Eigen::Vector2d>> &points2d_img2,
    const calib::PinholeCameraIntrinsics &intrinsics2,
    const Sophus::SE3d &c2_T_c1);

Eigen::Matrix<double, 2, 6> error_jacobian_wrt_perturbation(
    const Eigen::Vector3d &point3d_cam2,
    const calib::PinholeCameraIntrinsics &intrinsics2);

Eigen::Vector2d
camera_to_pixel(const Eigen::Vector3d &point3d,
                const calib::PinholeCameraIntrinsics &intrinsics);
}; // namespace geometry