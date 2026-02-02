#pragma once
#include "backend/calib.h"
#include <vector>
#include <sophus/se3.hpp>

void bundle_adjustment_gauss_newton(
    const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& points3d,
    const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>& points2d_img2,
    const PinholeCameraIntrinsics& intrinsics2,
    Sophus::SE3d& c2_T_c1
);

Eigen::Vector2d camera_to_pixel(const Eigen::Vector3d& point3d, const PinholeCameraIntrinsics& intrinsics){
    return Eigen::Vector2d(
        intrinsics.fx() * point3d[0] / point3d[2] + intrinsics.cx(),
        intrinsics.fy() * point3d[1] / point3d[2] + intrinsics.cy()
    );
}