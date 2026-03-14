#pragma once

#include <Eigen/Core>
#include <sophus/se3.hpp>

namespace backend{
    void point_to_point_svd(
    const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &points3d_cam1,
    const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &points3d_cam2,
    Sophus::SE3d& c2_T_c1
    );
} // namespace backend