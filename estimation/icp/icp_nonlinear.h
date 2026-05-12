#pragma once

#include <vector>

#include "estimation/icp/icp.h"

namespace estimation::icp {
struct NonlinearICPSolver : public ICPSolver {
  void solve(const std::vector<Eigen::Vector3d,
                               Eigen::aligned_allocator<Eigen::Vector3d>>&
                 points3d_cam1,
             const std::vector<Eigen::Vector3d,
                               Eigen::aligned_allocator<Eigen::Vector3d>>&
                 points3d_cam2,
             Sophus::SE3d& c2_T_c1) const override;
};
}  // namespace estimation::icp
