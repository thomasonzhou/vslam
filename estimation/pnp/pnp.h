#pragma once
#include <sophus/se3.hpp>
#include <vector>

#include "geometry/calib.h"
#include "geometry/reprojection.h"

namespace estimation::pnp {

struct PnPSolver {
  virtual ~PnPSolver() = default;
  virtual void solve(
      const std::vector<Eigen::Vector3d,
                        Eigen::aligned_allocator<Eigen::Vector3d>>& points3d,
      const std::vector<Eigen::Vector2d,
                        Eigen::aligned_allocator<Eigen::Vector2d>>&
          points2d_img2,
      const geometry::PinholeCameraIntrinsics& intrinsics2,
      Sophus::SE3d& c2_T_c1) const = 0;
};

};  // namespace estimation::pnp
