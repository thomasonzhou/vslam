#pragma once
#include "backend/pnp.h"

namespace backend{

    struct G2OPnPSolver : public PnPSolver{
        virtual void solve(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& points3d,
        const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>& points2d_img2,
        const calib::PinholeCameraIntrinsics& intrinsics2,
        Sophus::SE3d& c2_T_c1) const override;
    };

};