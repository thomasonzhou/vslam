#pragma once

#include "estimation/pnp/pnp.h"
#include "geometry/reprojection.h"

namespace estimation::pnp {

struct NaivePnPSolver : public PnPSolver {
  void solve(const std::vector<Eigen::Vector3d,
                               Eigen::aligned_allocator<Eigen::Vector3d>>&
                 points3d_cam1,
             const std::vector<Eigen::Vector2d,
                               Eigen::aligned_allocator<Eigen::Vector2d>>&
                 points2d_img2,
             const geometry::PinholeCameraIntrinsics& intrinsics2,
             Sophus::SE3d& c2_T_c1) const override {
    constexpr int max_iterations = 10;
    // pertubation norm threshold
    constexpr double kConvergenceEpsilon = 1e-6;

    // dogleg
    double delta = 1.0;
    constexpr double max_delta = 10.0;

    // Gauss Newton
    Eigen::Matrix<double, 6, 6> H_gn;
    Eigen::Matrix<double, 6, 1> b_gn;
    Eigen::Matrix<double, 6, 1> dx_gn;

    // steepest descent
    Eigen::Matrix<double, 6, 1> dx_sd;

    Eigen::Matrix<double, 6, 1> dx;

    for (int iter = 0; iter < max_iterations; ++iter) {
      H_gn = Eigen::Matrix<double, 6, 6>::Zero();
      b_gn = Eigen::Matrix<double, 6, 1>::Zero();

      double cost = 0.0;
      for (size_t i = 0; i < points3d_cam1.size(); ++i) {
        Eigen::Vector3d point3d_cam2 = c2_T_c1 * points3d_cam1[i];
        Eigen::Vector2d error =
            reprojection_error(point3d_cam2, points2d_img2[i], intrinsics2);
        cost += 0.5 * error.squaredNorm();

        Eigen::Matrix<double, 2, 6> J =
            geometry::jacobian_pixel_error_wrt_perturbation(point3d_cam2,
                                                            intrinsics2);

        H_gn += -J.transpose() * J;
        b_gn += J.transpose() * error;
      }

      dx_gn = -H_gn.ldlt().solve(b_gn);

      // case 1: full Gauss Newton step
      if (dx_gn.norm() < delta) {
        dx = dx_gn;
      } else {
        dx_sd = -(b_gn.squaredNorm()) / (b_gn.transpose() * H_gn * b_gn) * b_gn;
        // case 2: scale steepest descent to trust region boundary
        if (dx_sd.norm() >= delta) {
          dx = (delta / dx_sd.norm()) * dx_sd;
        }
        // case 3: dog leg, full steepest descent plus scaled Gauss Newton to
        // boundary
        else {
          const Eigen::Matrix<double, 6, 1> dx_dogleg = dx_gn - dx_sd;

          const double c1 = dx_dogleg.transpose() * dx_dogleg;
          const double c2 = 2.0 * dx_sd.transpose() * dx_dogleg;
          const double c3 = dx_sd.transpose() * dx_sd - delta * delta;

          const double tau =
              (-c2 + std::sqrt(c2 * c2 - 4.0 * c1 * c3)) / (2.0 * c1);

          dx = dx_sd + tau * dx_dogleg;
        }
      }

      if (dx.norm() < kConvergenceEpsilon) {
        std::cout << "converged, breaking" << std::endl;
        break;
      }

      const Sophus::SE3d candidate_pose = Sophus::SE3d::exp(dx) * c2_T_c1;
      const double candidate_cost = sum_of_squares_cost(
          points3d_cam1, points2d_img2, intrinsics2, candidate_pose);

      const double actual_reduction = cost - candidate_cost;
      // only the second term requires a negative, give, that b_gn already
      // accounts for the negative
      const double predicted_reduction =
          b_gn.dot(-dx) - 0.5 * dx.transpose() * H_gn * dx;

      const double gain_ratio = actual_reduction / predicted_reduction;

      // trust region update
      constexpr double tr_good_model_thresh = 0.75;
      constexpr double tr_poor_model_thresh = 0.25;
      constexpr double tr_can_expand_ratio = 0.8;
      constexpr double tr_scale = 2.0;
      if (gain_ratio > 0.0) {
        // loss decreased
        cost = candidate_cost;
        c2_T_c1 = candidate_pose;
        if (gain_ratio > tr_good_model_thresh &&
            dx.norm() >= tr_can_expand_ratio * delta) {
          delta = std::min(delta * tr_scale, max_delta);
        } else if (gain_ratio < tr_poor_model_thresh) {
          delta /= tr_scale;
        }
        std::cout << "lower cost " << cost << ", changing delta to " << delta
                  << std::endl;
      } else {
        delta /= tr_scale * tr_scale;
        std::cout << "cost not decreased from " << cost
                  << ", shrinking delta to " << delta << std::endl;
      }
    }
  }
};

};  // namespace estimation::pnp