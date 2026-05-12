#include "util/eval.h"

#include <cmath>
#include <sophus/se3.hpp>
#include <string>

#include "util/utillib.h"

namespace util {

void compare_trajectories(const std::string& est_traj,
                          const std::string& gt_traj, const bool align_traj) {
  poseVector est_poses = trajectory_from_file(est_traj);
  const poseVector gt_poses = trajectory_from_file(gt_traj);
  if (est_poses.size() == 0 || gt_poses.size() == 0) {
    return;
  }

  // find the tf from the first pose of est to gt
  if (align_traj) {
    const Sophus::SE3d T_gt_est = gt_poses[0] * est_poses[0].inverse();
    for (size_t i = 0; i < est_poses.size(); ++i) {
      est_poses[i] = T_gt_est * est_poses[i];
    }
  }
  trajectory_view est_view(est_config, est_poses);

  trajectory_view gt_view(gt_config, gt_poses);

  // gt vs estimate might be different sizes
  // double mse = 0.0;
  // for (size_t i = 0; i < gt_poses.size(); ++i) {
  //   const double error = (gt_poses[i].inverse() * est_poses[i]).log().norm();
  //   mse += error * error;
  // }
  // mse /= static_cast<double>(gt_poses.size());
  // const double rmse = std::sqrt(mse);
  // std::cout << "RMSE: " << rmse << std::endl;

  viz::pangolin_draw({est_view, gt_view});
}
}  // namespace util
