#include "viz/vizlib.h"
#include "utils/utillib.h"
#include <string>
#include <sophus/se3.hpp>
#include <cmath>

const auto est_config = pangolin_config()
    .trajectory_color("red");
const std::string est_traj = "../estimated.txt";

const auto gt_config = pangolin_config()
    .trajectory_color("blue");
const std::string gt_traj = "../groundtruth.txt";

void compare_trajectories() {
  poseVector est_poses = trajectory_from_file(est_traj);
  trajectory_view est_view(est_config, est_poses);

  poseVector gt_poses = trajectory_from_file(gt_traj);
  trajectory_view gt_view(gt_config, gt_poses);

  double mse = 0.0;
  for (size_t i = 0; i < gt_poses.size(); ++i){
    const double error = (gt_poses[i].inverse() * est_poses[i]).log().norm();
    mse += error * error;
  }
  mse /= static_cast<double>(gt_poses.size());
  const double rmse = std::sqrt(mse);
  std::cout << "RMSE: " << rmse << std::endl;

  pangolin_draw({est_view, gt_view});
}

int main(int argc, char** argv) {
  compare_trajectories();

  return 0;
}
