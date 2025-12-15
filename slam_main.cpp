#include "viz/vizlib.h"
#include "utils/utillib.h"
#include <string>
#include <vector>
#include <Eigen/Geometry>

using Eigen::Isometry3d;
using Eigen::aligned_allocator;
using Eigen::Quaterniond;

const auto est_config = pangolin_config()
    .trajectory_color("red");
const std::string est_traj = "../estimated.txt";
const auto gt_config = pangolin_config()
    .trajectory_color("blue");
const std::string gt_traj = "../groundtruth.txt";

void visualize_trajectory() {
  poseVector estimated_poses = trajectory_from_file(est_traj);
  trajectory_view estimated_view(est_config, estimated_poses);
  poseVector gt_poses = trajectory_from_file(gt_traj);
  trajectory_view gt_view(gt_config, gt_poses);
  pangolin_draw({estimated_view, gt_view});
}

int main(int argc, char** argv) {
  visualize_trajectory();

  return 0;
}
