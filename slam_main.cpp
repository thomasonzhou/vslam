#include "viz/vizlib.h"
#include "utils/utillib.h"
#include <string>
#include <vector>
#include <Eigen/Geometry>

using Eigen::Isometry3d;
using Eigen::aligned_allocator;
using Eigen::Quaterniond;

constexpr auto pg_config = pangolin_config()
    .draw_pose_axes(false);
const std::string estimated_traj = "../estimated.txt";
const std::string gt_traj = "../groundtruth.txt";

void visualize_trajectory() {
  poseVector estimated_poses = trajectory_from_file(estimated_traj);
  trajectory_view estimated_view(pg_config, estimated_poses);
  poseVector gt_poses = trajectory_from_file(gt_traj);
  trajectory_view gt_view(pg_config, gt_poses);
  pangolin_draw({estimated_view, gt_view});
}


int main(int argc, char** argv) {
  visualize_trajectory();

  return 0;
}
