#include "viz/vizlib.h"
#include "utils/utillib.h"
#include <string>
#include <vector>
#include <Eigen/Geometry>

using Eigen::Isometry3d;
using Eigen::aligned_allocator;
using Eigen::Quaterniond;

constexpr auto pg = pangolin_config().draw_pose_axes(false);

void visualize_trajectory() {
  std::string trajectory_file = "../trajectory.txt";
  poseVector poses = trajectory_from_file(trajectory_file);
  

  pangolin_draw(poses, pg);
}


int main(int argc, char** argv) {
  visualize_trajectory();

  return 0;
}
