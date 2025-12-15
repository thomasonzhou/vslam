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
  std::string trajectory_file = "../estimated.txt";
  poseVector poses = trajectory_from_file(trajectory_file);
  std::string trajectory_file2 = "../groundtruth.txt";
  poseVector poses2 = trajectory_from_file(trajectory_file2);
  poses.insert(poses.end(), poses2.begin(), poses2.end());
  pangolin_draw(poses, pg);
}


int main(int argc, char** argv) {
  visualize_trajectory();

  return 0;
}
