#include "viz/vizlib.h"
#include <string>
#include <vector>
#include <Eigen/Geometry>

using Eigen::Isometry3d;
using Eigen::aligned_allocator;
using Eigen::Quaterniond;

constexpr auto pg = pangolin_config().draw_pose_axes(false);

void visualize_trajectory() {
  std::string trajectory_file = "../trajectory.txt";

  std::vector<Isometry3d, aligned_allocator<Isometry3d>> poses;

  std::ifstream fin(trajectory_file);
  if (!fin) {
    std::cout << "file not found: " << trajectory_file << std::endl;
    return;
  }

  double time, tx, ty, tz, qx, qy, qz, qw;
  while (fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw) {
    Isometry3d Twr(Quaterniond(qw, qx, qy, qz));
    Twr.pretranslate(Vector3d(tx, ty, tz));
    poses.push_back(Twr);
  }
  std::cout << poses.size() << " poses" << std::endl;

  pangolin_draw(poses, pg);
}


int main(int argc, char** argv) {
  visualize_trajectory();

  return 0;
}
