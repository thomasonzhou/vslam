#include "util/eval.h"
#include <string>

int main(int argc, char **argv) {

  const std::string est_traj = "../dataset-seq2_est.txt";
  const std::string gt_traj = "../dataset-seq2_poses.txt";
  constexpr bool kAlignTraj = true;
  util::compare_trajectories(est_traj, gt_traj, kAlignTraj);

  return 0;
}
