#pragma once

#include "viz/vizlib.h"

const auto est_config = pangolin_config()
    .trajectory_color("red");
const std::string est_traj = "../estimated.txt";

const auto gt_config = pangolin_config()
    .trajectory_color("blue");
const std::string gt_traj = "../groundtruth.txt";

void compare_trajectories();