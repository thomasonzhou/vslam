#pragma once

#include "viz/vizlib.h"
namespace util {

    const auto est_config = pangolin_config().trajectory_color("red");
    
    const auto gt_config = pangolin_config().trajectory_color("blue");

    void compare_trajectories(const std::string &est_traj, const std::string &gt_traj);
}  // namespace util