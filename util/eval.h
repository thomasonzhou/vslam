#pragma once

#include <string>

#include "viz/vizlib.h"

namespace util {

const auto est_config = traj_viz_config{false, "red"};

const auto gt_config = traj_viz_config{false, "blue"};

void compare_trajectories(const std::string& est_traj,
                          const std::string& gt_traj, const bool align_traj);
}  // namespace util
