#pragma once

#include <vector>
#include <Eigen/Geometry>

using Eigen::Isometry3d;
using Eigen::aligned_allocator;

typedef std::vector<Isometry3d, aligned_allocator<Isometry3d>> poseVector;

poseVector trajectory_from_file(const std::string& trajectory_file);