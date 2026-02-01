#pragma once

#include <vector>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>

using Eigen::aligned_allocator;

typedef std::vector<Sophus::SE3d, aligned_allocator<Sophus::SE3d>> poseVector;

poseVector trajectory_from_file(const std::string& trajectory_file);

