#pragma once
#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#include <iostream>
#include <string>
#include <vector>

using Eigen::aligned_allocator;
using Eigen::Isometry3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

void pangolin_draw(
    const std::vector<Isometry3d, aligned_allocator<Isometry3d>>& poses); 