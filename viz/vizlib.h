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

typedef std::vector<Isometry3d, aligned_allocator<Isometry3d>> poseVector;

struct pangolin_config{
    bool draw_pose_axes_ = true;

    constexpr pangolin_config& draw_pose_axes(const bool b){
        draw_pose_axes_ = b;
        return *this;
    }
};

void pangolin_draw(const poseVector& poses, const pangolin_config& pg_config); 