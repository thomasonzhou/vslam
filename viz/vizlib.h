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

struct trajectory_view{
    const pangolin_config& pg_config;
    const poseVector& poses;

    trajectory_view(const pangolin_config& pg_config, const poseVector& poses): pg_config(pg_config), poses(poses) {};
};

void pangolin_draw(const std::vector<trajectory_view>& traj_view); 
void pangolin_draw(const trajectory_view& traj_view){
    pangolin_draw({traj_view});
}