#pragma once
#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <chrono>
#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>

using Eigen::aligned_allocator;
using Eigen::Isometry3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

typedef std::vector<Sophus::SE3d, aligned_allocator<Sophus::SE3d>> poseVector;

struct pangolin_config{
    bool draw_pose_axes_ = false;
    std::string trajectory_color_ = "black";

    pangolin_config& draw_pose_axes(const bool b){
        draw_pose_axes_ = b;
        return *this;
    }

    pangolin_config& trajectory_color(const std::string& color){
        trajectory_color_ = color;
        return *this;
    }
};

struct trajectory_view{
    const pangolin_config& pg_config;
    const poseVector& poses;

    trajectory_view(const pangolin_config& pg_config, const poseVector& poses): pg_config(pg_config), poses(poses) {};
};

namespace {
    const std::unordered_map<std::string, std::tuple<float, float, float>> color_map = {
        {"red", {1.0f, 0.0f, 0.0f}},
        {"black", {0.0f, 0.0f, 0.0f}},
        {"blue", {0.0f, 0.0f, 1.0f}}
    };
};

constexpr float view_w = 1920.0f;
constexpr float view_h = 1080.0f;

void pangolin_draw(const std::vector<trajectory_view>& traj_view); 
void pangolin_draw(const trajectory_view& traj_view){
    pangolin_draw({traj_view});
}