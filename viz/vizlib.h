#pragma once
#include <pangolin/pangolin.h>

#include <Eigen/Core>
#include <chrono>
#include <functional>
#include <iostream>
#include <sophus/se3.hpp>
#include <string>
#include <unordered_map>
#include <vector>

typedef std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>>
    poseVector;

struct traj_viz_config {
  bool draw_pose_axes = false;
  std::string trajectory_color = "black";
};

struct trajectory_view {
  const traj_viz_config& pg_config;
  const poseVector& poses;

  trajectory_view(const traj_viz_config& pg_config, const poseVector& poses)
      : pg_config(pg_config), poses(poses) {};
};

constexpr float kViewWidth = 1920.0f;
constexpr float kViewHeight = 1080.0f;
namespace viz {

inline const std::unordered_map<std::string, std::tuple<float, float, float>>
    kColorMap = {
                  {"red", {1.0f, 0.0f, 0.0f}},
                  {"dark_red", {0.5f, 0.0f, 0.0f}},
                  {"black", {0.0f, 0.0f, 0.0f}},
                  {"gray", {0.5f, 0.5f, 0.5f}},
                  {"blue", {0.0f, 0.0f, 1.0f}}};

inline void set_color(const std::string& color, const float alpha = 1.0f) noexcept{
  auto it = kColorMap.find(color);
  if (it != kColorMap.end()) {
    const auto& [r, g, b] = it->second;
    glColor4f(r, g, b, alpha);
  } else {
    glColor4f(0.0f, 0.0f, 0.0f, alpha);
  }
}
constexpr int kLineWidth = 2;

void draw_point(const Eigen::Vector3d& point);
void draw_line(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2);

void pangolin_run(const std::function<void()>& draw_fn);

void pangolin_draw(const std::vector<trajectory_view>& traj_view);
inline void pangolin_draw(const trajectory_view& traj_view) {
  pangolin_draw(std::vector<trajectory_view>{traj_view});
}
void pangolin_draw(const std::vector<Eigen::Vector3d>& points);
void pangolin_draw(const std::vector<Eigen::Vector3d>& points, const std::vector<Eigen::Vector3d>& points2);


struct point_config{
  std::string point_color = "black";
  float point_size = 2.0f;
  float point_alpha = 1.0f;
};

struct line_config{
  std::string line_color = "dark_red";
  float line_alpha = 0.05;
  int line_width = 2;
};

struct point_compare_config{
  point_config config1{"black", 2.0f, 0.25};
  point_config config2{"gray", 2.0f};
  line_config correspondence_config{};
  bool draw_correspondence_lines = true;
};

template<typename PointGetter>
void draw_points(const size_t count, PointGetter get_point, const point_config& p_config = point_config{}){
  set_color(p_config.point_color, p_config.point_alpha);
  glPointSize(p_config.point_size);
  glBegin(GL_POINTS);
  for (size_t i = 0; i < count; ++i) {
    draw_point(get_point(i));
  }
  glEnd();
}

template <typename PointGetter1, typename PointGetter2>
void draw_lines_between(const size_t count1, PointGetter1 get_point1, const size_t count2, PointGetter2 get_point2, const line_config& l_config = line_config{}){
  set_color(l_config.line_color, l_config.line_alpha);
  glLineWidth(l_config.line_width);
  glBegin(GL_LINES);
  for (size_t i = 0; i < std::min(count1, count2); ++i) {
    draw_line(get_point1(i), get_point2(i));
  }
  glEnd();
}

template<typename PointGetter1, typename PointGetter2>
void draw_points_compare(const size_t count1, PointGetter1 get_point1, const size_t count2, PointGetter2 get_point2, const point_compare_config& pcomp_config = point_compare_config{}){
  draw_points(count1, get_point1, pcomp_config.config1);
  draw_points(count2, get_point2, pcomp_config.config2);

  if(pcomp_config.draw_correspondence_lines){
    draw_lines_between(count1, get_point1, count2, get_point2, pcomp_config.correspondence_config);
  }
}

template<typename PointGetter1, typename PointGetter2>
void pangolin_draw(const size_t count1, PointGetter1 get_point1, const size_t count2, PointGetter2 get_point2, const point_compare_config& pcomp_config = point_compare_config{}){
  pangolin_run([&]() {
    draw_points_compare(count1, get_point1, count2, get_point2, pcomp_config);
  });
}



}  // namespace viz