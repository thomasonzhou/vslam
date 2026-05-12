#include "viz/vizlib.h"

namespace viz {

void draw_trajectory(const poseVector& poses, const traj_viz_config& pg_config);
void draw_axes(const Sophus::SE3d& pose);
void connect_axes(const Sophus::SE3d& pose1, const Sophus::SE3d& pose2,
                  const traj_viz_config& pg_config);
void draw_line(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2);

void draw_trajectory(const poseVector& poses,
                     const traj_viz_config& pg_config) {
  for (size_t i = 0; i < poses.size(); ++i) {
    if (pg_config.draw_pose_axes) {
      draw_axes(poses[i]);
    }
    if (i > 0) {
      connect_axes(poses[i - 1], poses[i], pg_config);
    }
  }
}

void draw_axes(const Sophus::SE3d& pose) {
  constexpr double axisLength = 0.1;
  const Eigen::Vector3d Ow = pose.translation();
  const Eigen::Vector3d Ox = pose * (axisLength * Eigen::Vector3d(1.0, 0, 0));
  const Eigen::Vector3d Oy = pose * (axisLength * Eigen::Vector3d(0, 1.0, 0));
  const Eigen::Vector3d Oz = pose * (axisLength * Eigen::Vector3d(0, 0, 1.0));
  glBegin(GL_LINES);
  glColor3f(1.0, 0.0, 0.0);
  draw_line(Ow, Ox);
  glColor3f(0.0, 1.0, 0.0);
  draw_line(Ow, Oy);
  glColor3f(0.0, 0.0, 1.0);
  draw_line(Ow, Oz);
  glEnd();
}

void connect_axes(const Sophus::SE3d& pose1, const Sophus::SE3d& pose2,
                  const traj_viz_config& pg_config) {
  const Eigen::Vector3d t1 = pose1.translation();
  const Eigen::Vector3d t2 = pose2.translation();
  glBegin(GL_LINES);
  set_color(pg_config.trajectory_color);

  draw_line(t1, t2);
  glEnd();
}

void draw_line(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2) {
  glVertex3d(v1[0], v1[1], v1[2]);
  glVertex3d(v2[0], v2[1], v2[2]);
}

void draw_point(const Eigen::Vector3d& point) {
  glVertex3d(point[0], point[1], point[2]);
}

void pangolin_run(const std::function<void()>& draw_fn) {
  pangolin::CreateWindowAndBind("Pangolin Visualizer", kViewWidth, kViewHeight);
  glEnable(GL_DEPTH_TEST);  // enable 3D depth buffer for occlusion
  glEnable(GL_BLEND);       // enable translucent objects
  glBlendFunc(
      GL_SRC_ALPHA,
      GL_ONE_MINUS_SRC_ALPHA);  // weighted blend for translucent objects

  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(kViewWidth, 500, kViewHeight, 500, 512, 389,
                                 0.1,
                                 1000),  // intrinsics
      pangolin::ModelViewLookAt(0, 0, 50, 0, 0, 0, 0.0, -1.0,
                                0.0));  // extrinsics

  pangolin::View& d_cam =
      pangolin::CreateDisplay()
          .SetBounds(0.0, 1.0, 0.0, 1.0, -kViewWidth / kViewHeight)
          .SetHandler(new pangolin::Handler3D(s_cam));

  constexpr float white = 1.0F;
  while (!pangolin::ShouldQuit()) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    d_cam.Activate(s_cam);  // use s_cam for display
    glClearColor(white, white, white, white);

    draw_fn();

    pangolin::FinishFrame();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

void pangolin_draw(const std::vector<trajectory_view>& traj_views) {
  pangolin_run([&]() {
    glLineWidth(kLineWidth);
    for (const auto& traj_view : traj_views) {
      draw_trajectory(traj_view.poses, traj_view.pg_config);
    }
  });
}

void pangolin_draw(const std::vector<Eigen::Vector3d>& points) {
  pangolin_run([&]() {
    draw_points(points.size(), [&](size_t i) { return points[i]; });
  });
}

void pangolin_draw(const std::vector<Eigen::Vector3d>& points,
                   const std::vector<Eigen::Vector3d>& points2) {
  pangolin_run([&]() {
    draw_points_compare(
        points.size(), [&](size_t i) { return points[i]; }, points2.size(),
        [&](size_t i) { return points2[i]; });
  });
}

}  // namespace viz
