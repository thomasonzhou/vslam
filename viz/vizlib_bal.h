#pragma once
#include "util/bal.h"
#include "viz/vizlib.h"

namespace viz {

void pangolin_draw(const util::BALProblem& bal_problem) {
  pangolin_run([&]() {
    glPointSize(2.0f);
    glColor3f(0.0f, 0.0f, 0.0f);
    glBegin(GL_POINTS);
    for (size_t i = 0; i < bal_problem.num_points; ++i) {
      Eigen::Map<const Eigen::Vector3d> p(bal_problem.points(i));
      draw_point(p);
    }
    glEnd();
  });
}

void pangolin_draw(const util::BALProblem& bal_problem,
                   const util::BALProblem& bal_problem2) {
  pangolin_run([&]() {
    glPointSize(2.0f);
    glColor3f(0.0f, 0.0f, 0.0f);
    glBegin(GL_POINTS);
    for (size_t i = 0; i < bal_problem.num_points; ++i) {
      Eigen::Map<const Eigen::Vector3d> p(bal_problem.points(i));
      draw_point(p);
    }
    glColor3f(0.5f, 0.5f, 0.5f);
    for (size_t i = 0; i < bal_problem2.num_points; ++i) {
      Eigen::Map<const Eigen::Vector3d> p(bal_problem2.points(i));
      draw_point(p);
    }
    glEnd();

    glBegin(GL_LINES);
    glLineWidth(kLineWidth);
    glColor3f(0.5, 0.0, 0.0);
    for (size_t i = 0;
         i < std::min(bal_problem.num_points, bal_problem2.num_points); ++i) {
      Eigen::Map<const Eigen::Vector3d> p1(bal_problem.points(i));
      Eigen::Map<const Eigen::Vector3d> p2(bal_problem2.points(i));
      draw_line(p1, p2);
    }
    glEnd();
  });
}

}  // namespace viz