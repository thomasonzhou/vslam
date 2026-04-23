#pragma once
#include "util/bal.h"
#include "viz/vizlib.h"

namespace viz {

void pangolin_draw(const util::BALProblem& bal_problem) {
  pangolin_run([&]() {
    draw_points(bal_problem.num_points, [&](size_t i){
      return Eigen::Map<const Eigen::Vector3d> (bal_problem.points(i));
    });
  });
}

void pangolin_draw(const util::BALProblem& bal_problem,
                   const util::BALProblem& bal_problem2) {
  pangolin_run([&]() {
    draw_points_compare(bal_problem.num_points, [&](size_t i){
      return Eigen::Map<const Eigen::Vector3d> (bal_problem.points(i));
    }, bal_problem2.num_points, [&](size_t i){
      return Eigen::Map<const Eigen::Vector3d> (bal_problem2.points(i));
    })
  });
}

}  // namespace viz