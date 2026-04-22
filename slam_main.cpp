#include <ceres/loss_function.h>
#include <ceres/problem.h>
#include <ceres/solver.h>

#include "estimation/bundle_adjustment/snavely_reprojection.h"
#include "util/bal.h"
#include "viz/vizlib_bal.h"

int main(int argc, char** argv) {
  const std::string problem_path = "../problem-16-22106-pre.txt";
  util::BALProblem bal_problem(problem_path);
  util::BALProblem bal_problem_untouched(problem_path);

  ceres::Problem problem;

  for (size_t i = 0; i < bal_problem.num_observations; ++i) {
    ceres::CostFunction* cost_fn =
        estimation::bundle_adjustment::SnavelyReprojectionError::create(
            bal_problem.observation_data(i)[0],
            bal_problem.observation_data(i)[1]);
    problem.AddResidualBlock(
        cost_fn, new ceres::HuberLoss(1.0),
        bal_problem.mutable_camera_data(bal_problem.camera_idx[i]),
        bal_problem.mutable_points(bal_problem.point_idx[i]));
  }

  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  options.trust_region_strategy_type =  ceres::DOGLEG;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  viz::pangolin_draw(bal_problem, bal_problem_untouched);
  return 0;
}
