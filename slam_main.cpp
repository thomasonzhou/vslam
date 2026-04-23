#include "estimation/bundle_adjustment/g2o_ba.h"
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/sparse_optimizer.h>

#include <ceres/rotation.h>

#include <sophus/se3.hpp>
#include "util/bal.h"
#include "viz/vizlib.h"

using BlockSolverType = g2o::BlockSolverX;
using LinearSolverType = g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>;

using estimation::bundle_adjustment::VertexIntrinsics;
using estimation::bundle_adjustment::VertexPose;
using estimation::bundle_adjustment::VertexPoint;
using estimation::bundle_adjustment::EdgeProjection;
using estimation::bundle_adjustment::kVertexIntrinsicsIdx;
using estimation::bundle_adjustment::kVertexPoseIdx;
using estimation::bundle_adjustment::kVertexPointIdx;

int main(int argc, char** argv) {
  const std::string problem_path = "../problem-16-22106-pre.txt";
  util::BALProblem bal_problem(problem_path);

  // typedef g2o::BlockSolver<g2o::BlockSolverTraits<util::bal::kPoseDim, util::bal::kTranslationDim>> BlockSolverType;

  auto solver = new g2o::OptimizationAlgorithmDogleg(
    std::make_unique<BlockSolverType>(
      std::make_unique<LinearSolverType>()
    )
  );

  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(true);

  std::vector<VertexIntrinsics*> intrinsics;
  std::vector<VertexPose*> poses;
  intrinsics.reserve(bal_problem.num_cameras);
  poses.reserve(bal_problem.num_cameras);
  std::vector<VertexPoint*> points;
  points.reserve(bal_problem.num_points);

  int id = 0;
  for (size_t cam = 0; cam < bal_problem.num_cameras; ++cam){
    VertexIntrinsics* v_k = new VertexIntrinsics();
    VertexPose* v_pose = new VertexPose();

    const auto cam_data = bal_problem.camera_data(cam);
    v_pose->setId(id++);

    Eigen::Quaterniond q;
    ceres::AngleAxisToQuaternion<ceres::EigenQuaternionOrder>(cam_data, q.coeffs().data());
    Sophus::SE3d pose;
    pose.setQuaternion(q);
    pose.translation() = Eigen::Map<const Eigen::Vector3d>(cam_data + util::bal::kRotationDim);
    v_pose->setEstimate(pose);

    poses.emplace_back(v_pose);
    if (cam == 0){
      poses[0]->setFixed(true);
    }
    optimizer.addVertex(v_pose);
    
    v_k->setId(id++);
    v_k->setEstimate(Eigen::Map<const Eigen::Vector3d>(cam_data+util::bal::kPoseDim));
    optimizer.addVertex(v_k);
    intrinsics.emplace_back(v_k);
  }

  for (size_t point = 0; point < bal_problem.num_points; ++point){
    VertexPoint* v_point = new VertexPoint();
    v_point->setId(id++);
    v_point->setEstimate(Eigen::Map<const Eigen::Vector3d>(bal_problem.points(point)));

    v_point->setMarginalized(true);
    optimizer.addVertex(v_point);
    points.emplace_back(v_point);
  }

  for (size_t obs = 0; obs < bal_problem.num_observations; ++obs){
    const size_t cam_idx = bal_problem.camera_idx[obs];
    const size_t point_idx = bal_problem.point_idx[obs];
    EdgeProjection* edge = new EdgeProjection();
    edge->setVertex(kVertexIntrinsicsIdx, intrinsics[cam_idx]); // can change to share intrinsics across views
    edge->setVertex(kVertexPoseIdx, poses[cam_idx]);
    edge->setVertex(kVertexPointIdx, points[point_idx]);

    edge->setMeasurement(Eigen::Map<const Eigen::Vector2d>(bal_problem.observation_data(obs)));
    edge->setInformation(Eigen::Matrix2d::Identity());
    optimizer.addEdge(edge);
  }


  std::vector<Eigen::Vector3d> points_viz;
  points_viz.reserve(points.size());
  for (size_t point = 0; point < points.size(); ++point){
    points_viz.emplace_back(points[point]->estimate());
  }

  optimizer.initializeOptimization();
  constexpr int kIters = 10;
  optimizer.optimize(kIters);

  std::vector<Eigen::Vector3d> points_viz2;
  points_viz2.reserve(points.size());
  for (size_t point = 0; point < points.size(); ++point){
    points_viz2.emplace_back(points[point]->estimate());
  }

  viz::pangolin_draw(points_viz, points_viz2);

  return 0;
}
