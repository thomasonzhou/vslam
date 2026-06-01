#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
// #include <g2o/types/slam3d/types_slam3d.h>

#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "estimation/pose_graph/pose_edge_g2o.h"
#include "estimation/pose_graph/pose_vertex_g2o.h"
#include "viz/g2o_viz.h"
#include "viz/vizlib.h"

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cout << "Second argument is *.g2o file" << std::endl;
    return 1;
  }

  const std::string input_g2o_file = argv[1];
  std::fstream fin(input_g2o_file);
  if (!fin) {
    std::cout << input_g2o_file << " does not exist" << std::endl;
    return 1;
  }

  constexpr int kPoseDim = 6;
  constexpr int kOdometryDim = 6;
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<kPoseDim, kOdometryDim>>
      BlockSolverType;
  using LinearSolverType =
      g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>;
  auto solver = new g2o::OptimizationAlgorithmDogleg(
      std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));

  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(true);

  int vertex_count = 0;
  int edge_count = 0;

  std::string name;
  int vertex_idx = 0;
  int edge_idx0 = 0;
  int edge_idx1 = 1;
  while (fin >> name) {
    if (name == "VERTEX_SE3:QUAT") {
      // g2o::VertexSE3* v = new g2o::VertexSE3();
      estimation::pose_graph::VertexPose* v =
          new estimation::pose_graph::VertexPose();
      fin >> vertex_idx;
      v->setId(vertex_idx);
      v->read(fin);
      if (vertex_idx == 0) {
        v->setFixed(true);
      }
      optimizer.addVertex(v);
      vertex_count++;
    } else if (name == "EDGE_SE3:QUAT") {
      // g2o::EdgeSE3* e = new g2o::EdgeSE3();
      estimation::pose_graph::EdgePoseOdom* e =
          new estimation::pose_graph::EdgePoseOdom();
      fin >> edge_idx0 >> edge_idx1;
      e->setId(edge_count++);
      e->setVertex(0, optimizer.vertices()[edge_idx0]);
      e->setVertex(1, optimizer.vertices()[edge_idx1]);
      e->read(fin);
      optimizer.addEdge(e);
    }
  }

  std::vector<Eigen::Vector3d> points_unoptimized;
  points_unoptimized.reserve(vertex_count);
  std::vector<estimation::pose_graph::VertexPose*> pose_vertices;
  pose_vertices.reserve(vertex_count);
  for (int i = 0; i < vertex_count; ++i) {
    // auto v = static_cast<g2o::VertexSE3*>(optimizer.vertex(i));
    auto v =
        static_cast<estimation::pose_graph::VertexPose*>(optimizer.vertex(i));
    points_unoptimized.emplace_back(v->estimate().translation());
    pose_vertices.emplace_back(
        static_cast<estimation::pose_graph::VertexPose*>(optimizer.vertex(i)));
  }
  std::vector<Eigen::Vector3d> points_optimized = points_unoptimized;

  std::mutex points_m;
  viz::PointUpdateAction update_action(
      pose_vertices, points_optimized, points_m,
      [](estimation::pose_graph::VertexPose* point) {
        return point->estimate().translation();
      });
  optimizer.addPostIterationAction(&update_action);

  std::cout << "read " << vertex_count << " vertices, " << edge_count
            << " edges" << std::endl;
  std::thread optimizer_thread([&]() {
    optimizer.initializeOptimization();

    constexpr int kOptimizeSteps = 30;
    optimizer.optimize(kOptimizeSteps);
  });

  try {
    viz::pangolin_draw(
        vertex_count,
        [&points_unoptimized](size_t i) { return points_unoptimized[i]; },
        vertex_count,
        [&optimizer, &points_m, &points_optimized](size_t i) {
          std::lock_guard<std::mutex> optimizer_points_lk(points_m);
          // auto v = static_cast<g2o::VertexSE3*>(optimizer.vertex(i));
          return points_optimized[i];
        });
  } catch (const std::runtime_error& e) {
    std::cerr << "Visualization disabled: " << e.what() << std::endl;
  }

  if (optimizer_thread.joinable()) {
    optimizer_thread.join();
  }
  return 0;
}
