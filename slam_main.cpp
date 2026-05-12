#include <iostream>
#include <fstream>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam3d/types_slam3d.h>

int main(int argc, char** argv) {
  if (argc != 2){
    std::cout << "Second argument is *.g2o file" << std::endl;
    return 1;
  }

  const std::string input_g2o_file = argv[1];
  std::fstream fin(input_g2o_file);
  if (!fin){
    std::cout << input_g2o_file << " does not exist" << std::endl;
    return 1;
  }

  constexpr int kPoseDim = 6;
  constexpr int kOdometryDim = 6;
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<kPoseDim, kOdometryDim>> BlockSolverType;
  typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType> LinearSolverType;
  auto solver = new g2o::OptimizationAlgorithmDogleg(std::make_unique<BlockSolverType>(
    std::make_unique<LinearSolverType>()
  ));

  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(true);

  int vertex_count = 0;
  int edge_count = 0;

  std::string name;
  int vertex_idx = 0;
  int edge_idx0 = 0;
  int edge_idx1 = 1;
  while (!fin.eof()){
    fin >> name;
    if (name == "VERTEX_SE3:QUAT"){
      g2o::VertexSE3 *v = new g2o::VertexSE3();
      fin >> vertex_idx;
      v->setId(vertex_idx);
      v->read(fin);
      optimizer.addVertex(v);
      vertex_count++;
      if(vertex_idx == 0){
        v->setFixed(true);
      }
      optimizer.addVertex(v);
    }
    else if(name == "EDGE_SE3:QUAT"){
      g2o::EdgeSE3 *e = new g2o::EdgeSE3();
      fin >> edge_idx0 >> edge_idx1;
      e->setId(edge_count++);
      e->setVertex(0, optimizer.vertices()[edge_idx0]);
      e->setVertex(1, optimizer.vertices()[edge_idx1]);
      e->read(fin);
      optimizer.addEdge(e);
    }
    if (!fin.good()) break;
  }

  std::cout << "read " << vertex_count << " vertices, " << edge_count << " edges" << std::endl;
  optimizer.initializeOptimization();

  constexpr int kOptimizeSteps = 30;
  optimizer.optimize(kOptimizeSteps);

  const std::string output_file = "result_" + input_g2o_file;
  optimizer.save(output_file.c_str());

  return 0;
}
