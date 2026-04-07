#include "estimation/icp/icp_nonlinear.h"

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

#include <algorithm>
#include <sophus/se3.hpp>

namespace {

constexpr int kPoseDim = 6;
class VertexPose : public g2o::BaseVertex<kPoseDim, Sophus::SE3d> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  void setToOriginImpl() override { _estimate = Sophus::SE3d(); }

  void oplusImpl(const double* update) override {
    Eigen::Map<const Eigen::Vector<double, kPoseDim>> dx(update);
    _estimate = Sophus::SE3d::exp(dx) * _estimate;
  }

  bool read(std::istream& in) override { return true; }
  bool write(std::ostream& out) const override { return true; }
};

constexpr int kTranslationDim = 3;
class EdgePointTranslation
    : public g2o::BaseUnaryEdge<kTranslationDim, Eigen::Vector3d, VertexPose> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  EdgePointTranslation(const Eigen::Vector3d& translation)
      : translation_(translation) {};

  void computeError() override {
    const VertexPose* pose = static_cast<const VertexPose*>(_vertices[0]);
    const Sophus::SE3d& T = pose->estimate();
    const Eigen::Vector3d& xyz = T * translation_;
    _error = _measurement - xyz;
  }

  void linearizeOplus() override {
    const VertexPose* pose = static_cast<const VertexPose*>(_vertices[0]);
    const Sophus::SE3d& T = pose->estimate();
    const Eigen::Vector3d& xyz = T * translation_;

    _jacobianOplusXi.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity();
    _jacobianOplusXi.block<3, 3>(0, 3) = Sophus::SO3d::hat(xyz);
  }

  bool read(std::istream& in) override { return true; }
  bool write(std::ostream& out) const override { return true; }

 private:
  Eigen::Vector3d translation_;
};

typedef g2o::BlockSolver<g2o::BlockSolverTraits<kPoseDim, kTranslationDim>>
    BlockSolverType;
typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>
    LinearSolverType;

}  // namespace

namespace estimation::icp {
void NonlinearICPSolver::solve(
    const std::vector<Eigen::Vector3d,
                      Eigen::aligned_allocator<Eigen::Vector3d>>& points3d_cam1,
    const std::vector<Eigen::Vector3d,
                      Eigen::aligned_allocator<Eigen::Vector3d>>& points3d_cam2,
    Sophus::SE3d& c2_T_c1) const {
  auto solver = new g2o::OptimizationAlgorithmDogleg(
      std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));
  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(true);

  VertexPose* v_pose = new VertexPose();

  int id = 0;
  v_pose->setId(id++);
  v_pose->setEstimate(Sophus::SE3d());

  optimizer.addVertex(v_pose);

  for (size_t i = 0; i < std::min(points3d_cam1.size(), points3d_cam2.size());
       ++i) {
    EdgePointTranslation* edge_xyz = new EdgePointTranslation(points3d_cam1[i]);
    edge_xyz->setId(id++);
    edge_xyz->setVertex(0, v_pose);
    edge_xyz->setMeasurement(points3d_cam2[i]);
    edge_xyz->setInformation(Eigen::Matrix3d::Identity());
    optimizer.addEdge(edge_xyz);
  }

  optimizer.initializeOptimization();
  constexpr int kIters = 10;
  optimizer.optimize(kIters);

  c2_T_c1 = v_pose->estimate();
};

}  // namespace estimation::icp