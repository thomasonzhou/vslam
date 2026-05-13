#pragma once
#include <g2o/core/base_binary_edge.h>
#include "estimation/pose_graph/pose_vertex_g2o.h"
#include "geometry/reprojection.h"

namespace estimation::pose_graph {

[[nodiscard]] Eigen::Matrix<double, 6, 6> inv_right_jacobian_approx_se3(
    const Sophus::SE3d& error) {
  Eigen::Matrix<double, 6, 6> adjoint;
  const Eigen::Matrix3d phi_hat = Sophus::SO3d::hat(error.so3().log());
  adjoint.block(0, 0, 3, 3) = phi_hat;
  adjoint.block(0, 3, 3, 3) = Sophus::SO3d::hat(error.translation());
  adjoint.block(3, 0, 3, 3) = Eigen::Matrix3d::Zero();
  adjoint.block(3, 3, 3, 3) = phi_hat;

  return Eigen::Matrix<double, 6, 6>::Identity() + 0.5 * adjoint;
}

class EdgePoseOdom
    : public g2o::BaseBinaryEdge<6, Sophus::SE3d, VertexPose, VertexPose> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void computeError() override {
    const Sophus::SE3d v1 =
        (static_cast<const VertexPose*>(_vertices[0]))->estimate();
    const Sophus::SE3d v2 =
        (static_cast<const VertexPose*>(_vertices[1]))->estimate();

    _error = (_measurement.inverse() * v1.inverse() * v2).log();
  }

  void linearizeOplus() override {
    const Sophus::SE3d v1 =
        (static_cast<const VertexPose*>(_vertices[0]))->estimate();
    const Sophus::SE3d v2 =
        (static_cast<const VertexPose*>(_vertices[1]))->estimate();

    Eigen::Matrix<double, 6, 6> J_r_inv =
        inv_right_jacobian_approx_se3(Sophus::SE3d::exp(_error));

    _jacobianOplusXi = -J_r_inv * v2.inverse().Adj();
    _jacobianOplusXj = J_r_inv * v2.inverse().Adj();
  }

  bool read(std::istream& in) override {
    setMeasurement(util::g2o_io::read_g2o_se3(in));

    for (long int r = 0; r < information().rows() && in.good(); ++r) {
      for (long int c = r; c < information().cols() && in.good(); ++c) {
        in >> information()(r, c);
        if (r != c) {
          information()(c, r) = information()(r, c);
        }
      }
    }
    return true;
  }

  bool write(std::ostream& out) const override {
    const VertexPose* v1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* v2 = static_cast<const VertexPose*>(_vertices[1]);

    out << v1->id() << " " << v2->id() << " ";
    util::g2o_io::write_g2o_se3(out, _measurement);
    for (long int r = 0; r < information().rows(); ++r) {
      for (long int c = r; c < information().cols(); ++c) {
        out << " " << information()(r, c);
      }
    }
    out << std::endl;
    return true;
  }
};

}  // namespace estimation::pose_graph
