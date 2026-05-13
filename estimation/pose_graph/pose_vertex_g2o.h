#pragma once

#include <g2o/core/base_vertex.h>
#include <sophus/se3.hpp>
#include "util/g2o_io.h"

namespace estimation::pose_graph {

class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  void setToOriginImpl() override { _estimate = Sophus::SE3d(); }

  void oplusImpl(const double* update) override {
    Eigen::Map<const Eigen::Vector<double, 6>> update_eigen(update);
    _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
  }

  bool read(std::istream& in) override {
    setEstimate(util::g2o_io::read_g2o_se3(in));
    return true;
  }

  bool write(std::ostream& out) const override {
    out << id() << " ";
    util::g2o_io::write_g2o_se3(out, _estimate);
    out << std::endl;
    return true;
  }
};
}  // namespace estimation::pose_graph
