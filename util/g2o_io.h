#pragma once

#include <Eigen/Core>
#include <istream>
#include <sophus/se3.hpp>

namespace util::g2o_io {

[[nodiscard]] Sophus::SE3d read_g2o_se3(std::istream& in) {
  Eigen::Vector3d translation;
  for (size_t i = 0; i < 3; ++i) {
    in >> translation[i];
  }
  Eigen::Quaterniond rotation;
  in >> rotation.x();
  in >> rotation.y();
  in >> rotation.z();
  in >> rotation.w();
  rotation.normalize();
  return Sophus::SE3d{rotation, translation};
}

void write_g2o_se3(std::ostream& out, const Sophus::SE3d& estimate) {
  out << estimate.translation().transpose();
  const Eigen::Quaterniond unit_q = estimate.unit_quaternion();
  out << " " << unit_q.x();
  out << " " << unit_q.y();
  out << " " << unit_q.z();
  out << " " << unit_q.w();
}

}  // namespace util::g2o_io
