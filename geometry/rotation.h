#pragma once
#include <Eigen/Core>

namespace geometry {

[[nodiscard]] inline Eigen::Vector<double, 4> angle_axis_to_quat(
    const Eigen::Vector3d& angle_axis) {
  Eigen::Vector4d quat;
  const double theta_2 = angle_axis.squaredNorm();
  if (theta_2 > std::numeric_limits<double>::epsilon()) {
    const double theta = std::sqrt(theta_2);
    const double half_theta = 0.5 * theta;

    quat[0] = std::cos(half_theta);
    quat.tail<3>() = angle_axis * std::sin(half_theta) / theta;
  } else {
    quat[0] = 1.0;
    quat.tail<3>() = 0.5 * angle_axis;
  }
  return quat;
}

}  // namespace geometry