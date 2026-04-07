#pragma once
#include <Eigen/Core>
#include <opencv2/core.hpp>

namespace geometry {

struct PinholeCameraIntrinsics {
  Eigen::Matrix3d K;

  inline double fx() const noexcept { return K(0, 0); };
  inline double fy() const noexcept { return K(1, 1); };
  inline double cx() const noexcept { return K(0, 2); };
  inline double cy() const noexcept { return K(1, 2); };

  PinholeCameraIntrinsics(const double fx, const double fy, const double cx,
                          const double cy) {
    K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
  }

  void scale(const double scale_factor) noexcept {
    K(0, 0) *= scale_factor;
    K(1, 1) *= scale_factor;
    K(0, 2) *= scale_factor;
    K(1, 2) *= scale_factor;
  }

  cv::Mat camera_matrix_cv() const {
    return (cv::Mat_<double>(3, 3) << fx(), K(0, 1), cx(), K(1, 0), fy(), cy(),
            K(2, 0), K(2, 1), K(2, 2));
  }
};

inline cv::Point2d pixel_to_camera(const cv::Point2d& pixel,
                                   const PinholeCameraIntrinsics& intrinsics) {
  return cv::Point2d((pixel.x - intrinsics.cx()) / intrinsics.fx(),
                     (pixel.y - intrinsics.cy()) / intrinsics.fy());
};

};  // namespace geometry