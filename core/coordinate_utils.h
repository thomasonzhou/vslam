#pragma once
#include <opencv2/core.hpp>

namespace core {

cv::Point2d pixel_to_camera(const cv::Point2d &pixel,
                            const calib::PinholeCameraIntrinsics &intrinsics) {
  return cv::Point2d((pixel.x - intrinsics.cx()) / intrinsics.fx(),
                     (pixel.y - intrinsics.cy()) / intrinsics.fy());
};

cv::Mat homogenous_coordinates(const cv::Point2d &xy) {
  return (cv::Mat_<double>(3, 1) << xy.x, xy.y, 1.0);
};

cv::Mat hat(const cv::Mat &t) {
  const double x = t.at<double>(0, 0);
  const double y = t.at<double>(1, 0);
  const double z = t.at<double>(2, 0);
  return (cv::Mat_<double>(3, 3) << 0, -z, y, z, 0, -x, -y, x, 0);
};

}; // namespace core