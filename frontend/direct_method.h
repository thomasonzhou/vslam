#pragma once
#include <opencv2/core.hpp>
#include <vector>
#include <sophus/se3.hpp>
#include "geometry/calib.h"

namespace frontend {

void project_points(const cv::Mat &depth1, 
    const std::vector<cv::Point2d> &p1, 
    std::vector<cv::Point2d> &p2, 
    std::vector<uchar> &status,
    const geometry::PinholeCameraIntrinsics &intrinsics1,
    const geometry::PinholeCameraIntrinsics &intrinsics2,
    const cv::Size &img2_size,
    const Sophus::SE3d &T_12);



void direct_method_single_level(
    const cv::Mat &img1, 
    const cv::Mat &img2,
    const cv::Mat &depth_img1,
    const geometry::PinholeCameraIntrinsics &intrinsics1,
    const geometry::PinholeCameraIntrinsics &intrinsics2,
    const std::vector<cv::Point2d> &p1,
    std::vector<uchar> &status,
    Sophus::SE3d &T_12
);

}  // namespace frontend