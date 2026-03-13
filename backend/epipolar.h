#pragma once

#include "calib/calib.h"
#include "core/coordinate_utils.h"
#include <opencv2/calib3d.hpp>
#include <vector>

void pose_estimation_2d2d(
    const std::vector<cv::KeyPoint>& keypoints1, 
    const std::vector<cv::KeyPoint>& keypoints2,
    const std::vector<cv::DMatch>& matches,
    const calib::PinholeCameraIntrinsics& intrinsics1,
    const calib::PinholeCameraIntrinsics& intrinsics2,
    cv::Mat& c1_R_c2,
    cv::Mat& t_21
);

void triangulation(
    const std::vector<cv::KeyPoint>& keypoints1,
    const std::vector<cv::KeyPoint>& keypoints2,
    const std::vector<cv::DMatch>& matches,
    const calib::PinholeCameraIntrinsics& intrinsics1,
    const calib::PinholeCameraIntrinsics& intrinsics2,
    const cv::Mat& R, 
    const cv::Mat& t, 
    std::vector<cv::Point3d>& points
);