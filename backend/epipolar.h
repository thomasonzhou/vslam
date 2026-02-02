#pragma once

#include "backend/calib.h"
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>

void pose_estimation_2d2d(
    const std::vector<cv::KeyPoint>& keypoints1, 
    const std::vector<cv::KeyPoint>& keypoints2,
    const std::vector<cv::DMatch>& matches,
    const PinholeCameraIntrinsics& intrinsics1,
    const PinholeCameraIntrinsics& intrinsics2,
    cv::Mat& c1_R_c2,
    cv::Mat& t_21
);

cv::Point2d pixel_to_camera(const cv::Point2d& pixel, const PinholeCameraIntrinsics& intrinsics){
    return cv::Point2d((pixel.x - intrinsics.cx()) / intrinsics.fx(),
                        (pixel.y - intrinsics.cy()) / intrinsics.fy());
};

cv::Mat homogenous_coordinates(const cv::Point2d& xy){
    return (cv::Mat_<double> (3, 1) << xy.x, xy.y, 1.0);
};

cv::Mat hat(const cv::Mat& t){
    const double x = t.at<double>(0, 0);
    const double y = t.at<double>(1, 0);
    const double z = t.at<double>(2, 0);
    return (cv::Mat_<double>(3, 3) << 0, -z, y, 
                                    z, 0, -x,
                                    -y, x, 0);
};

void triangulation(
    const std::vector<cv::KeyPoint>& keypoints1,
    const std::vector<cv::KeyPoint>& keypoints2,
    const std::vector<cv::DMatch>& matches,
    const PinholeCameraIntrinsics& intrinsics1,
    const PinholeCameraIntrinsics& intrinsics2,
    const cv::Mat& R, 
    const cv::Mat& t, 
    std::vector<cv::Point3d>& points
);