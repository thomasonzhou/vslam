#pragma once

#include "geometry/calib.h"
#include "geometry/coordinate_utils.h"
#include <opencv2/calib3d.hpp>
#include <vector>

namespace estimation::epipolar {
    void pose_estimation_2d2d(const std::vector<cv::KeyPoint> &keypoints1,
                            const std::vector<cv::KeyPoint> &keypoints2,
                            const std::vector<cv::DMatch> &matches,
                            const geometry::PinholeCameraIntrinsics &intrinsics1,
                            const geometry::PinholeCameraIntrinsics &intrinsics2,
                            cv::Mat &c1_R_c2, cv::Mat &t_21);

    void triangulation(const std::vector<cv::KeyPoint> &keypoints1,
                    const std::vector<cv::KeyPoint> &keypoints2,
                    const std::vector<cv::DMatch> &matches,
                    const geometry::PinholeCameraIntrinsics &intrinsics1,
                    const geometry::PinholeCameraIntrinsics &intrinsics2,
                    const cv::Mat &R, const cv::Mat &t,
                    std::vector<cv::Point3d> &points);
} // namespace estimation::epipolar