#pragma once
#include "geometry/calib.h"
#include <sophus/se3.hpp>

namespace geometry {

[[nodiscard]] inline cv::Mat depth_from_disparity(const cv::Mat &disparity_img,
    const double baseline_m,
    const geometry::PinholeCameraIntrinsics &intrinsics){
    return intrinsics.fx() * baseline_m / disparity_img;
}

struct PointPixel{
    Eigen::Vector3d point3d;
    double x;
    double y;
};

[[nodiscard]] bool cam2_from_cam1(const double depth_p1, const cv::Point2d &p1, const Eigen::Matrix3d &K1_inv, 
    const geometry::PinholeCameraIntrinsics &intrinsics2,
    const cv::Size &img2_size,
    PointPixel &point2_data, const Sophus::SE3d &T_12);

[[nodiscard]] Eigen::Matrix<double, 2, 6> jacobian_pixel_error_wrt_perturbation(
    const Eigen::Vector3d &point3d_cam2,
    const geometry::PinholeCameraIntrinsics &intrinsics2);

[[nodiscard]] Eigen::Vector2d reprojection_error(const Eigen::Vector3d &point3d_cam2,
                   const Eigen::Vector2d &point2d_img2,
                   const geometry::PinholeCameraIntrinsics &intrinsics2);

[[nodiscard]] double sum_of_squares_cost(
    const std::vector<Eigen::Vector3d,
                      Eigen::aligned_allocator<Eigen::Vector3d>> &points3d_cam1,
    const std::vector<Eigen::Vector2d,
                      Eigen::aligned_allocator<Eigen::Vector2d>> &points2d_img2,
    const geometry::PinholeCameraIntrinsics &intrinsics2,
    const Sophus::SE3d &c2_T_c1);


[[nodiscard]] Eigen::Vector2d camera_to_pixel(const Eigen::Vector3d &point3d,
                const geometry::PinholeCameraIntrinsics &intrinsics);
}; // namespace geometry