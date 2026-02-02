#include "backend/solvers.h"

Eigen::Vector2d reprojection_error(
    const Eigen::Vector3d& point3d_cam2,
    const Eigen::Vector2d& point2d_img2,
    const PinholeCameraIntrinsics& intrinsics2
){
    const Eigen::Vector2d projected_pixel2 = camera_to_pixel(point3d_cam2, intrinsics2);
    return point2d_img2 - projected_pixel2;
}

double sum_of_squares_cost(
    const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& points3d_cam1,
    const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>& points2d_img2,
    const PinholeCameraIntrinsics& intrinsics2,
    const Sophus::SE3d& c2_T_c1
){
    double cost = 0.0;
    Eigen::Vector3d point3d_cam2;
    for (size_t i =0; i < points3d_cam1.size(); ++i){
        point3d_cam2 = c2_T_c1 * points3d_cam1[i];
        cost += reprojection_error(point3d_cam2, points2d_img2[i], intrinsics2).squaredNorm();
    }
    return cost;
}


Eigen::Matrix<double, 2, 6> error_jacobian_wrt_perturbation(
    const Eigen::Vector3d& point3d_cam2,
    const PinholeCameraIntrinsics& intrinsics2
){

    const double fx = intrinsics2.fx();
    const double fy = intrinsics2.fy();
    const double cx = intrinsics2.cx();
    const double cy = intrinsics2.cy();

    Eigen::Matrix<double, 2, 6> J;
    const double x = point3d_cam2[0];
    const double y = point3d_cam2[1];
    const double z = point3d_cam2[2];
    const double z2 = z * z;

    J << -fx / z, 
    0, 
    fx * x / z2, 
    fx * x * y / z2, 
    -fx - fx * x * x / z2, 
    fx * y / z,
    0,
    -fy / z,
    fy * y / z2,
    fy + fy * y * y / z2,
    -fy * y * x / z2,
    -fy * x / z;
    return J;
}


Eigen::Vector2d camera_to_pixel(const Eigen::Vector3d& point3d, const PinholeCameraIntrinsics& intrinsics){
    return Eigen::Vector2d(
        intrinsics.fx() * point3d[0] / point3d[2] + intrinsics.cx(),
        intrinsics.fy() * point3d[1] / point3d[2] + intrinsics.cy()
    );
}