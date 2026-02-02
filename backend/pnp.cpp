#include "pnp.h"
#include <iostream>

void bundle_adjustment_gauss_newton(
    const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& points3d_cam1,
    const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>& points2d_img2,
    const PinholeCameraIntrinsics& intrinsics2,
    Sophus::SE3d& c2_T_c1
){
    constexpr int max_iterations = 10;
    constexpr double convergence_epsilon = 1e-6;

    Eigen::Matrix<double, 6, 1> b;
    Eigen::Matrix<double, 6, 6> H;
    Eigen::Matrix<double, 6, 1> dx;
    // fill out J matrix

    const double fx = intrinsics2.fx();
    const double fy = intrinsics2.fy();
    const double cx = intrinsics2.cx();
    const double cy = intrinsics2.cy();

    double lastCost = std::numeric_limits<double>::max();
    for(size_t iter = 0; iter < max_iterations; ++iter){
        b = Eigen::Matrix<double, 6, 1>::Zero();
        H = Eigen::Matrix<double, 6, 6>::Zero();
        double cost = 0.0;
        for(size_t i = 0; i < points3d_cam1.size(); ++i){
            const Eigen::Vector3d point3d_frame2 = c2_T_c1 * points3d_cam1[i];
            const Eigen::Vector2d projected_pixel = camera_to_pixel(point3d_frame2, intrinsics2);
            const Eigen::Vector2d reprojection_error = points2d_img2[i] - projected_pixel;
            cost += reprojection_error.squaredNorm();

            Eigen::Matrix<double, 2, 6> J;
            const double x = point3d_frame2[0];
            const double y = point3d_frame2[1];
            const double z = point3d_frame2[2];
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

            H += J.transpose() * J;
            b += -J.transpose() * reprojection_error;
        }

        // solve for perturbation in the tangent space of the Lie Algebra
        dx = H.ldlt().solve(b);

        std::cout << "iter: " << iter << ", cost: " << std::cout.precision(12) << cost << std::endl;
        if (std::isnan(dx[0]) || std::isnan(dx[1])){
            std::cerr << "nan update, breaking" << std::endl;
            break;
        }
        if (cost > lastCost) {
            std::cerr << "cost increased from update, breaking" << std::endl;
            break;
        }

        lastCost = cost;

        c2_T_c1 = Sophus::SE3d::exp(dx) * c2_T_c1;

        if (dx.norm() <= convergence_epsilon){
            std::cout << "converged" << std::endl;
            break;
        }
    }
}