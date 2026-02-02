#include "pnp.h"
#include <iostream>

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
    Sophus::SE3d& c2_T_c1
){
    double cost = 0.0;
    Eigen::Vector3d point3d_cam2;
    for (size_t i =0; i < points3d_cam1.size(); ++i){
        point3d_cam2 = c2_T_c1 * points3d_cam1[i];
        cost += reprojection_error(point3d_cam2, points2d_img2[i], intrinsics2).squaredNorm();
    }
    return cost;
}

void bundle_adjustment(
    const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& points3d_cam1,
    const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>& points2d_img2,
    const PinholeCameraIntrinsics& intrinsics2,
    Sophus::SE3d& c2_T_c1
){
    constexpr int max_iterations = 10;
    // pertubation norm threshold
    constexpr double convergence_epsilon = 1e-6;

    // levenberg marquardt
    constexpr double lm_lambda_multiplier = 10.0;
    constexpr int max_lm_iterations = 10;

    Eigen::Matrix<double, 6, 1> b;
    Eigen::Matrix<double, 6, 6> H;
    Eigen::Matrix<double, 6, 1> dx;
    // fill out J matrix
    double last_cost = std::numeric_limits<double>::max();

    const double fx = intrinsics2.fx();
    const double fy = intrinsics2.fy();
    const double cx = intrinsics2.cx();
    const double cy = intrinsics2.cy();

    double lambda = 0.001;
    for(size_t iter = 0; iter < max_iterations; ++iter){
        b = Eigen::Matrix<double, 6, 1>::Zero();
        H = Eigen::Matrix<double, 6, 6>::Zero();
        
        for(size_t i = 0; i < points3d_cam1.size(); ++i){
            const Eigen::Vector3d point3d_cam2 = c2_T_c1 * points3d_cam1[i];
            const Eigen::Vector2d error = reprojection_error(point3d_cam2, points2d_img2[i], intrinsics2);

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

            H += J.transpose() * J;
            b += -J.transpose() * error;
        }

        // solve for perturbation in the tangent space of the Lie Algebra
        dx = (H + lambda * Eigen::Matrix<double, 6, 6>::Identity()).ldlt().solve(b);

        if (std::isnan(dx[0]) || std::isnan(dx[1])){
            std::cerr << "nan update, breaking" << std::endl;
            break;
        }
        
        // check to see if update will increase or reduce cost
        Sophus::SE3d pose_candidate = Sophus::SE3d::exp(dx) * c2_T_c1;
        double cost = sum_of_squares_cost(
            points3d_cam1,
            points2d_img2,
            intrinsics2, 
            pose_candidate
        );
        std::cout << "iter: " << iter << ", cost: " << std::cout.precision(12) << cost << ", last_cost: " << last_cost << std::endl;

        if (dx.norm() <= convergence_epsilon){
            std::cout << "converged" << std::endl;
            break;
        }
        if (cost > last_cost) {
            std::cout << "cost increased from from " << last_cost << " to " << cost << ", updating lambda" << std::endl;

            for (int lm_iter = 0; lm_iter < max_lm_iterations; ++lm_iter){
                lambda *= lm_lambda_multiplier;
                Eigen::Matrix<double, 6, 6> H_lambda_eye = H + lambda * Eigen::Matrix<double, 6, 6>::Identity();

                dx = H_lambda_eye.ldlt().solve(b);
                pose_candidate = Sophus::SE3d::exp(dx) * c2_T_c1;
                cost = sum_of_squares_cost(
                    points3d_cam1,
                    points2d_img2,
                    intrinsics2, 
                    pose_candidate
                );
                std::cout << "lambda: " << lambda << ", cost: " << cost << ", last cost: " << last_cost << std::endl;

                if (cost < last_cost){
                    break;
                } 
            }

            if (cost >= last_cost){
                std::cerr << "Levenberg-Marquardt didn't converge, breaking" << std::endl;
                break;
            }

        } else{
            lambda /= lm_lambda_multiplier;
        }
        c2_T_c1 = Sophus::SE3d::exp(dx) * c2_T_c1;
        last_cost = cost;

    }
}