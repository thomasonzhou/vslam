#include "frontend/optical_flow.h"
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include "geometry/bilinear.h"
#include <iostream>


namespace frontend {

void OpticalFlowTracker::trackFeatures(const cv::Range &range){

    // for each pixel, find a patch around the area
    // TODO: implement inverse mode
    for (size_t i = range.start; i < range.end; ++i){
        const cv::Point2d kp1 = p1_[i];

        double dx = 0.0;
        double dy = 0.0;

        double cost = 0.0;
        double prev_cost = 0.0;

        Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
        Eigen::Vector2d b = Eigen::Vector2d::Zero();
        Eigen::Vector2d J = Eigen::Vector2d::Zero();

        status_[i] = true; // successful unless something goes wrong

        for (int iter = 0; iter < kMaxIter; ++iter){

            cost = 0.0;

            H = Eigen::Matrix2d::Zero();
            b = Eigen::Vector2d::Zero();

            const cv::Point2d kp2_pred = kp1 + cv::Point2d(dx, dy);
            for(int r = -kHalfPatchSize; r < kHalfPatchSize; ++r){
                for(int c = -kHalfPatchSize; c < kHalfPatchSize; ++c){
                    const double diff_y = dy + r;
                    const double diff_x = dx + c;

                    const double value1 = geometry::bilinear_interpolation(img1_, kp1.x + diff_x, kp1.y + diff_y);
                    const double value2 = geometry::bilinear_interpolation(img2_, kp2_pred.x + diff_x, kp2_pred.y + diff_y);

                    const double error = value2 - value1;

                    // central difference method
                    const double grad_x = 0.5 * (
                        geometry::bilinear_interpolation(img2_, kp2_pred.x + diff_x + 1, kp2_pred.y + diff_y) - 
                        geometry::bilinear_interpolation(img2_, kp2_pred.x + diff_x - 1, kp2_pred.y + diff_y)
                    );
                    const double grad_y = 0.5 * (
                        geometry::bilinear_interpolation(img2_, kp2_pred.x + diff_x, kp2_pred.y + diff_y + 1) - 
                        geometry::bilinear_interpolation(img2_, kp2_pred.x + diff_x, kp2_pred.y + diff_y - 1)
                    );
                    J = Eigen::Vector2d(grad_x, grad_y);

                    H += J * J.transpose();
                    b += -error * J;
                    cost += error * error;
                }
            }

            const Eigen::Vector2d update = H.ldlt().solve(b);

            if (std::isnan(update[0]) || std::isnan(update[1])){
                status_[i] = false;
                std::cout << "Invalid update" << std::endl;
                break;
            }

            if (iter > 0 && cost > prev_cost){
                break;
            }

            dx += update[0];
            dy += update[1];

            constexpr double kConvergenceEpsilon = 1e-6;
            if (update.norm() < kConvergenceEpsilon){
                break;
            }

            prev_cost = cost;
        }
        p2_[i] = kp1 + cv::Point2d(dx, dy);
    }
}

void optical_flow_one_level(
    const cv::Mat &img1,
    const cv::Mat &img2,
    const std::vector<cv::Point2d> &p1,
    std::vector<cv::Point2d> &p2,
    std::vector<bool> &status,
    std::vector<double> &error
){
    p2.resize(p1.size());
    status.resize(p1.size());
    error.resize(p1.size());

    OpticalFlowTracker tracker(img1, img2, p1, p2, status, error);
    cv::parallel_for_(cv::Range(0, static_cast<int>(p1.size())), [&](const cv::Range &range){
        tracker.trackFeatures(range);
    });
}
}  // namespace frontend