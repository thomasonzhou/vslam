#include "frontend/direct_method.h"
#include "geometry/bilinear.h"
#include "geometry/reprojection.h"
#include <mutex>

namespace frontend {

constexpr int kPoseDim = 6;
constexpr int kPixelDim = 2;
constexpr int kErrorDim = 1; // intensity

class DirectMethodTracker{
public:
    DirectMethodTracker(
        const cv::Mat &img1, 
        const cv::Mat &img2,
        const cv::Mat &depth1,
        const geometry::PinholeCameraIntrinsics &intrinsics1,
        const geometry::PinholeCameraIntrinsics &intrinsics2,
        const std::vector<cv::Point2d> &p1,
        std::vector<uchar> &status,
        const Sophus::SE3d &T_12
    ): img1_(img1), img2_(img2), depth1_(depth1), 
    intrinsics1_(intrinsics1), 
    intrinsics2_(intrinsics2), p1_(p1), status_(status), T_12_(T_12) {
        reset_optimization();
    }

    void track(const cv::Range& range);
    void reset_optimization(){
        std::lock_guard<std::mutex> lk(hessian_bias_cost_mutex_);
        H_ = Eigen::Matrix<double, kPoseDim, kPoseDim>::Zero();
        b_ = Eigen::Vector<double, kPoseDim>::Zero();
        cost_ = 0.0;
    }

    Eigen::Vector<double, kPoseDim> perturbation() const {
        std::lock_guard<std::mutex> lk(hessian_bias_cost_mutex_);
        return H_.ldlt().solve(b_);
    }

    double cost() const {
        std::lock_guard<std::mutex> lk(hessian_bias_cost_mutex_);
        return cost_;
    } // called after all processing is completed
private:
    void update(Eigen::Matrix<double, kPoseDim, kPoseDim> &H, Eigen::Vector<double, kPoseDim> &b, const double cost){
        std::lock_guard<std::mutex> lk(hessian_bias_cost_mutex_);
        H_ += H;
        b_ += b;
        cost_ += cost;
    }

    const cv::Mat &img1_; 
    const cv::Mat &img2_;
    const cv::Mat &depth1_;
    const geometry::PinholeCameraIntrinsics &intrinsics1_;
    const geometry::PinholeCameraIntrinsics &intrinsics2_;
    const std::vector<cv::Point2d> &p1_;
    std::vector<uchar> &status_;
    const Sophus::SE3d &T_12_;

    Eigen::Matrix<double, kPoseDim, kPoseDim> H_;
    Eigen::Vector<double, kPoseDim> b_;
    double cost_;

    mutable std::mutex hessian_bias_cost_mutex_;
};

void DirectMethodTracker::track(const cv::Range& range){
    const Eigen::Matrix3d K1_inv = intrinsics1_.K.inverse();
    Eigen::Matrix<double, kPoseDim, kPoseDim> H_local = Eigen::Matrix<double, kPoseDim, kPoseDim>::Zero();
    Eigen::Vector<double, kPoseDim> b_local = Eigen::Vector<double, kPoseDim>::Zero();
    double cost_local = 0.0;

    for (size_t i = range.start; i < range.end; ++i){
        if (!status_[i]) continue;
        constexpr int kHalfPatch=2;

        const cv::Point2d &kp1 = p1_[i];
        for (int r = -kHalfPatch; r <= kHalfPatch; ++r){
            if(!status_[i]) break;
            for (int c = -kHalfPatch; c <= kHalfPatch; ++c){

                
                // get 3d point in camera1 frame
                // const Eigen::Vector3d &point1_3d = points3d1_[i];
                const double x1 = kp1.x + c;
                const double y1 = kp1.y + r;

                const double depth_p1 = geometry::bilinear_interpolation(depth1_, x1, y1);
                if (depth_p1 <= 0){
                    status_[i] = false;
                    break;
                }
                // get pixel intensity using image
                const double intensity1 = geometry::bilinear_interpolation(img1_, x1, y1);
                
                const Eigen::Vector3d pixel1_homogenous = Eigen::Vector3d(x1, y1, 1.0);
                const Eigen::Vector3d point1_3d = depth_p1 * K1_inv * pixel1_homogenous;
                
                // transform 3dpoint to cam2 frame, intrinsics to get pixel location, pixel intensity
                const Eigen::Vector3d point2_3d = T_12_ * point1_3d;
                const Eigen::Vector3d pixel2_homogenous = intrinsics2_.K * point2_3d / point2_3d[2];
        
                // out of bounds is not enough grounds for disqualification
                constexpr double kBorder = 5;
                const double x2 = pixel2_homogenous[0];
                if (x2 < kBorder || x2 > img2_.cols - kBorder - 1){
                    continue;
                }
                const double y2 = pixel2_homogenous[1];
                if (y2 < kBorder || y2 > img2_.rows - kBorder - 1){
                    continue;
                }
        
                const double intensity2 = geometry::bilinear_interpolation(img2_, x2, y2);
                const double photometric_error = intensity2 - intensity1;

                const Eigen::Matrix<double, kPixelDim, kPoseDim> J_pixel_perturb = geometry::jacobian_pixel_error_wrt_perturbation(point2_3d, intrinsics2_);
                
                const Eigen::Vector<double, kPixelDim> J_intensity_pixel(
                    0.5 * (geometry::bilinear_interpolation(img2_, x2 + 1, y2) - 
                    geometry::bilinear_interpolation(img2_, x2 - 1, y2)),
                    0.5 * (geometry::bilinear_interpolation(img2_, x2, y2 + 1) - 
                    geometry::bilinear_interpolation(img2_, x2, y2 - 1))
                );

                const Eigen::Matrix<double, kErrorDim, kPoseDim> J = J_intensity_pixel.transpose() * J_pixel_perturb;
                H_local += J.transpose() * J;
                b_local += -J.transpose() * photometric_error;
                cost_local += 0.5 * photometric_error * photometric_error;
            }
        }
    }
    update(H_local, b_local, cost_local);
}

void direct_method_single_level(
    const cv::Mat &img1, 
    const cv::Mat &img2,
    const cv::Mat &disparity_img1,
    const double baseline_m,
    const geometry::PinholeCameraIntrinsics &intrinsics1,
    const geometry::PinholeCameraIntrinsics &intrinsics2,
    const std::vector<cv::Point2d> &p1,
    std::vector<uchar> &status,
    Sophus::SE3d &T_12
){
    if (status.size() != p1.size()){
        status.resize(p1.size(), true);
    }

    const cv::Mat depth1 = intrinsics1.fx() * baseline_m / disparity_img1;

    double cost = 0.0;
    double last_cost = std::numeric_limits<double>::max();
    constexpr double kConverged = 0.001;

    constexpr int kIters = 10;
    for (int iter = 0; iter < kIters; ++iter){
        DirectMethodTracker tracker(img1, img2, depth1, intrinsics1, intrinsics2, p1, status, T_12);
        cv::parallel_for_(cv::Range(0, static_cast<int>(p1.size())), [&](const cv::Range &range){
            tracker.track(range);
        });

        cost = tracker.cost();
        if (iter > 0 && cost > last_cost){
            tracker.reset_optimization();
            break;
        }

        const Eigen::Vector<double, kPoseDim> dx = tracker.perturbation();
        tracker.reset_optimization();
        if (dx.norm() < kConverged){
            break;
        }
        T_12 = Sophus::SE3d::exp(dx) * T_12;
        last_cost = cost;
    }
}

}  // namespace frontend