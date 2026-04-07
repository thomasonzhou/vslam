#include "frontend/direct_method.h"

#include <mutex>
#include <opencv2/imgproc.hpp>

#include "geometry/bilinear.h"
#include "geometry/reprojection.h"

namespace {

constexpr int kPoseDim = 6;
constexpr int kPixelDim = 2;
constexpr int kErrorDim = 1;  // intensity
constexpr int kHalfPatch = 1;

class DirectMethodTracker {
 public:
  DirectMethodTracker(const cv::Mat& img1, const cv::Mat& img2,
                      const std::vector<double>& depths1,
                      const geometry::PinholeCameraIntrinsics& intrinsics1,
                      const geometry::PinholeCameraIntrinsics& intrinsics2,
                      const std::vector<cv::Point2d>& p1,
                      std::vector<uchar>& status, const Sophus::SE3d& T_12)
      : img1_(img1),
        img2_(img2),
        depths1_(depths1),
        intrinsics1_(intrinsics1),
        intrinsics2_(intrinsics2),
        p1_(p1),
        status_(status),
        T_12_(T_12) {
    reset_optimization();
    reset_cost();
  }

  void track(const cv::Range& range);
  void reset_optimization() {
    std::lock_guard<std::mutex> lk_hb(hessian_bias_mutex_);
    H_ = Eigen::Matrix<double, kPoseDim, kPoseDim>::Zero();
    b_ = Eigen::Vector<double, kPoseDim>::Zero();
  }

  void reset_cost() {
    std::lock_guard<std::mutex> lk_c(cost_mutex_);
    cost_ = 0.0;
  }

  Eigen::Vector<double, kPoseDim> perturbation() const {
    std::lock_guard<std::mutex> lk(hessian_bias_mutex_);
    return H_.ldlt().solve(b_);
  }

  double cost() const {
    std::lock_guard<std::mutex> lk(cost_mutex_);
    return cost_;
  }  // called after all processing is completed

  void eval(const cv::Range& range, const Sophus::SE3d& candidate_T_12);

 private:
  void update(Eigen::Matrix<double, kPoseDim, kPoseDim>& H,
              Eigen::Vector<double, kPoseDim>& b) {
    std::lock_guard<std::mutex> lk(hessian_bias_mutex_);
    H_ += H;
    b_ += b;
  }
  void update(const double cost) {
    std::lock_guard<std::mutex> lk(cost_mutex_);
    cost_ += cost;
  }

  bool cam2_from_cam1(const size_t i, const cv::Point2d& p1,
                      const Eigen::Matrix3d& K1_inv,
                      geometry::PointPixel& point2_data,
                      const Sophus::SE3d& T_12) {
    return geometry::cam2_from_cam1(depths1_[i], p1, K1_inv, intrinsics2_,
                                    img2_.size(), point2_data, T_12,
                                    kHalfPatch);
  };

  const cv::Mat& img1_;
  const cv::Mat& img2_;
  const std::vector<double>& depths1_;
  const geometry::PinholeCameraIntrinsics& intrinsics1_;
  const geometry::PinholeCameraIntrinsics& intrinsics2_;
  const std::vector<cv::Point2d>& p1_;
  std::vector<uchar>& status_;
  const Sophus::SE3d& T_12_;

  Eigen::Matrix<double, kPoseDim, kPoseDim> H_;
  Eigen::Vector<double, kPoseDim> b_;
  double cost_;

  mutable std::mutex hessian_bias_mutex_;
  mutable std::mutex cost_mutex_;
};

void DirectMethodTracker::track(const cv::Range& range) {
  const Eigen::Matrix3d K1_inv = intrinsics1_.K.inverse();
  Eigen::Matrix<double, kPoseDim, kPoseDim> H_local =
      Eigen::Matrix<double, kPoseDim, kPoseDim>::Zero();
  Eigen::Vector<double, kPoseDim> b_local =
      Eigen::Vector<double, kPoseDim>::Zero();

  for (size_t i = range.start; i < range.end; ++i) {
    if (!status_[i]) continue;

    const cv::Point2d& kp1 = p1_[i];

    geometry::PointPixel center_cam2;
    if (!cam2_from_cam1(i, kp1, K1_inv, center_cam2, T_12_)) {
      status_[i] = false;
      continue;
    }

    const Eigen::Matrix<double, kPixelDim, kPoseDim> J_pixel_perturb =
        geometry::jacobian_pixel_error_wrt_perturbation(center_cam2.point3d,
                                                        intrinsics2_);

    for (int r = -kHalfPatch; r <= kHalfPatch; ++r) {
      for (int c = -kHalfPatch; c <= kHalfPatch; ++c) {
        const cv::Point2d p1 = kp1 + cv::Point2d(c, r);
        const double x2 = center_cam2.x + c;
        const double y2 = center_cam2.y + r;
        // get pixel intensity using image
        const double intensity1 =
            geometry::bilinear_interpolation(img1_, p1.x, p1.y);
        const double intensity2 =
            geometry::bilinear_interpolation(img2_, x2, y2);
        const double photometric_error = intensity2 - intensity1;

        const Eigen::Vector<double, kPixelDim> J_intensity_pixel(
            0.5 * (geometry::bilinear_interpolation(img2_, x2 + 1, y2) -
                   geometry::bilinear_interpolation(img2_, x2 - 1, y2)),
            0.5 * (geometry::bilinear_interpolation(img2_, x2, y2 + 1) -
                   geometry::bilinear_interpolation(img2_, x2, y2 - 1)));

        const Eigen::Matrix<double, kErrorDim, kPoseDim> J =
            J_intensity_pixel.transpose() * J_pixel_perturb;
        H_local += J.transpose() * J;
        b_local += -J.transpose() * photometric_error;
      }
    }
  }
  update(H_local, b_local);
}

void DirectMethodTracker::eval(const cv::Range& range,
                               const Sophus::SE3d& candidate_T_12) {
  const Eigen::Matrix3d K1_inv = intrinsics1_.K.inverse();
  int count_good = 0;
  double cost_local = 0.0;

  for (size_t i = range.start; i < range.end; ++i) {
    // if (!status_[i]) continue;
    const cv::Point2d& kp1 = p1_[i];

    geometry::PointPixel center_cam2;
    if (!cam2_from_cam1(i, kp1, K1_inv, center_cam2, candidate_T_12)) {
      continue;
    }
    for (int r = -kHalfPatch; r <= kHalfPatch; ++r) {
      for (int c = -kHalfPatch; c <= kHalfPatch; ++c) {
        const cv::Point2d p1 = kp1 + cv::Point2d(c, r);
        const double x2 = center_cam2.x + c;
        const double y2 = center_cam2.y + r;
        count_good += 1;
        // get pixel intensity using image
        const double intensity1 =
            geometry::bilinear_interpolation(img1_, p1.x, p1.y);
        const double intensity2 =
            geometry::bilinear_interpolation(img2_, x2, y2);
        const double photometric_error = intensity2 - intensity1;

        cost_local += 0.5 * photometric_error * photometric_error;
      }
    }
  }
  if (count_good > 0) {
    update(cost_local / count_good);
  }
}

}  // namespace

namespace frontend {

void project_points(const std::vector<double>& depths1,
                    const std::vector<cv::Point2d>& p1,
                    std::vector<cv::Point2d>& p2, std::vector<uchar>& status,
                    const geometry::PinholeCameraIntrinsics& intrinsics1,
                    const geometry::PinholeCameraIntrinsics& intrinsics2,
                    const cv::Size& img2_size, const Sophus::SE3d& T_12) {
  const Eigen::Matrix3d K1_inv = intrinsics1.K.inverse();
  if (p2.size() != p1.size()) {
    p2.resize(p1.size());
  }
  status.resize(p1.size(), true);

  for (size_t i = 0; i < p1.size(); ++i) {
    geometry::PointPixel data_cam2;
    if (geometry::cam2_from_cam1(depths1[i], p1[i], K1_inv, intrinsics2,
                                 img2_size, data_cam2, T_12, kHalfPatch)) {
      p2[i] = cv::Point2d(data_cam2.x, data_cam2.y);
    } else {
      status[i] = false;
    }
  }
}

void direct_method_single_level(
    const cv::Mat& img1, const cv::Mat& img2,
    const std::vector<double>& depths1,
    const geometry::PinholeCameraIntrinsics& intrinsics1,
    const geometry::PinholeCameraIntrinsics& intrinsics2,
    const std::vector<cv::Point2d>& p1, std::vector<uchar>& status,
    Sophus::SE3d& T_12) {
  if (status.size() != p1.size()) {
    status.resize(p1.size(), true);
  }

  double last_cost = [&]() {
    DirectMethodTracker tracker(img1, img2, depths1, intrinsics1, intrinsics2,
                                p1, status, T_12);
    cv::parallel_for_(cv::Range(0, static_cast<int>(p1.size())),
                      [&](const cv::Range& range) { tracker.track(range); });
    return tracker.cost();
  }();
  constexpr double kConverged = 0.00001;

  constexpr int kIters = 10;
  for (int iter = 0; iter < kIters; ++iter) {
    DirectMethodTracker tracker(img1, img2, depths1, intrinsics1, intrinsics2,
                                p1, status, T_12);
    cv::parallel_for_(cv::Range(0, static_cast<int>(p1.size())),
                      [&](const cv::Range& range) { tracker.track(range); });

    const Eigen::Vector<double, kPoseDim> dx = tracker.perturbation();
    tracker.reset_optimization();
    if (dx.norm() < kConverged) {
      break;
    }
    const Sophus::SE3d candidate_T_12 = Sophus::SE3d::exp(dx) * T_12;
    cv::parallel_for_(
        cv::Range(0, static_cast<int>(p1.size())),
        [&](const cv::Range& range) { tracker.eval(range, candidate_T_12); });
    const double candidate_cost = tracker.cost();
    tracker.reset_cost();
    if (iter > 0 && candidate_cost > last_cost) {
      break;
    }
    last_cost = candidate_cost;
    T_12 = candidate_T_12;
  }
}

void direct_method_pyramid(const cv::Mat& img1, const cv::Mat& img2,
                           const std::vector<double>& depths1,
                           const geometry::PinholeCameraIntrinsics& intrinsics1,
                           const geometry::PinholeCameraIntrinsics& intrinsics2,
                           const std::vector<cv::Point2d>& p1,
                           std::vector<uchar>& status, Sophus::SE3d& T_12) {
  constexpr int kPyramids = 4;
  constexpr double kPyramidScale = 0.5;
  constexpr std::array<double, kPyramids> kScales{1.0, 0.5, 0.25, 0.125};

  std::vector<cv::Mat> pyr1;
  std::vector<cv::Mat> pyr2;
  pyr1.reserve(kPyramids);
  pyr2.reserve(kPyramids);
  for (size_t i = 0; i < kPyramids; ++i) {
    if (kScales[i] == 1.0) {
      pyr1.emplace_back(img1);
      pyr2.emplace_back(img2);
    } else {
      pyr1.emplace_back();
      pyr2.emplace_back();

      const cv::Mat& prev_pyr1 = pyr1[i - 1];
      cv::resize(prev_pyr1, pyr1.back(),
                 cv::Size(prev_pyr1.cols * kPyramidScale,
                          prev_pyr1.rows * kPyramidScale));
      const cv::Mat& prev_pyr2 = pyr2[i - 1];
      cv::resize(prev_pyr2, pyr2.back(),
                 cv::Size(prev_pyr2.cols * kPyramidScale,
                          prev_pyr2.rows * kPyramidScale));
    }
  }

  std::vector<cv::Point2d> pyr_p1;
  pyr_p1.reserve(p1.size());
  for (const auto& kp : p1) {
    pyr_p1.emplace_back(kScales.back() * kp);
  }

  if (status.size() != p1.size()) {
    status.resize(p1.size(), true);
  }

  geometry::PinholeCameraIntrinsics pyr_intrinsics1 = intrinsics1;
  geometry::PinholeCameraIntrinsics pyr_intrinsics2 = intrinsics2;
  pyr_intrinsics1.scale(kScales.back());
  pyr_intrinsics2.scale(kScales.back());

  for (int level = kScales.size() - 1; level >= 0; --level) {
    direct_method_single_level(pyr1[level], pyr2[level], depths1,
                               pyr_intrinsics1, pyr_intrinsics2, pyr_p1, status,
                               T_12);
    // optical_flow_one_level(pyr1[level], pyr2[level], pyr_p1, pyr_p2, status);

    if (level > 0) {
      for (size_t i = 0; i < pyr_p1.size(); ++i) {
        pyr_p1[i] /= kPyramidScale;
      }
      pyr_intrinsics1.scale(1.0 / kPyramidScale);
      pyr_intrinsics2.scale(1.0 / kPyramidScale);
    }
  }
}

}  // namespace frontend