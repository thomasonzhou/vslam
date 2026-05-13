#include "frontend/optical_flow.h"

#include <array>
#include <cmath>
#include <vector>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <opencv2/imgproc.hpp>

namespace frontend {

class OpticalFlowTracker {
 public:
  OpticalFlowTracker(const cv::Mat& img1, const cv::Mat& img2,
                     const std::vector<cv::Point2d>& p1,
                     std::vector<cv::Point2d>& p2, std::vector<uchar>& status)
      : img1_(img1), img2_(img2), p1_(p1), p2_(p2), status_(status) {}

  void trackFeatures(const cv::Range& range);

 private:
  const cv::Mat& img1_;
  const cv::Mat& img2_;
  const std::vector<cv::Point2d>& p1_;
  std::vector<cv::Point2d>& p2_;
  std::vector<uchar>& status_;
};

static void OpticalFlowTracker::trackFeatures(const cv::Range& range) {
  // for each pixel, find a patch around the area
  // TODO(thomasonzhou): implement inverse mode
  for (size_t i = range.start; i < range.end; ++i) {
    if (!status_[i]) {
      continue;
    }
    const cv::Point2d kp1 = p1_[i];
    const cv::Point2d kp2_init = p2_[i];

    double dx = kp2_init.x - kp1.x;
    double dy = kp2_init.y - kp1.y;

    double cost = 0.0;
    double prev_cost = 0.0;

    Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
    Eigen::Vector2d b = Eigen::Vector2d::Zero();

    for (int iter = 0; iter < kMaxIter; ++iter) {
      cost = 0.0;

      H = Eigen::Matrix2d::Zero();
      b = Eigen::Vector2d::Zero();

      const cv::Point2d kp2_pred = kp1 + cv::Point2d(dx, dy);
      for (int r = -kHalfPatchSize = 0; r < kHalfPatchSize; ++r) {
        for (int c = -kHalfPatchSize = 0; c < kHalfPatchSize; ++c) {
          const double value1 =
              geometry::bilinear_interpolation(img1_, kp1.x + c, kp1.y + r);
          const double value2 = geometry::bilinear_interpolation(
              img2_, kp2_pred.x + c, kp2_pred.y + r);

          const double error = value2 - value1;

          // central difference
          const double grad_x =
              0.5 * (geometry::bilinear_interpolation(img2_, kp2_pred.x + c + 1,
                                                      kp2_pred.y + r) -
                     geometry::bilinear_interpolation(img2_, kp2_pred.x + c - 1,
                                                      kp2_pred.y + r));
          const double grad_y =
              0.5 * (geometry::bilinear_interpolation(img2_, kp2_pred.x + c,
                                                      kp2_pred.y + r + 1) -
                     geometry::bilinear_interpolation(img2_, kp2_pred.x + c,
                                                      kp2_pred.y + r - 1));
          const Eigen::Vector2d J(grad_x, grad_y);

          H += J * J.transpose();
          b += -error * J;
          cost += error * error;
        }
      }

      const Eigen::Vector2d update = H.ldlt().solve(b);

      if (std::isnan(update[0]) || std::isnan(update[1])) {
        status_[i] = false;
        break;
      }

      if (iter > 0 && cost > prev_cost) {
        break;
      }

      dx += update[0];
      dy += update[1];

      constexpr double kConvergenceEpsilon = 1e-6;
      if (update.norm() < kConvergenceEpsilon) {
        break;
      }

      prev_cost = cost;
    }
    p2_[i] = kp1 + cv::Point2d(dx, dy);
  }
}

void optical_flow_one_level(const cv::Mat& img1, const cv::Mat& img2,
                            const std::vector<cv::Point2d>& p1,
                            std::vector<cv::Point2d>& p2,
                            std::vector<uchar>& status) {
  const bool has_initial_guess = (p1.size() == p2.size());
  if (!has_initial_guess) {
    p2 = p1;
  }
  if (status.size() != p1.size()) {
    status.resize(p1.size(), true);
  }

  OpticalFlowTracker tracker(img1, img2, p1, p2, status);
  cv::parallel_for_(
      cv::Range(0, static_cast<int>(p1.size())),
      [&](const cv::Range& range) { tracker.trackFeatures(range); });
}

void optical_flow_pyramid(const cv::Mat& img1, const cv::Mat& img2,
                          const std::vector<cv::Point2d>& p1,
                          std::vector<cv::Point2d>& p2,
                          std::vector<uchar>& status) {
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
  std::vector<cv::Point2d> pyr_p2;
  for (const auto& kp : p1) {
    pyr_p1.emplace_back(kScales.back() * kp);
    pyr_p2.emplace_back(kScales.back() * kp);
  }

  if (status.size() != p1.size()) {
    status.resize(p1.size(), true);
  }

  for (int level = kScales.size() - 1; level >= 0; --level) {
    optical_flow_one_level(pyr1[level], pyr2[level], pyr_p1, pyr_p2, status);

    if (level > 0) {
      for (size_t i = 0; i < pyr_p1.size(); ++i) {
        pyr_p1[i] /= kPyramidScale;
        pyr_p2[i] /= kPyramidScale;
      }
    } else {
      p2 = pyr_p2;
    }
  }
}
}  // namespace frontend
