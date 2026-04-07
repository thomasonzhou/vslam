#pragma once
#include <opencv2/core.hpp>
#include <vector>

namespace frontend {

[[nodiscard]] std::vector<cv::Point2d> sample_pixels_uniform(
    const cv::Mat& img, const size_t samples, const double border) {
  constexpr uint64 kSeed = 88;
  cv::RNG rng(kSeed);

  std::vector<cv::Point2d> pixels;
  pixels.reserve(samples);

  const double x_min = border;
  const double x_max = img.cols - border - 1.0;
  const double y_min = border;
  const double y_max = img.rows - border - 1.0;

  for (size_t i = 0; i < samples; ++i) {
    pixels.emplace_back(rng.uniform(x_min, x_max), rng.uniform(y_min, y_max));
  }
  return pixels;
}

}  // namespace frontend