#pragma once
#include <opencv2/core.hpp>
#include <vector>

#include "geometry/bilinear.h"

namespace frontend {

constexpr int kHalfPatchSize = 4;
constexpr int kMaxIter = 10;

void optical_flow_pyramid(const cv::Mat& img1, const cv::Mat& img2,
                          const std::vector<cv::Point2d>& p1,
                          std::vector<cv::Point2d>& p2,
                          std::vector<uchar>& status);

}  // namespace frontend