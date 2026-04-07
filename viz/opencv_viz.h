#pragma once
#include <opencv2/highgui.hpp>

namespace viz {

constexpr int kRadius = 3;
constexpr int kThickness = 1;
const cv::Scalar kGreen(0, 255, 0);
const cv::Scalar kRed(0, 0, 255);

void viz_match(const cv::Mat& img1, const cv::Mat& img2,
               const std::vector<cv::Point2d>& p1,
               const std::vector<cv::Point2d>& p2,
               const std::vector<uchar>& match_status) {
  cv::Mat full_img_gray;
  cv::hconcat(img1, img2, full_img_gray);

  cv::Mat full_img;
  if (full_img_gray.channels() == 1) {
    cv::cvtColor(full_img_gray, full_img, cv::COLOR_GRAY2BGR);
  } else {
    full_img = full_img_gray;
  }

  for (size_t i = 0; i < std::min(p1.size(), p2.size()); ++i) {
    if (!match_status[i]) continue;

    const cv::Point2d pt1 = p1[i];
    const cv::Point2d pt2 =
        p2[i] + cv::Point2d(static_cast<double>(img1.cols), 0.0);

    cv::circle(full_img, pt1, kRadius, kGreen);
    cv::circle(full_img, pt2, kRadius, kGreen);
    cv::line(full_img, pt1, pt2, kRed, kThickness);
  }

  cv::imshow("Point Match", full_img);
  cv::waitKey(0);
}

void viz_match_overlay(const cv::Mat& img, const std::vector<cv::Point2d>& p1,
                       const std::vector<cv::Point2d>& p2,
                       const std::vector<uchar>& match_status) {
  cv::Mat full_img = img;
  if (full_img.channels() == 1) {
    cv::cvtColor(img, full_img, cv::COLOR_GRAY2BGR);
  }

  for (size_t i = 0; i < std::min(p1.size(), p2.size()); ++i) {
    if (!match_status[i]) continue;

    const cv::Point2d pt1 = p1[i];
    const cv::Point2d pt2 = p2[i];

    // cv::circle(full_img, pt1, kRadius, kGreen);
    cv::circle(full_img, pt2, kRadius, kGreen);
    cv::line(full_img, pt1, pt2, kGreen, kThickness);
  }

  cv::imshow("Point Match", full_img);
  cv::waitKey(0);
}

}  // namespace viz