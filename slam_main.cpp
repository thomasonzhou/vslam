#include <string>
#include <vector>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "frontend/point_sampling.h"
#include "frontend/direct_method.h"

int main(int argc, char **argv) {
  const std::string file1 = "../images/left.png";
  const std::string file2 = "../images/000001.png";

  const std::string disparity_file1 = "../images/disparity.png";

  cv::Mat img1 = cv::imread(file1, cv::IMREAD_GRAYSCALE);
  cv::Mat img2 = cv::imread(file2, cv::IMREAD_GRAYSCALE);
  cv::Mat disparity_img1 = cv::imread(disparity_file1, cv::IMREAD_UNCHANGED);
  if (img1.empty() || img2.empty() || disparity_img1.empty()) return -1;

  constexpr double kBaselineMetersKITTI = 0.573;
  const geometry::PinholeCameraIntrinsics kIntrinsicsKITTI(718.856, 718.856, 607.1928, 185.2157);

  constexpr int kSamplesToTrack = 1000;
  constexpr int kBorder = 2;
  const std::vector<cv::Point2d> p1 = frontend::sample_pixels_uniform(img1, kSamplesToTrack, kBorder);

  std::vector<uchar> status;

  Sophus::SE3d T_12;

  frontend::direct_method_single_level(img1, img2, disparity_img1, kBaselineMetersKITTI, kIntrinsicsKITTI, kIntrinsicsKITTI, p1, status, T_12);
  std::cout << "pose: " << T_12.matrix3x4() << std::endl;

  return 0;
}
