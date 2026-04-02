#include <string>
#include <opencv2/imgcodecs.hpp>
#include <vector>
#include <opencv2/imgproc.hpp>
#include "frontend/optical_flow.h"
#include "viz/opencv_viz.h"

int main(int argc, char **argv) {
  const std::string file1 = "../images/LK1.png";
  const std::string file2 = "../images/LK2.png";

  cv::Mat img1 = cv::imread(file1, cv::IMREAD_GRAYSCALE);
  cv::Mat img2 = cv::imread(file2, cv::IMREAD_GRAYSCALE);
  if (img1.empty() || img2.empty()) return -1;

  std::vector<cv::Point2d> p1;
  constexpr int kNoMaxFeatures = 0;
  constexpr double kQuality = 0.1;
  constexpr double kMinDist = 7.0;
  cv::goodFeaturesToTrack(img1, p1, kNoMaxFeatures, kQuality, kMinDist);
  

  std::vector<bool> status;
  std::vector<double> error;
  std::vector<cv::Point2d> p2;
  frontend::optical_flow_pyramid(img1, img2, p1, p2, status, error);

  viz::viz_match(img1, img2, p1, p2, status);

  return 0;
}
