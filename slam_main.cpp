#include <string>
#include <opencv2/imgcodecs.hpp>
#include <vector>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui.hpp>

int main(int argc, char **argv) {
  const std::string file1 = "../optical_flow/LK1.png";
  const std::string file2 = "../optical_flow/LK2.png";

  cv::Mat img1 = cv::imread(file1, cv::IMREAD_GRAYSCALE);
  cv::Mat img2 = cv::imread(file2, cv::IMREAD_GRAYSCALE);
  if (img1.empty() || img2.empty()) return -1;

  std::vector<cv::Point2f> p1;
  constexpr int kNoMaxFeatures = 0;
  constexpr double kQuality = 0.1;
  constexpr double kMinDist = 7.0;
  cv::goodFeaturesToTrack(img1, p1, kNoMaxFeatures, kQuality, kMinDist);
  

  std::vector<uchar> status;
  std::vector<float> error;
  std::vector<cv::Point2f> p2;
  cv::calcOpticalFlowPyrLK(img1, img2, p1, p2, status, error);

  cv::Mat full_img_gray;
  cv::hconcat(img1, img2, full_img_gray);

  cv::Mat full_img;
  cv::cvtColor(full_img_gray, full_img, cv::COLOR_GRAY2BGR);

  const int kRadius = 3;
  const cv::Scalar kGreen(0, 255, 0);
  const cv::Scalar kRed(0, 0, 255);
  for (size_t i = 0; i < std::min(p1.size(), p2.size()); ++i){
    if(!status[i]) continue;

    const cv::Point2f pt1 = p1[i];
    const cv::Point2f pt2 = p2[i] + cv::Point2f(static_cast<double>(img1.cols), 0.0);

    cv::circle(full_img, pt1, kRadius, kGreen);
    cv::circle(full_img, pt2, kRadius, kGreen);
    cv::line(full_img, pt1, pt2, kRed);
  }

  cv::imshow("LK Optical Flow Match", full_img);
  cv::waitKey(0);

  return 0;
}
