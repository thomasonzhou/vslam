#include "utils/eval.h"
#include "frontend/orb.h"
#include <string>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

int main(int argc, char** argv) {

  // frontend

  std::string file1 = "../1.png";
  std::string file2 = "../2.png";
  cv::Mat img1 = cv::imread(file1, cv::IMREAD_GRAYSCALE);
  cv::Mat img2 = cv::imread(file2, cv::IMREAD_GRAYSCALE);

  // oriented FAST keypoints
  std::vector<cv::KeyPoint> keypoints1;
  std::vector<cv::KeyPoint> keypoints2;

  constexpr int FAST_threshold = 40;
  cv::FAST(img1, keypoints1, FAST_threshold);
  cv::FAST(img2, keypoints2, FAST_threshold);

  // rotated BRIEF descriptors
  std::vector<Descriptor> descriptors1;
  std::vector<Descriptor> descriptors2;

  compute_orb(img1, keypoints1, descriptors1);
  compute_orb(img2, keypoints2, descriptors2);

  // match
  std::vector<cv::DMatch> matches;
  brute_force_match(descriptors1, descriptors2, matches);

  // cv::Mat img1_descriptors;
  // cv::Mat img2_descriptors;
  cv::Mat match_img;
  cv::drawMatches(img1, keypoints1, img2, keypoints2, matches, match_img);

  cv::imshow("matched image", match_img);
  cv::waitKey(0);

  // backend

  // evaluate
  // compare_trajectories();

  return 0;
}
