#include "utils/eval.h"
#include "frontend/orb.h"
#include "backend/epipolar.h"
#include <string>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

cv::Scalar depthmap_color(double depth){
  constexpr double upper = 50.0;
  constexpr double lower = 10.0;
  constexpr double range = upper - lower;

  if (depth < lower) depth = lower;
  if (depth > upper) depth = upper;
  return cv::Scalar(depth * 255.0 / range, 0.0, (1 - (depth / range)) * 255.0);
}

int main(int argc, char** argv) {

  // frontend
  std::string file1 = "../1.png";
  std::string file2 = "../2.png";
  cv::Mat img1 = cv::imread(file1, cv::IMREAD_COLOR);
  cv::Mat img2 = cv::imread(file2, cv::IMREAD_COLOR);

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

  cv::Mat match_img;
  cv::drawMatches(img1, keypoints1, img2, keypoints2, matches, match_img);

  cv::imshow("matched image", match_img);

  // backend
  cv::Mat c1_R_c2;
  cv::Mat t_21;

  const PinholeCameraIntrinsics intrinsics1(521.0, 521.0, 325.1, 249.7);
  const PinholeCameraIntrinsics intrinsics2 = intrinsics1;
  pose_estimation_2d2d(keypoints1, keypoints2, matches, intrinsics1, intrinsics2, c1_R_c2, t_21);

  std::cout << "c1_R_c2 " << c1_R_c2 << std::endl;
  std::cout << "t_21 " << t_21 << std::endl;
  // evaluate
  // compare_trajectories();

  std::vector<cv::Point3d> points3d;
  triangulation(keypoints1, keypoints2, matches, intrinsics1, intrinsics2, c1_R_c2, t_21, points3d);

  cv::Mat img1_tri = img1.clone();
  cv::Mat img2_tri = img2.clone();
  for (size_t i = 0; i < matches.size(); ++i){
    cv::Point2d pt1 = pixel_to_camera(keypoints1[matches[i].queryIdx].pt, intrinsics1); 
    cv::Mat pt2_trans = c1_R_c2 * (cv::Mat_<double>(3, 1) << points3d[i].x, points3d[i].y, points3d[i].z) + t_21;
    
    const double depth1 = points3d[i].z;
    const double depth2 = pt2_trans.at<double>(2, 0);

    constexpr int radius = 2;
    constexpr int thickness = 2;
    cv::circle(img1_tri, keypoints1[matches[i].queryIdx].pt, radius, depthmap_color(depth1), thickness);
    cv::circle(img2_tri, keypoints2[matches[i].trainIdx].pt, radius, depthmap_color(depth2), thickness);
  }
  cv::namedWindow("img1", cv::WINDOW_NORMAL);
  cv::namedWindow("img2", cv::WINDOW_NORMAL);
  cv::imshow("img1", img1_tri);
  cv::imshow("img2", img2_tri);

  cv::waitKey(0);

  return 0;
}
