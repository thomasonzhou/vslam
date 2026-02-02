#include "utils/eval.h"
#include "frontend/orb.h"
#include "backend/epipolar.h"
#include <string>
#include <opencv2/imgcodecs.hpp>

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

  // backend
  cv::Mat c1_R_c2;
  cv::Mat t_21;

  const PinholeCameraIntrinsics intrinsics1(521.0, 521.0, 325.1, 249.7);
  const PinholeCameraIntrinsics intrinsics2 = intrinsics1;
  pose_estimation_2d2d(keypoints1, keypoints2, matches, intrinsics1, intrinsics2, c1_R_c2, t_21);

  std::cout << "c1_R_c2 " << c1_R_c2 << std::endl;
  std::cout << "t_21 " << t_21 << std::endl;

  // PnP
  std::string depth_file1 = "../depth1.png";
  cv::Mat depth1 = cv::imread(depth_file1, cv::IMREAD_UNCHANGED);

  std::vector<cv::Point3d> points3d;
  std::vector<cv::Point2d> points2d_img2;

  constexpr double depth_scaling = 5000.0;
  for (const cv::DMatch& match: matches){
    const cv::Point2d pixel1 = keypoints1[match.queryIdx].pt;
    unsigned short d = depth1.ptr<unsigned short>(static_cast<int>(pixel1.y))[static_cast<int>(pixel1.x)];
    if (d <= 0){
      continue;
    }

    const double metric_depth = d / depth_scaling;
    cv::Point2d point1 = pixel_to_camera(pixel1, intrinsics1);
    points3d.emplace_back(point1.x * metric_depth, point1.y * metric_depth, metric_depth);
    points2d_img2.push_back(keypoints2[match.trainIdx].pt);
  }

  cv::Mat r_vec;
  cv::Mat t_vec;

  constexpr bool use_extrinsic_guess = false;
  cv::solvePnP(points3d, points2d_img2, intrinsics2.K, cv::Mat(), r_vec, t_vec, use_extrinsic_guess);

  cv::Mat R_pnp;
  cv::Rodrigues(r_vec, R_pnp);

  std::cout << "Comparing results from epipolar geometry vs PnP" << std::endl;
  std::cout << c1_R_c2 << std::endl;
  std::cout << R_pnp << std::endl;

  // note how the translation is very different, due to a difference in depth.
  std::cout << t_21 << std::endl;
  std::cout << t_vec << std::endl;

  return 0;
}
