#include "backend/epipolar.h"
#include "backend/pnp_naive.h"
#include "backend/pnp_g2o.h"
#include "calib/calib.h"
#include "core/coordinate_utils.h"
#include "frontend/orb.h"
#include "utils/eval.h"
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
  std::vector<frontend::Descriptor> descriptors1;
  std::vector<frontend::Descriptor> descriptors2;

  frontend::compute_orb(img1, keypoints1, descriptors1);
  frontend::compute_orb(img2, keypoints2, descriptors2);

  // match
  std::vector<cv::DMatch> matches;
  frontend::brute_force_match(descriptors1, descriptors2, matches);

  // backend

  // rotation from camera 1 to camera 2
  cv::Mat c2_R_c1;
  // translation from camera 1 to camera 2, add to the pose of camera 2 to get to camera 1 coordinates
  cv::Mat t_12; 

  const calib::PinholeCameraIntrinsics intrinsics1(521.0, 521.0, 325.1, 249.7);
  const calib::PinholeCameraIntrinsics intrinsics2 = intrinsics1;

  // PnP
  std::string depth_file1 = "../depth1.png";
  cv::Mat depth1 = cv::imread(depth_file1, cv::IMREAD_UNCHANGED);

  std::vector<cv::Point3d> points3d_cam1;
  std::vector<cv::Point2d> points2d_img2;

  constexpr double depth_scaling = 5000.0;
  for (const cv::DMatch& match: matches){
    const cv::Point2d pixel1 = keypoints1[match.queryIdx].pt;
    unsigned short d = depth1.ptr<unsigned short>(static_cast<int>(pixel1.y))[static_cast<int>(pixel1.x)];
    if (d <= 0){
      continue;
    }

    const double metric_depth = d / depth_scaling;
    cv::Point2d point1 = core::pixel_to_camera(pixel1, intrinsics1);
    points3d_cam1.emplace_back(point1.x * metric_depth, point1.y * metric_depth, metric_depth);
    points2d_img2.push_back(keypoints2[match.trainIdx].pt);
  }

  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> points3d_eigen;
  std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> points2d_eigen;
  for(size_t i = 0; i < points3d_cam1.size(); ++i){
    points3d_eigen.emplace_back(points3d_cam1[i].x, points3d_cam1[i].y, points3d_cam1[i].z);
    points2d_eigen.emplace_back(points2d_img2[i].x, points2d_img2[i].y);
  }



  Sophus::SE3d c2_T_c1;
  backend::G2OPnPSolver solver;
  // backend::NaivePnPSolver solver;
  solver.solve(points3d_eigen, points2d_eigen, intrinsics2, c2_T_c1);

  std::cout << c2_T_c1.matrix() << std::endl;
  return 0;
}
