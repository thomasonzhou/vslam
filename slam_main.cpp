#include "backend/icp.h"
#include "calib/calib.h"
#include "core/coordinate_utils.h"
#include "frontend/orb.h"
#include "utils/eval.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <string>

int main(int argc, char **argv) {

  // frontend
  std::string file1 = "../1.png";
  std::string file2 = "../2.png";
  cv::Mat img1 = cv::imread(file1, cv::IMREAD_COLOR);
  cv::Mat img2 = cv::imread(file2, cv::IMREAD_COLOR);
  cv::Mat img1_gray;
  cv::Mat img2_gray;
  cv::cvtColor(img1, img1_gray, cv::COLOR_BGR2GRAY);
  cv::cvtColor(img2, img2_gray, cv::COLOR_BGR2GRAY);

  // oriented FAST keypoints
  std::vector<cv::KeyPoint> keypoints1;
  std::vector<cv::KeyPoint> keypoints2;

  constexpr int FAST_threshold = 40;
  cv::FAST(img1_gray, keypoints1, FAST_threshold);
  cv::FAST(img2_gray, keypoints2, FAST_threshold);

  // rotated BRIEF descriptors
  std::vector<frontend::Descriptor> descriptors1;
  std::vector<frontend::Descriptor> descriptors2;

  frontend::compute_orb(img1_gray, keypoints1, descriptors1);
  frontend::compute_orb(img2_gray, keypoints2, descriptors2);

  // match
  std::vector<cv::DMatch> matches;
  frontend::brute_force_match(descriptors1, descriptors2, matches);

  // backend

  // rotation from camera 1 to camera 2
  cv::Mat c2_R_c1;
  // translation from camera 1 to camera 2, add to the pose of camera 2 to get
  // to camera 1 coordinates
  cv::Mat t_12;

  const calib::PinholeCameraIntrinsics intrinsics1(521.0, 521.0, 325.1, 249.7);
  const calib::PinholeCameraIntrinsics intrinsics2 = intrinsics1;

  // ICP
  std::string depth_file1 = "../depth1.png";
  std::string depth_file2 = "../depth2.png";
  cv::Mat depth1 = cv::imread(depth_file1, cv::IMREAD_UNCHANGED);
  cv::Mat depth2 = cv::imread(depth_file2, cv::IMREAD_UNCHANGED);

  std::vector<cv::Point3d> points3d_cam1;
  std::vector<cv::Point3d> points3d_cam2;

  constexpr double depth_scaling = 5000.0;
  for (const cv::DMatch &match : matches) {
    const cv::Point2d pixel1 = keypoints1[match.queryIdx].pt;
    unsigned short d1 = depth1.ptr<unsigned short>(
        static_cast<int>(pixel1.y))[static_cast<int>(pixel1.x)];
    if (d1 <= 0) continue;

    const cv::Point2d pixel2 = keypoints2[match.trainIdx].pt;
    unsigned short d2 = depth2.ptr<unsigned short>(
      static_cast<int>(pixel2.y))[static_cast<int>(pixel2.x)];
    if (d2 <= 0) continue;

    const double metric_depth1 = d1 / depth_scaling;
    const cv::Point2d point1 = core::pixel_to_camera(pixel1, intrinsics1);
    points3d_cam1.emplace_back(point1.x * metric_depth1, point1.y * metric_depth1,
                               metric_depth1);

    const double metric_depth2 = d2 / depth_scaling;
    const cv::Point2d point2 = core::pixel_to_camera(pixel2, intrinsics2);
    points3d_cam2.emplace_back(point2.x * metric_depth2, point2.y * metric_depth2, metric_depth2);
  }

  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
      points3d_eigen1;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
      points3d_eigen2;
  for (size_t i = 0; i < points3d_cam1.size(); ++i) {
    points3d_eigen1.emplace_back(points3d_cam1[i].x, points3d_cam1[i].y,
                                points3d_cam1[i].z);
    points3d_eigen2.emplace_back(points3d_cam2[i].x, points3d_cam2[i].y,
                                points3d_cam2[i].z);
  }

  Sophus::SE3d c2_T_c1;
  
  backend::point_to_point_svd(points3d_eigen1, points3d_eigen2, c2_T_c1);
  std::cout << c2_T_c1.matrix() << std::endl;
  return 0;
}
