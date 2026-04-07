#include "estimation/epipolar/epipolar.h"

namespace estimation::epipolar {
void pose_estimation_2d2d(const std::vector<cv::KeyPoint>& keypoints1,
                          const std::vector<cv::KeyPoint>& keypoints2,
                          const std::vector<cv::DMatch>& matches,
                          const geometry::PinholeCameraIntrinsics& intrinsics1,
                          const geometry::PinholeCameraIntrinsics& intrinsics2,
                          cv::Mat& c2_R_c1, cv::Mat& t_12) {
  std::vector<cv::Point2d> matched1;
  std::vector<cv::Point2d> matched2;

  matched1.reserve(matches.size());
  matched2.reserve(matches.size());

  for (const cv::DMatch& match : matches) {
    matched1.push_back(keypoints1[match.queryIdx].pt);
    matched2.push_back(keypoints2[match.trainIdx].pt);
  }

  // constexpr double ransacReprojThreshold = 3.0;
  // constexpr double ransacConfidence = 0.99;
  // cv::Mat fundamental = cv::findFundamentalMat(matched1, matched2,
  // cv::FM_RANSAC, ransacReprojThreshold, ransacConfidence); std::cout <<
  // "fundamental matrix: " << std::endl << fundamental << std::endl;

  // cv::Mat homography = cv::findHomography(matched1, matched2, cv::RANSAC,
  // ransacReprojThreshold); std::cout << "homography matrix: " << std::endl <<
  // homography << std::endl;

  const cv::Mat distortion1 = cv::Mat::zeros(1, 5, CV_64F);
  const cv::Mat distortion2 = cv::Mat::zeros(1, 5, CV_64F);
  // cv::Mat essential = cv::findEssentialMat(
  //     matched1, matched2,
  //     intrinsics1.K, distortion1,
  //     intrinsics2.K, distortion2);
  // std::cout << "essential matrix: " << std::endl << essential << std::endl;

  cv::Mat essential2;
  cv::recoverPose(matched1, matched2, intrinsics1.camera_matrix_cv(),
                  distortion1, intrinsics2.camera_matrix_cv(), distortion2,
                  essential2, c2_R_c1, t_12);

  std::cout << "essential 2 matrix: " << '\n' << essential2 << '\n';

  // verify epipolar constraint

  const cv::Mat E = geometry::hat(t_12) * c2_R_c1;

  for (int i = 0; i < matches.size(); ++i) {
    cv::Mat p1 = geometry::homogenous_coordinates(
        geometry::pixel_to_camera(matched1[i], intrinsics1));
    cv::Mat p2 = geometry::homogenous_coordinates(
        geometry::pixel_to_camera(matched2[i], intrinsics2));

    const cv::Mat epipolar_constraint = p2.t() * E * p1;
    std::cout << epipolar_constraint << '\n';
  }
};

void triangulation(const std::vector<cv::KeyPoint>& keypoints1,
                   const std::vector<cv::KeyPoint>& keypoints2,
                   const std::vector<cv::DMatch>& matches,
                   const geometry::PinholeCameraIntrinsics& intrinsics1,
                   const geometry::PinholeCameraIntrinsics& intrinsics2,
                   const cv::Mat& R, const cv::Mat& t,
                   std::vector<cv::Point3d>& points) {
  std::vector<cv::Point2d> points1;
  std::vector<cv::Point2d> points2;

  for (const cv::DMatch& match : matches) {
    points1.push_back(
        geometry::pixel_to_camera(keypoints1[match.queryIdx].pt, intrinsics1));
    points2.push_back(
        geometry::pixel_to_camera(keypoints2[match.trainIdx].pt, intrinsics2));
  }

  cv::Mat T1 = (cv::Mat_<double>(3, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0);
  cv::Mat T2 = (cv::Mat_<double>(3, 4) << R.at<double>(0, 0),
                R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
                R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
                t.at<double>(1, 0), R.at<double>(2, 0), R.at<double>(2, 1),
                R.at<double>(2, 2), t.at<double>(2, 0));

  cv::Mat points4d;
  cv::triangulatePoints(T1, T2, points1, points2, points4d);

  for (size_t i = 0; i < points4d.cols; ++i) {
    cv::Mat x = points4d.col(i);
    x /= x.at<double>(3, 0);

    points.emplace_back(x.at<double>(0, 0), x.at<double>(1, 0),
                        x.at<double>(2, 0));
  }
};
}  // namespace estimation::epipolar