#include "epipolar.h"
#include <opencv2/calib3d.hpp>

void pose_estimation_2d2d(
    const std::vector<cv::KeyPoint>& keypoints1, 
    const std::vector<cv::KeyPoint>& keypoints2,
    const std::vector<cv::DMatch>& matches,
    const PinholeCameraIntrinsics& intrinsics1,
    const PinholeCameraIntrinsics& intrinsics2,
    cv::Mat& c2_R_c1,
    cv::Mat& t_12
){
    std::vector<cv::Point2d> matched1;
    std::vector<cv::Point2d> matched2;

    matched1.reserve(matches.size());
    matched2.reserve(matches.size());

    for (const cv::DMatch& match: matches){
        matched1.push_back(keypoints1[match.queryIdx].pt);
        matched2.push_back(keypoints2[match.trainIdx].pt);
    }

    // constexpr double ransacReprojThreshold = 3.0;
    // constexpr double ransacConfidence = 0.99;
    // cv::Mat fundamental = cv::findFundamentalMat(matched1, matched2, cv::FM_RANSAC, ransacReprojThreshold, ransacConfidence);
    // std::cout << "fundamental matrix: " << std::endl << fundamental << std::endl;

    // cv::Mat homography = cv::findHomography(matched1, matched2, cv::RANSAC, ransacReprojThreshold);
    // std::cout << "homography matrix: " << std::endl << homography << std::endl;

    const cv::Mat distortion1 = cv::Mat::zeros(1, 5, CV_64F);
    const cv::Mat distortion2 = cv::Mat::zeros(1, 5, CV_64F);
    // cv::Mat essential = cv::findEssentialMat(
    //     matched1, matched2, 
    //     intrinsics1.K, distortion1, 
    //     intrinsics2.K, distortion2);
    // std::cout << "essential matrix: " << std::endl << essential << std::endl;

    cv::Mat essential2;
    cv::recoverPose(matched1, matched2, 
        intrinsics1.K, distortion1, 
        intrinsics2.K, distortion2, 
        essential2, c2_R_c1, t_12);

    std::cout << "essential 2 matrix: " << std::endl << essential2 << std::endl;

    // verify epipolar constraint

    const cv::Mat E = hat(t_12) * c2_R_c1;

    for (int i = 0; i < matches.size(); ++i){
        cv::Mat p1 = homogenous_coordinates(pixel_to_camera(matched1[i], intrinsics1));
        cv::Mat p2 = homogenous_coordinates(pixel_to_camera(matched2[i], intrinsics2));
        
        const cv::Mat epipolar_constraint = p2.t() * E * p1;
        std::cout << epipolar_constraint << std::endl;
    }

};

