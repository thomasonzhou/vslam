#include <opencv2/core.hpp>
#include <vector>

struct PinholeCameraIntrinsics{
    cv::Mat K;

    inline double fx() const noexcept {
        return K.at<double>(0,0);
    };
    inline double fy() const noexcept {
        return K.at<double>(1,1);
    };
    inline double cx() const noexcept {
        return K.at<double>(0,2);
    };
    inline double cy() const noexcept {
        return K.at<double>(1,2);
    };
};


void pose_estimation_2d2d(
    const std::vector<cv::KeyPoint>& keypoints1, 
    const std::vector<cv::KeyPoint>& keypoints2,
    const std::vector<cv::DMatch>& matches,
    const PinholeCameraIntrinsics& intrinsics1,
    const PinholeCameraIntrinsics& intrinsics2,
    cv::Mat& c1_R_c2,
    cv::Mat& t_21
);

cv::Point2d pixel_to_camera(const cv::Point2d& pixel, const PinholeCameraIntrinsics& intrinsics);
cv::Point2d camera_to_pixel(const cv::Point2d& camera, const PinholeCameraIntrinsics& intrinsics);