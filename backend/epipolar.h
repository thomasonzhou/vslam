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

    PinholeCameraIntrinsics(const double fx, const double fy, const double cx, const double cy){
        K = (cv::Mat_<double>(3,3) << fx, 0.0, cx, 
        0.0, fy, cy,
        0.0, 0.0, 1.0);
    }
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

cv::Point2d pixel_to_camera(const cv::Point2d& pixel, const PinholeCameraIntrinsics& intrinsics){
    return cv::Point2d((pixel.x - intrinsics.cx()) / intrinsics.fx(),
                        (pixel.y - intrinsics.cy()) / intrinsics.fy());
};

cv::Mat homogenous_coordinates(const cv::Point2d& xy){
    return (cv::Mat_<double> (3, 1) << xy.x, xy.y, 1.0);
};

cv::Mat hat(const cv::Mat& t){
    const double x = t.at<double>(0, 0);
    const double y = t.at<double>(1, 0);
    const double z = t.at<double>(2, 0);
    return (cv::Mat_<double>(3, 3) << 0, -z, y, 
                                    z, 0, -x,
                                    -y, x, 0);
};