#pragma once
#include <opencv2/core.hpp>

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