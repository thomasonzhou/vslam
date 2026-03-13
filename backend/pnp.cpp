#include "backend/pnp.h"
#include <iostream>

namespace backend{
    Eigen::Matrix<double, 2, 6> error_jacobian_wrt_perturbation(
        const Eigen::Vector3d& point3d_cam2,
        const calib::PinholeCameraIntrinsics& intrinsics2
    ){

        const double fx = intrinsics2.fx();
        const double fy = intrinsics2.fy();
        const double cx = intrinsics2.cx();
        const double cy = intrinsics2.cy();

        Eigen::Matrix<double, 2, 6> J;
        const double x = point3d_cam2[0];
        const double y = point3d_cam2[1];
        const double z = point3d_cam2[2];
        const double z2 = z * z;

        J << -fx / z, 
        0, 
        fx * x / z2, 
        fx * x * y / z2, 
        -fx - fx * x * x / z2, 
        fx * y / z,
        0,
        -fy / z,
        fy * y / z2,
        fy + fy * y * y / z2,
        -fy * y * x / z2,
        -fy * x / z;
        return J;
    }

};