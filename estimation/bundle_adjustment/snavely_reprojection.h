#pragma once

#include <ceres/autodiff_cost_function.h>
#include <ceres/rotation.h>
#include <array>
#include "util/bal.h"


namespace estimation::bundle_adjustment {

struct SnavelyReprojectionError{
    SnavelyReprojectionError(const double observed_u, const double observed_v): observed_u(observed_u), observed_v(observed_v){}

    template <typename T>
    bool operator()(const  T* const camera, const T* const point, T* residuals) const{
        std::array<T, util::bal::kTranslationDim> point_cam;
        ceres::AngleAxisRotatePoint(camera, point, point_cam.data());
        for (size_t i = 0; i < util::bal::kTranslationDim; ++i){
            point_cam[i] += camera[util::bal::kRotationDim + i];
        }
        const T depth = point_cam[2];
        for (size_t i = 0; i < util::bal::kPixelDim; ++i){
            point_cam[i] /= -depth;
        } // projection plane is behind camera, so use negative
        const T x = point_cam[0];
        const T y = point_cam[1];
        const T radius_2 = x * x + y * y;
        const T k1_dist = camera[util::bal::kK1Idx];
        const T k2_dist = camera[util::bal::kK2Idx];
        const T distortion_correction = 1.0 + radius_2 * (k1_dist + radius_2 * k2_dist);
        
        const T focal_length = camera[util::bal::kFocalIdx];
        const T pred_u = focal_length * distortion_correction * x;
        const T pred_v = focal_length * distortion_correction * y;

        residuals[0] = pred_u - static_cast<T>(observed_u);
        residuals[1] = pred_v - static_cast<T>(observed_v);
        return true;
    }

    static ceres::CostFunction* create(const double observed_u, const double observed_v){
        return new ceres::AutoDiffCostFunction<SnavelyReprojectionError, util::bal::kPixelDim, util::bal::kBALCameraBlockSize, util::bal::kTranslationDim> (observed_u, observed_v);
    }

    double observed_u;
    double observed_v;
};
 
}  // namespace estimation::bundle_adjustment