#pragma once

#include "util/bal.h"
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_multi_edge.h>
#include <Eigen/Core>
#include <sophus/se3.hpp>
// as an exercise, optimize
// intrinsics (focal length, distortion parameters)
// points
// poses



namespace estimation::bundle_adjustment {

constexpr size_t kFocalLengthIdx = 0;
constexpr size_t kK1Idx = 1;
constexpr size_t kK2Idx = 2;
class VertexIntrinsics: public g2o::BaseVertex<util::bal::kIntrinsicsDim, Eigen::Vector3d>{
public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
void setToOriginImpl() override{
    _estimate = Eigen::Vector3d::Zero(); // is this a good estimate?
    // _estimate[kFocalLengthIdx] = util::bal::kFocalLengthGuess;
}

void oplusImpl(const double* update) override{
    Eigen::Map<const Eigen::Vector3d> update_eigen(update);
    _estimate += update_eigen;
}

Eigen::Vector2d project(const Eigen::Vector3d& camera_point) const{

    //leading negative because of BAL storage representation
    // *note that this does not hold for the final dataset
    const double x = -camera_point[0] / camera_point[2];
    const double y = -camera_point[1] / camera_point[2];
    const double radius_2 = x * x + y * y;

    const double distortion_correction = \
        1.0 + radius_2 * (_estimate[kK1Idx] + radius_2 * _estimate[kK2Idx]);

    const double focal_length = _estimate[kFocalLengthIdx];
    return Eigen::Vector2d(focal_length * distortion_correction * x,
                            focal_length * distortion_correction * y);
}

bool read(std::istream& in) override {return true;}
bool write(std::ostream& out) const override {return true;}

};


class VertexPose: public g2o::BaseVertex<util::bal::kPoseDim, Sophus::SE3d>{
public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
void setToOriginImpl() override{
    _estimate = Sophus::SE3d();
}

void oplusImpl(const double* update) override{
    Eigen::Map<const Eigen::Vector<double, util::bal::kPoseDim>> update_eigen(update);
    _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
}

bool read(std::istream& in) override {return true;}
bool write(std::ostream& out) const override {return true;}
};

class VertexPoint: public g2o::BaseVertex<util::bal::kTranslationDim, Eigen::Vector3d>{
public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
void setToOriginImpl() override{
    _estimate = Eigen::Vector3d::Zero();
}

void oplusImpl(const double* update) override{
    Eigen::Map<const Eigen::Vector<double, util::bal::kTranslationDim>> update_eigen(update);
    _estimate += update_eigen;
}

bool read(std::istream& in) override {return true;}
bool write(std::ostream& out) const override {return true;}
};

constexpr size_t kVertexIntrinsicsIdx = 0;
constexpr size_t kVertexPoseIdx = 1;
constexpr size_t kVertexPointIdx = 2;
class EdgeProjection: public g2o::BaseMultiEdge<util::bal::kPixelDim, Eigen::Vector2d>
{
public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

EdgeProjection(){
    resize(3);
}

void computeError() override{
const VertexIntrinsics* v_k = static_cast<const VertexIntrinsics*> (_vertices[kVertexIntrinsicsIdx]);
const VertexPose* v_pose = static_cast<const VertexPose*> (_vertices[kVertexPoseIdx]);
const VertexPoint* v_point = static_cast<const VertexPoint*> (_vertices[kVertexPointIdx]);

const Eigen::Vector2d pos_pixel = v_k->project(v_pose->estimate() * v_point->estimate());
_error = _measurement - pos_pixel;
}

// TODO: use explicit derivatives instead of numeric derviatives

bool read(std::istream& in) override {return true;}
bool write(std::ostream& out) const override {return true;}
};


}// namespace estimation::bundle_adjustment
