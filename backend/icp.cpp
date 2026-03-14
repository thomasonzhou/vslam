#include "backend/icp.h"

#include <Eigen/SVD>

namespace backend {

    void point_to_point_svd(
        const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &points3d_cam1,
        const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &points3d_cam2,
        Sophus::SE3d &c2_T_c1) {
    Eigen::Vector3d centroid1 = Eigen::Vector3d::Zero();
    Eigen::Vector3d centroid2 = Eigen::Vector3d::Zero();
    const size_t points = std::min(points3d_cam1.size(), points3d_cam2.size());

    for (size_t i = 0; i < points; ++i) {
        centroid1 += points3d_cam1[i];
        centroid2 += points3d_cam2[i];
    }
    centroid1 /= static_cast<double>(points);
    centroid2 /= static_cast<double>(points);

    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for (size_t i = 0; i < points; ++i) {
        const Eigen::Vector3d q1 = points3d_cam1[i] - centroid1;
        const Eigen::Vector3d q2 = points3d_cam2[i] - centroid2;
        W += q2 * q1.transpose();
    }

    const Eigen::JacobiSVD<Eigen::Matrix3d> svd(
        W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Eigen::Matrix3d U = svd.matrixU();
    const Eigen::Matrix3d V = svd.matrixV();

    Eigen::Matrix3d D = Eigen::Matrix3d::Identity();
    if ((U * V.transpose()).determinant() < 0.0) {
        D(2, 2) = -1.0;
    }

    Eigen::Matrix3d R = U * D * V.transpose();
    c2_T_c1.setRotationMatrix(R);

    Eigen::Vector3d t = centroid1 - R * centroid2;
    c2_T_c1.translation() = t;
    }

} // namespace backend
