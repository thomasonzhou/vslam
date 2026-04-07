#pragma once
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <vector>

// helpers to parse bundle adjustment in the large files
// https://grail.cs.washington.edu/projects/bal/

namespace util::bal {
constexpr int kPixelDim = 2;
constexpr int kRotationVecDim = 3;
// constexpr int kQuaternionDim = kRotationVecDim + 1;
constexpr int kRotationDim = kRotationVecDim;
constexpr int kTranslationDim = 3;
constexpr int kIntrinsicsDim = 3;
constexpr int kPoseDim = kTranslationDim + kRotationDim;
constexpr int kBALCameraBlockSize =
    kRotationDim + kTranslationDim + kIntrinsicsDim;

constexpr int kFocalIdx = kPoseDim + 0;
constexpr int kK1Idx = kPoseDim + 1;
constexpr int kK2Idx = kPoseDim + 2;
}  // namespace util::bal

namespace util {

class BALProblem {
 public:
  explicit BALProblem(const std::filesystem::path& path) {
    std::ifstream in(path);
    in >> num_cameras >> num_points >> num_observations;

    camera_idx.resize(num_observations);
    point_idx.resize(num_observations);
    observations.resize(num_observations * bal::kPixelDim);
    camera_params.resize(num_cameras * bal::kBALCameraBlockSize);
    point_positions.resize(num_points * bal::kTranslationDim);

    for (int i = 0; i < num_observations; ++i) {
      in >> camera_idx[i] >> point_idx[i] >> observations[i * 2] >>
          observations[i * 2 + 1];
    }

    for (int cam = 0; cam < num_cameras; ++cam) {
      const std::size_t cam_start = cam_block_idx(cam);
      for (int i = 0; i < bal::kRotationDim; ++i) {
        in >> camera_params[cam_start + i];
      }
      // quaternion logic, requires manifolds so we will approach this later
      // // always convert rotation vector to quaternion
      // Eigen::Vector3d vec;
      // for (int i = 0; i < kRotationVecDim; ++i){
      //     in >> vec[i];
      // }
      // const Eigen::Vector<double, 4> quat =
      // geometry::angle_axis_to_quat(vec); for (int i = 0; i < kQuaternionDim;
      // ++i){
      //     camera_params[cam_start + i] = quat[i];
      // }

      for (int i = 0; i < bal::kTranslationDim; ++i) {
        in >> camera_params[cam_start + bal::kRotationDim + i];
      }
      for (int i = 0; i < bal::kIntrinsicsDim; ++i) {
        in >> camera_params[cam_start + bal::kRotationDim +
                            bal::kTranslationDim + i];
      }
    }
    for (int point = 0; point < num_points; ++point) {
      const std::size_t point_start = point_block_idx(point);
      for (int i = 0; i < bal::kTranslationDim; ++i) {
        in >> point_positions[point_start + i];
      }
    }
  }

  const double* observation_data(std::size_t idx) const noexcept {
    return observations.data() + idx * bal::kPixelDim;
  }

  double* mutable_camera_data(std::size_t cam) noexcept {
    return camera_params.data() + cam_block_idx(cam);
  }

  const double* camera_data(std::size_t cam) const noexcept {
    return camera_params.data() + cam_block_idx(cam);
  }

  double* mutable_points(std::size_t point) noexcept {
    return point_positions.data() + point_block_idx(point);
  }

  const double* points(std::size_t point) const noexcept {
    return point_positions.data() + point_block_idx(point);
  }

  std::size_t num_observations;
  std::size_t num_cameras;
  std::size_t num_points;

  std::vector<int> camera_idx;
  std::vector<int> point_idx;

 private:
  size_t cam_block_idx(const size_t cam) const noexcept {
    return bal::kBALCameraBlockSize * cam;
  }

  size_t point_block_idx(const size_t point) const noexcept {
    return bal::kTranslationDim * point;
  }

  std::vector<double> observations;
  std::vector<double> camera_params;
  std::vector<double> point_positions;
};

}  // namespace util