#pragma once

#include <g2o/core/hyper_graph_action.h>
#include <memory>
#include <vector>

namespace viz {

template<typename VertexType, typename PointReader>
class PointUpdateAction : public g2o::HyperGraphAction {
 public:
  PointUpdateAction(
      const std::vector<VertexType*>& points,
      std::vector<Eigen::Vector3d>& points_viz, std::mutex& points_mutex, PointReader get_point_fn)
      : points_(points), points_viz_(points_viz), points_mutex_(points_mutex), get_point_fn_(get_point_fn) {}

  g2o::HyperGraphAction* operator()(
      const g2o::HyperGraph*, g2o::HyperGraphAction::Parameters*) override {
    std::lock_guard<std::mutex> lk(points_mutex_);
    for (size_t i = 0; i < points_.size(); ++i) {
      points_viz_[i] = get_point_fn_(points_[i]);
    }
    return this;
  }

 private:
  const std::vector<VertexType*>& points_;
  std::vector<Eigen::Vector3d>& points_viz_;
  std::mutex& points_mutex_;
  PointReader get_point_fn_;
};

}  // namespace viz
