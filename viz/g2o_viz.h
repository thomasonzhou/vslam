#pragma once

#include <g2o/core/hyper_graph_action.h>

#include <vector>

#include "estimation/bundle_adjustment/g2o_ba.h"

namespace viz {

class PointUpdateAction : public g2o::HyperGraphAction {
 public:
  PointUpdateAction(
      const std::vector<estimation::bundle_adjustment::VertexPoint*>& points,
      std::vector<Eigen::Vector3d>& points_viz, std::mutex& points_mutex)
      : points_(points), points_viz_(points_viz), points_mutex_(points_mutex) {}

  g2o::HyperGraphAction* operator()(
      const g2o::HyperGraph*, g2o::HyperGraphAction::Parameters*) override {
    std::lock_guard lk(points_mutex_);
    for (size_t i = 0; i < points_.size(); ++i) {
      points_viz_[i] = points_[i]->estimate();
    }
    return this;
  }

 private:
  const std::vector<estimation::bundle_adjustment::VertexPoint*>& points_;
  std::vector<Eigen::Vector3d>& points_viz_;
  std::mutex& points_mutex_;
};

}  // namespace viz
