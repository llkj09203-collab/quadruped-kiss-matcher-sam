#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Core>

#include "kiss_matcher_slam/core/keyframe.hpp"
#include "kiss_matcher_slam/core/mapping_runtime_state.hpp"
#include "kiss_matcher_slam/core/pose_graph_backend.hpp"
#include "kiss_matcher_slam/core/visualization_state.hpp"

namespace kiss_matcher_slam {

struct LoopConstraintConfig {
  std::vector<double> base_variances;
  double huber_k = 1.345;
  bool adaptive_noise = true;
  double minimum_confidence = 0.1;
};

struct LoopRegistrationMeasurement {
  std::size_t query_id = 0;
  std::size_t match_id = 0;
  Eigen::Matrix4d query_alignment = Eigen::Matrix4d::Identity();
  double overlap_percent = 0.0;
};

struct AppliedLoopConstraint {
  std::size_t query_id = 0;
  std::size_t match_id = 0;
  std::vector<double> variances;
};

class LoopConstraintProcessor {
 public:
  LoopConstraintProcessor(LoopConstraintConfig config,
                          PoseGraphBackend &pose_graph_backend,
                          VisualizationState &visualization_state,
                          MappingRuntimeState &runtime_state);

  AppliedLoopConstraint apply(const LoopRegistrationMeasurement &measurement,
                              const std::vector<Keyframe> &keyframes);

 private:
  LoopConstraintConfig config_;
  PoseGraphBackend &pose_graph_backend_;
  VisualizationState &visualization_state_;
  MappingRuntimeState &runtime_state_;
};

}  // namespace kiss_matcher_slam
