#include "kiss_matcher_slam/core/loop_constraint_processor.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

namespace kiss_matcher_slam {
namespace {

bool isRigidTransform(const Eigen::Matrix4d &transform) {
  if (!transform.allFinite()) return false;
  if (!transform.row(3).isApprox(Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0), 1e-9)) {
    return false;
  }
  const Eigen::Matrix3d rotation = transform.block<3, 3>(0, 0);
  return (rotation.transpose() * rotation).isApprox(Eigen::Matrix3d::Identity(), 1e-6) &&
         std::abs(rotation.determinant() - 1.0) <= 1e-6;
}

gtsam::Pose3 toGtsam(const Eigen::Matrix4d &pose) {
  return {gtsam::Rot3(pose.block<3, 3>(0, 0)),
          gtsam::Point3(pose(0, 3), pose(1, 3), pose(2, 3))};
}

}  // namespace

LoopConstraintProcessor::LoopConstraintProcessor(LoopConstraintConfig config,
                                                 PoseGraphBackend &pose_graph_backend,
                                                 VisualizationState &visualization_state,
                                                 MappingRuntimeState &runtime_state)
    : config_(std::move(config)),
      pose_graph_backend_(pose_graph_backend),
      visualization_state_(visualization_state),
      runtime_state_(runtime_state) {
  if (config_.base_variances.size() != 6 || !std::isfinite(config_.huber_k) ||
      config_.huber_k <= 0.0 || !std::isfinite(config_.minimum_confidence) ||
      config_.minimum_confidence <= 0.0 || config_.minimum_confidence > 1.0) {
    throw std::invalid_argument("invalid loop constraint configuration");
  }
  for (const double variance : config_.base_variances) {
    if (!std::isfinite(variance) || variance <= 0.0) {
      throw std::invalid_argument("loop variances must be positive and finite");
    }
  }
}

AppliedLoopConstraint LoopConstraintProcessor::apply(
    const LoopRegistrationMeasurement &measurement,
    const std::vector<Keyframe> &keyframes) {
  if (measurement.query_id >= keyframes.size() || measurement.match_id >= keyframes.size()) {
    throw std::out_of_range("loop constraint references an unknown keyframe");
  }
  if (measurement.query_id == measurement.match_id) {
    throw std::invalid_argument("loop constraint cannot connect a keyframe to itself");
  }
  if (!isRigidTransform(measurement.query_alignment) ||
      !std::isfinite(measurement.overlap_percent) || measurement.overlap_percent < 0.0 ||
      measurement.overlap_percent > 100.0) {
    throw std::invalid_argument("invalid loop registration measurement");
  }

  const auto &query = keyframes[measurement.query_id];
  const auto &match = keyframes[measurement.match_id];
  const auto visualization = visualization_state_.snapshot();
  if (measurement.query_id >= visualization.corrected_poses.size() ||
      measurement.match_id >= visualization.corrected_poses.size()) {
    throw std::logic_error("loop visualization state is behind the map database");
  }
  const auto aligned_query_pose =
      toGtsam(measurement.query_alignment * query.optimized_pose);
  const auto match_pose = toGtsam(match.optimized_pose);

  auto variances = config_.base_variances;
  if (config_.adaptive_noise) {
    const double confidence = std::clamp(
        measurement.overlap_percent / 100.0, config_.minimum_confidence, 1.0);
    const double scale = 1.0 / (confidence * confidence);
    for (double &variance : variances) variance *= scale;
  }

  pose_graph_backend_.addLoop(measurement.query_id,
                              measurement.match_id,
                              aligned_query_pose.between(match_pose),
                              variances,
                              config_.huber_k);
  visualization_state_.addLoopEdge(measurement.query_id, measurement.match_id);
  runtime_state_.requestGraphRefresh();
  return {measurement.query_id, measurement.match_id, std::move(variances)};
}

}  // namespace kiss_matcher_slam
