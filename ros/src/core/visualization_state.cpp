#include "kiss_matcher_slam/core/visualization_state.hpp"

#include <mutex>
#include <stdexcept>

namespace kiss_matcher_slam {

void VisualizationState::validatePose(const Eigen::Matrix4d &pose) {
  if (!pose.allFinite()) throw std::invalid_argument("visualization pose must be finite");
}

void VisualizationState::appendKeyframe(const Eigen::Matrix4d &odometry_pose,
                                        const Eigen::Matrix4d &corrected_pose) {
  validatePose(odometry_pose);
  validatePose(corrected_pose);
  std::unique_lock<std::shared_mutex> lock(mutex_);
  state_.odometry_poses.push_back(odometry_pose);
  state_.corrected_poses.push_back(corrected_pose);
}

void VisualizationState::updateCorrectedPoses(
    std::vector<Eigen::Matrix4d> corrected_poses) {
  for (const auto &pose : corrected_poses) validatePose(pose);
  std::unique_lock<std::shared_mutex> lock(mutex_);
  if (corrected_poses.size() > state_.odometry_poses.size()) {
    throw std::invalid_argument("corrected visualization path exceeds odometry path size");
  }
  for (std::size_t i = 0; i < corrected_poses.size(); ++i) {
    state_.corrected_poses[i] = std::move(corrected_poses[i]);
  }
}

void VisualizationState::addLoopEdge(const std::size_t query_id,
                                     const std::size_t match_id) {
  if (query_id == match_id) throw std::invalid_argument("loop visualization edge is self-linked");
  std::unique_lock<std::shared_mutex> lock(mutex_);
  if (query_id >= state_.odometry_poses.size() || match_id >= state_.odometry_poses.size()) {
    throw std::out_of_range("loop visualization edge references an unknown keyframe");
  }
  state_.loop_edges.emplace_back(query_id, match_id);
}

std::optional<Eigen::Matrix4d> VisualizationState::latestCorrectedPose() const {
  std::shared_lock<std::shared_mutex> lock(mutex_);
  if (state_.corrected_poses.empty()) return std::nullopt;
  return state_.corrected_poses.back();
}

VisualizationSnapshot VisualizationState::snapshot() const {
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return state_;
}

}  // namespace kiss_matcher_slam
