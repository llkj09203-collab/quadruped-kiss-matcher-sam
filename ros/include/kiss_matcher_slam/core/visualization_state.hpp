#pragma once

#include <cstddef>
#include <optional>
#include <shared_mutex>
#include <utility>
#include <vector>

#include <Eigen/Core>

namespace kiss_matcher_slam {

struct VisualizationSnapshot {
  std::vector<Eigen::Matrix4d> odometry_poses;
  std::vector<Eigen::Matrix4d> corrected_poses;
  std::vector<std::pair<std::size_t, std::size_t>> loop_edges;
};

// Thread-safe, ROS-independent source for path and loop-edge visualization.
class VisualizationState {
 public:
  void appendKeyframe(const Eigen::Matrix4d &odometry_pose,
                      const Eigen::Matrix4d &corrected_pose);
  void updateCorrectedPoses(std::vector<Eigen::Matrix4d> corrected_poses);
  void addLoopEdge(std::size_t query_id, std::size_t match_id);

  std::optional<Eigen::Matrix4d> latestCorrectedPose() const;
  VisualizationSnapshot snapshot() const;

 private:
  static void validatePose(const Eigen::Matrix4d &pose);

  mutable std::shared_mutex mutex_;
  VisualizationSnapshot state_;
};

}  // namespace kiss_matcher_slam
