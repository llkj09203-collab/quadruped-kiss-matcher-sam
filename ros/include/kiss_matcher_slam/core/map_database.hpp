#pragma once

#include <cstddef>
#include <shared_mutex>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "kiss_matcher_slam/core/keyframe.hpp"

namespace kiss_matcher_slam {

// Thread-safe in-memory owner of mapping keyframes. IDs are allocated here so
// callers cannot create gaps or duplicate graph node identifiers.
class MapDatabase {
 public:
  using PoseUpdate = std::pair<std::size_t, Eigen::Matrix4d>;

  std::size_t append(Keyframe keyframe);
  bool empty() const;
  std::size_t size() const;

  // Return copies deliberately: references must not outlive the lock or permit
  // unsynchronised mutation of stored point clouds.
  Keyframe at(std::size_t id) const;
  Keyframe latest() const;
  std::vector<Keyframe> snapshot() const;

  // Apply the complete batch or nothing. Invalid input leaves all data intact.
  bool updateOptimizedPoses(const std::vector<PoseUpdate> &updates);
  void clear();

 private:
  mutable std::shared_mutex mutex_;
  std::vector<Keyframe> keyframes_;
};

}  // namespace kiss_matcher_slam
