#include "kiss_matcher_slam/core/map_database.hpp"

#include <cmath>
#include <mutex>
#include <stdexcept>
#include <utility>

namespace kiss_matcher_slam {

std::size_t MapDatabase::append(Keyframe keyframe) {
  if (!std::isfinite(keyframe.timestamp) || !keyframe.odometry_pose.allFinite() ||
      !keyframe.optimized_pose.allFinite()) {
    throw std::invalid_argument("keyframe contains a non-finite timestamp or pose");
  }

  std::unique_lock<std::shared_mutex> lock(mutex_);
  if (!keyframes_.empty() && keyframe.timestamp < keyframes_.back().timestamp) {
    throw std::invalid_argument("keyframe timestamps must be non-decreasing");
  }

  keyframe.id = keyframes_.size();
  keyframes_.push_back(std::move(keyframe));
  return keyframes_.back().id;
}

bool MapDatabase::empty() const {
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return keyframes_.empty();
}

std::size_t MapDatabase::size() const {
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return keyframes_.size();
}

Keyframe MapDatabase::at(const std::size_t id) const {
  std::shared_lock<std::shared_mutex> lock(mutex_);
  if (id >= keyframes_.size()) throw std::out_of_range("keyframe ID is out of range");
  return keyframes_[id];
}

Keyframe MapDatabase::latest() const {
  std::shared_lock<std::shared_mutex> lock(mutex_);
  if (keyframes_.empty()) throw std::out_of_range("map database is empty");
  return keyframes_.back();
}

std::vector<Keyframe> MapDatabase::snapshot() const {
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return keyframes_;
}

bool MapDatabase::updateOptimizedPoses(const std::vector<PoseUpdate> &updates) {
  std::unique_lock<std::shared_mutex> lock(mutex_);
  for (const auto &[id, pose] : updates) {
    if (id >= keyframes_.size() || !pose.allFinite()) return false;
  }
  for (const auto &[id, pose] : updates) keyframes_[id].optimized_pose = pose;
  return true;
}

void MapDatabase::clear() {
  std::unique_lock<std::shared_mutex> lock(mutex_);
  keyframes_.clear();
}

}  // namespace kiss_matcher_slam
