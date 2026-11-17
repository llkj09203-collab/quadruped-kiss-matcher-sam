#pragma once

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace kiss_matcher_slam {

struct PlaceCandidate {
  std::size_t keyframe_id = std::numeric_limits<std::size_t>::max();
  double descriptor_distance = std::numeric_limits<double>::infinity();
  double yaw_offset_rad = 0.0;
};

struct RegistrationResult {
  bool success = false;
  bool converged = false;
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  double overlap_percent = 0.0;
  double fitness = std::numeric_limits<double>::infinity();
  std::size_t inlier_count = 0;
  std::string failure_reason;
};

enum class TrackingState {
  kUninitialized,
  kLoadingMap,
  kRelocalizing,
  kTracking,
  kDegraded,
  kLost,
  kError,
};

struct LocalizationResult {
  bool success = false;
  TrackingState state = TrackingState::kUninitialized;
  Eigen::Matrix4d map_from_base = Eigen::Matrix4d::Identity();
  double confidence = 0.0;
  std::vector<PlaceCandidate> candidates;
  RegistrationResult registration;
  std::string message;
};

}  // namespace kiss_matcher_slam
