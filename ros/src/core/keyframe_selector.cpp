#include "kiss_matcher_slam/core/keyframe_selector.hpp"

#include <cmath>
#include <stdexcept>

#include <Eigen/Geometry>

namespace kiss_matcher_slam {

KeyframeSelector::KeyframeSelector(KeyframeSelectorConfig config) : config_(config) {
  if (config_.translation_threshold_m < 0.0 || config_.rotation_threshold_rad < 0.0 ||
      config_.minimum_time_interval_s < 0.0) {
    throw std::invalid_argument("Keyframe thresholds must be non-negative");
  }
}

bool KeyframeSelector::shouldCreate(const Eigen::Matrix4d &current_pose,
                                    double current_timestamp,
                                    const Eigen::Matrix4d &previous_pose,
                                    double previous_timestamp) const {
  if (!current_pose.allFinite() || !previous_pose.allFinite() ||
      !std::isfinite(current_timestamp) || !std::isfinite(previous_timestamp)) {
    return false;
  }
  if (current_timestamp < previous_timestamp ||
      current_timestamp - previous_timestamp < config_.minimum_time_interval_s) {
    return false;
  }

  const double translation =
      (current_pose.block<3, 1>(0, 3) - previous_pose.block<3, 1>(0, 3)).norm();
  const Eigen::Matrix3d relative_rotation =
      previous_pose.block<3, 3>(0, 0).transpose() * current_pose.block<3, 3>(0, 0);
  const double rotation = Eigen::AngleAxisd(relative_rotation).angle();
  return translation > config_.translation_threshold_m ||
         rotation > config_.rotation_threshold_rad;
}

}  // namespace kiss_matcher_slam
