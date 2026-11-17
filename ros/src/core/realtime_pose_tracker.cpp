#include "kiss_matcher_slam/core/realtime_pose_tracker.hpp"

#include <cmath>
#include <stdexcept>

#include <Eigen/LU>

namespace kiss_matcher_slam {

void RealtimePoseTracker::validatePose(const Eigen::Matrix4d &pose) {
  if (!pose.allFinite()) throw std::invalid_argument("pose must contain only finite values");
  constexpr double tolerance = 1e-5;
  const Eigen::RowVector4d expected_last_row(0.0, 0.0, 0.0, 1.0);
  const Eigen::Matrix3d rotation = pose.block<3, 3>(0, 0);
  if (!pose.row(3).isApprox(expected_last_row, tolerance) ||
      !rotation.transpose().isApprox(rotation.inverse(), tolerance) ||
      std::abs(rotation.determinant() - 1.0) > tolerance) {
    throw std::invalid_argument("pose must be a rigid SE(3) transform");
  }
}

Eigen::Matrix4d RealtimePoseTracker::advance(const Eigen::Matrix4d &previous_odometry,
                                             const Eigen::Matrix4d &current_odometry) {
  validatePose(previous_odometry);
  validatePose(current_odometry);
  std::lock_guard<std::mutex> lock(mutex_);
  state_.odometry_delta =
      state_.odometry_delta * previous_odometry.inverse() * current_odometry;
  state_.corrected_pose = state_.correction_anchor * state_.odometry_delta;
  return state_.corrected_pose;
}

void RealtimePoseTracker::applyCorrection(const Eigen::Matrix4d &corrected_pose) {
  validatePose(corrected_pose);
  std::lock_guard<std::mutex> lock(mutex_);
  state_.correction_anchor = corrected_pose;
  state_.odometry_delta = Eigen::Matrix4d::Identity();
  state_.corrected_pose = corrected_pose;
}

RealtimePoseState RealtimePoseTracker::state() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

}  // namespace kiss_matcher_slam
