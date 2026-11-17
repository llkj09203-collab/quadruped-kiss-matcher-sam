#pragma once

#include <mutex>

#include <Eigen/Core>

namespace kiss_matcher_slam {

struct RealtimePoseState {
  Eigen::Matrix4d correction_anchor = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d odometry_delta = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d corrected_pose = Eigen::Matrix4d::Identity();
};

// Propagates the latest optimized keyframe pose with frame-to-frame odometry.
// A graph correction establishes a new anchor and resets only the accumulated
// delta; subsequent odometry remains continuous from that anchor.
class RealtimePoseTracker {
 public:
  Eigen::Matrix4d advance(const Eigen::Matrix4d &previous_odometry,
                          const Eigen::Matrix4d &current_odometry);
  void applyCorrection(const Eigen::Matrix4d &corrected_pose);
  RealtimePoseState state() const;

 private:
  static void validatePose(const Eigen::Matrix4d &pose);

  mutable std::mutex mutex_;
  RealtimePoseState state_;
};

}  // namespace kiss_matcher_slam
