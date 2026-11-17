#include <atomic>
#include <limits>
#include <stdexcept>
#include <thread>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "kiss_matcher_slam/core/realtime_pose_tracker.hpp"

namespace {

Eigen::Matrix4d pose(const double x, const double yaw = 0.0) {
  Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
  result.block<3, 3>(0, 0) =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  result(0, 3) = x;
  return result;
}

TEST(RealtimePoseTracker, AccumulatesConsecutiveOdometryMotion) {
  kiss_matcher_slam::RealtimePoseTracker tracker;
  EXPECT_NEAR(tracker.advance(pose(0.0), pose(1.0))(0, 3), 1.0, 1e-9);
  const auto corrected = tracker.advance(pose(1.0), pose(3.0));
  EXPECT_NEAR(corrected(0, 3), 3.0, 1e-9);
  EXPECT_TRUE(tracker.state().odometry_delta.isApprox(pose(3.0), 1e-9));
}

TEST(RealtimePoseTracker, CorrectionCreatesNewAnchorAndResetsDelta) {
  kiss_matcher_slam::RealtimePoseTracker tracker;
  tracker.advance(pose(0.0), pose(2.0));
  tracker.applyCorrection(pose(10.0));

  const auto reset = tracker.state();
  EXPECT_TRUE(reset.odometry_delta.isIdentity(1e-9));
  EXPECT_TRUE(reset.corrected_pose.isApprox(pose(10.0), 1e-9));
  EXPECT_NEAR(tracker.advance(pose(2.0), pose(3.5))(0, 3), 11.5, 1e-9);
}

TEST(RealtimePoseTracker, PreservesFullSe3Composition) {
  kiss_matcher_slam::RealtimePoseTracker tracker;
  constexpr double half_pi = 1.57079632679489661923;
  tracker.applyCorrection(pose(5.0, half_pi));
  const auto corrected = tracker.advance(pose(0.0), pose(1.0));
  EXPECT_NEAR(corrected(0, 3), 5.0, 1e-9);
  EXPECT_NEAR(corrected(1, 3), 1.0, 1e-9);
  EXPECT_NEAR(std::atan2(corrected(1, 0), corrected(0, 0)), half_pi, 1e-9);
}

TEST(RealtimePoseTracker, RejectsInvalidTransformsWithoutChangingState) {
  kiss_matcher_slam::RealtimePoseTracker tracker;
  tracker.applyCorrection(pose(4.0));
  Eigen::Matrix4d invalid = Eigen::Matrix4d::Identity();
  invalid(0, 0) = 2.0;
  EXPECT_THROW(tracker.advance(pose(0.0), invalid), std::invalid_argument);

  invalid = Eigen::Matrix4d::Identity();
  invalid(0, 3) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(tracker.applyCorrection(invalid), std::invalid_argument);
  EXPECT_TRUE(tracker.state().corrected_pose.isApprox(pose(4.0), 1e-9));
}

TEST(RealtimePoseTracker, ConcurrentSnapshotsRemainRigidAndFinite) {
  kiss_matcher_slam::RealtimePoseTracker tracker;
  std::atomic<bool> done{false};
  std::atomic<bool> invalid{false};

  std::thread writer([&] {
    for (int i = 0; i < 100; ++i) {
      tracker.applyCorrection(pose(static_cast<double>(i)));
    }
    done = true;
  });
  std::thread reader([&] {
    while (!done) {
      const auto snapshot = tracker.state();
      if (!snapshot.corrected_pose.allFinite() ||
          !snapshot.odometry_delta.isIdentity(1e-9)) {
        invalid = true;
      }
    }
  });

  writer.join();
  reader.join();
  EXPECT_FALSE(invalid);
}

}  // namespace
