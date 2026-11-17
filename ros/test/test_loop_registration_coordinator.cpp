#include <gtest/gtest.h>

#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "kiss_matcher_slam/core/loop_constraint_processor.hpp"
#include "slam/loop_registration_coordinator.hpp"

namespace {

kiss_matcher::LoopClosureConfig registrationConfig() {
  kiss_matcher::LoopClosureConfig config;
  config.enable_global_registration_ = false;
  config.num_submap_keyframes_ = 1;
  config.voxel_res_ = 0.3;
  config.matcher_config_ = kiss_matcher::KISSMatcherConfig(config.voxel_res_, false);
  return config;
}

kiss_matcher_slam::Keyframe makeFrame(const std::size_t id, const double x) {
  kiss_matcher_slam::Keyframe frame;
  frame.id = id;
  frame.timestamp = static_cast<double>(id);
  frame.odometry_pose(0, 3) = x;
  frame.optimized_pose = frame.odometry_pose;
  return frame;
}

struct Fixture {
  std::vector<double> variances = std::vector<double>(6, 1e-3);
  kiss_matcher_slam::LoopCandidateQueue queue;
  kiss_matcher_slam::MapDatabase database;
  kiss_matcher_slam::PoseGraphBackend backend;
  kiss_matcher_slam::VisualizationState visualization;
  kiss_matcher_slam::MappingRuntimeState runtime;
  kiss_matcher::LoopClosure closure{registrationConfig(), rclcpp::get_logger("loop_test")};
  kiss_matcher_slam::LoopConstraintProcessor constraint_processor{
      {variances, 1.345, true, 0.1}, backend, visualization, runtime};
  kiss_matcher_slam::LoopRegistrationCoordinator coordinator{
      queue, database, closure, constraint_processor};

  Fixture() {
    const auto first = makeFrame(0, 0.0);
    const auto second = makeFrame(1, 1.0);
    database.append(first);
    database.append(second);
    visualization.appendKeyframe(first.odometry_pose, first.optimized_pose);
    visualization.appendKeyframe(second.odometry_pose, second.optimized_pose);
    backend.addPrior(0, gtsam::Pose3(), variances);
    backend.addOdometry(0,
                        1,
                        gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1.0, 0.0, 0.0)),
                        gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1.0, 0.0, 0.0)),
                        variances);
    backend.optimize();
  }
};

TEST(LoopRegistrationCoordinator, ReportsNoWorkForEmptyQueue) {
  Fixture fixture;
  const auto result = fixture.coordinator.processNext();
  EXPECT_EQ(result.status, kiss_matcher_slam::LoopRegistrationStatus::kNoWork);
}

TEST(LoopRegistrationCoordinator, RejectsAndConsumesInvalidCandidate) {
  Fixture fixture;
  fixture.queue.push({7, 0});
  const auto result = fixture.coordinator.processNext();
  EXPECT_EQ(result.status, kiss_matcher_slam::LoopRegistrationStatus::kInvalidCandidate);
  EXPECT_EQ(fixture.queue.size(), 0U);
}

TEST(LoopRegistrationCoordinator, ClassifiesEmptyCloudRegistrationFailure) {
  Fixture fixture;
  fixture.queue.push({1, 0});
  const auto result = fixture.coordinator.processNext();
  EXPECT_EQ(result.status, kiss_matcher_slam::LoopRegistrationStatus::kRejectedMatcher);
  EXPECT_DOUBLE_EQ(result.overlap_percent, 0.0);
  EXPECT_GE(result.elapsed_ms, 0.0);
  EXPECT_FALSE(fixture.coordinator.consumeDebugClouds().has_value());
  EXPECT_TRUE(fixture.visualization.snapshot().loop_edges.empty());
}

}  // namespace
