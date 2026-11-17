#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>
#include <vector>

#include <gtsam/geometry/Pose3.h>

#include "kiss_matcher_slam/core/loop_constraint_processor.hpp"

namespace {

constexpr double kVariance = 1e-3;

kiss_matcher_slam::Keyframe keyframe(const std::size_t id, const double x) {
  kiss_matcher_slam::Keyframe frame;
  frame.id = id;
  frame.timestamp = static_cast<double>(id);
  frame.odometry_pose(0, 3) = x;
  frame.optimized_pose = frame.odometry_pose;
  return frame;
}

struct Fixture {
  kiss_matcher_slam::PoseGraphBackend backend;
  kiss_matcher_slam::VisualizationState visualization;
  kiss_matcher_slam::MappingRuntimeState runtime;
  std::vector<kiss_matcher_slam::Keyframe> keyframes{keyframe(0, 0.0), keyframe(1, 2.0)};
  std::vector<double> variances = std::vector<double>(6, kVariance);

  Fixture() {
    visualization.appendKeyframe(keyframes[0].odometry_pose, keyframes[0].optimized_pose);
    visualization.appendKeyframe(keyframes[1].odometry_pose, keyframes[1].optimized_pose);
    backend.addPrior(0, gtsam::Pose3(), variances);
    backend.addOdometry(0,
                        1,
                        gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2.0, 0.0, 0.0)),
                        gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2.0, 0.0, 0.0)),
                        variances);
    backend.optimize();
  }

  kiss_matcher_slam::LoopConstraintProcessor makeProcessor(bool adaptive = true) {
    return {{variances, 1.345, adaptive, 0.1}, backend, visualization, runtime};
  }
};

TEST(LoopConstraintProcessor, AddsAdaptiveConstraintAndUpdatesState) {
  Fixture fixture;
  auto processor = fixture.makeProcessor();
  const auto result = processor.apply({1, 0, Eigen::Matrix4d::Identity(), 50.0},
                                      fixture.keyframes);

  ASSERT_EQ(result.variances.size(), 6U);
  EXPECT_NEAR(result.variances.front(), 4.0 * kVariance, 1e-12);
  EXPECT_TRUE(fixture.runtime.consumeGraphRefreshRequest());
  const auto snapshot = fixture.visualization.snapshot();
  ASSERT_EQ(snapshot.loop_edges.size(), 1U);
  const std::pair<std::size_t, std::size_t> expected_edge{1, 0};
  EXPECT_EQ(snapshot.loop_edges.front(), expected_edge);
  EXPECT_TRUE(fixture.backend.optimize().loop_constraints_applied);
}

TEST(LoopConstraintProcessor, CanUseFixedNoise) {
  Fixture fixture;
  auto processor = fixture.makeProcessor(false);
  const auto result = processor.apply({1, 0, Eigen::Matrix4d::Identity(), 25.0},
                                      fixture.keyframes);
  EXPECT_NEAR(result.variances.front(), kVariance, 1e-12);
}

TEST(LoopConstraintProcessor, RejectsInvalidMeasurementsWithoutSideEffects) {
  Fixture fixture;
  auto processor = fixture.makeProcessor();
  EXPECT_THROW(processor.apply({2, 0, Eigen::Matrix4d::Identity(), 50.0}, fixture.keyframes),
               std::out_of_range);
  EXPECT_THROW(processor.apply({1, 1, Eigen::Matrix4d::Identity(), 50.0}, fixture.keyframes),
               std::invalid_argument);
  Eigen::Matrix4d invalid_transform = Eigen::Matrix4d::Identity();
  invalid_transform(0, 0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(processor.apply({1, 0, invalid_transform, 50.0}, fixture.keyframes),
               std::invalid_argument);
  EXPECT_FALSE(fixture.runtime.consumeGraphRefreshRequest());
  EXPECT_TRUE(fixture.visualization.snapshot().loop_edges.empty());
}

TEST(LoopConstraintProcessor, RejectsInvalidConfiguration) {
  Fixture fixture;
  auto invalid_variances = fixture.variances;
  invalid_variances[0] = 0.0;
  EXPECT_THROW(kiss_matcher_slam::LoopConstraintProcessor(
                   {invalid_variances, 1.345, true, 0.1},
                   fixture.backend,
                   fixture.visualization,
                   fixture.runtime),
               std::invalid_argument);
}

}  // namespace
