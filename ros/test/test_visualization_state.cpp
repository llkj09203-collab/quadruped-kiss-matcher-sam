#include <atomic>
#include <limits>
#include <stdexcept>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "kiss_matcher_slam/core/visualization_state.hpp"

namespace {

Eigen::Matrix4d pose(const double x) {
  Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
  result(0, 3) = x;
  return result;
}

TEST(VisualizationState, AppendsPathsAndReturnsIndependentSnapshot) {
  kiss_matcher_slam::VisualizationState state;
  state.appendKeyframe(pose(1.0), pose(2.0));
  auto snapshot = state.snapshot();
  ASSERT_EQ(snapshot.odometry_poses.size(), 1U);
  ASSERT_EQ(snapshot.corrected_poses.size(), 1U);
  snapshot.corrected_poses.front()(0, 3) = 100.0;
  EXPECT_NEAR(state.latestCorrectedPose()->operator()(0, 3), 2.0, 1e-9);
}

TEST(VisualizationState, UpdatesAvailableCorrectedPathPrefix) {
  kiss_matcher_slam::VisualizationState state;
  state.appendKeyframe(pose(1.0), pose(1.0));
  state.appendKeyframe(pose(2.0), pose(2.0));
  state.updateCorrectedPoses({pose(10.0)});
  EXPECT_NEAR(state.snapshot().corrected_poses[1](0, 3), 2.0, 1e-9);
  state.updateCorrectedPoses({pose(10.0), pose(20.0)});
  EXPECT_NEAR(state.latestCorrectedPose()->operator()(0, 3), 20.0, 1e-9);
  EXPECT_THROW(state.updateCorrectedPoses({pose(1.0), pose(2.0), pose(3.0)}),
               std::invalid_argument);
}

TEST(VisualizationState, ValidatesLoopEdges) {
  kiss_matcher_slam::VisualizationState state;
  state.appendKeyframe(pose(0.0), pose(0.0));
  state.appendKeyframe(pose(1.0), pose(1.0));
  state.addLoopEdge(1, 0);
  EXPECT_EQ(state.snapshot().loop_edges.size(), 1U);
  EXPECT_THROW(state.addLoopEdge(0, 0), std::invalid_argument);
  EXPECT_THROW(state.addLoopEdge(2, 0), std::out_of_range);
}

TEST(VisualizationState, RejectsNonFinitePose) {
  kiss_matcher_slam::VisualizationState state;
  Eigen::Matrix4d invalid = Eigen::Matrix4d::Identity();
  invalid(0, 0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(state.appendKeyframe(invalid, pose(0.0)), std::invalid_argument);
  EXPECT_TRUE(state.snapshot().odometry_poses.empty());
}

TEST(VisualizationState, ConcurrentSnapshotsRemainConsistent) {
  kiss_matcher_slam::VisualizationState state;
  std::atomic<bool> done{false};
  std::atomic<bool> inconsistent{false};
  std::thread writer([&] {
    for (int i = 0; i < 100; ++i) state.appendKeyframe(pose(i), pose(i));
    done = true;
  });
  std::thread reader([&] {
    while (!done) {
      const auto snapshot = state.snapshot();
      if (snapshot.odometry_poses.size() != snapshot.corrected_poses.size()) {
        inconsistent = true;
      }
    }
  });
  writer.join();
  reader.join();
  EXPECT_FALSE(inconsistent);
  EXPECT_EQ(state.snapshot().odometry_poses.size(), 100U);
}

}  // namespace
