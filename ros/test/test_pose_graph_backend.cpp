#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "kiss_matcher_slam/core/pose_graph_backend.hpp"

namespace {

using kiss_matcher_slam::PoseGraphBackend;
using kiss_matcher_slam::PoseGraphBackendConfig;

const std::vector<double> kPoseVariances{1e-4, 1e-4, 1e-4, 1e-3, 1e-3, 1e-3};

gtsam::Pose3 translated(const double x, const double y = 0.0, const double z = 0.0) {
  return {gtsam::Rot3(), gtsam::Point3(x, y, z)};
}

TEST(PoseGraphBackend, RejectsInvalidConfiguration) {
  EXPECT_THROW(PoseGraphBackend(PoseGraphBackendConfig{0.0, 1, 3}),
               std::invalid_argument);
  EXPECT_THROW(PoseGraphBackend(PoseGraphBackendConfig{0.01, 0, 3}),
               std::invalid_argument);
  EXPECT_THROW(PoseGraphBackend(PoseGraphBackendConfig{0.01, 1, -1}),
               std::invalid_argument);
}

TEST(PoseGraphBackend, RejectsInvalidConstraintNoise) {
  PoseGraphBackend backend;
  EXPECT_THROW(backend.addPrior(0, gtsam::Pose3(), {1.0}), std::invalid_argument);

  auto invalid = kPoseVariances;
  invalid[2] = 0.0;
  EXPECT_THROW(backend.addPrior(0, gtsam::Pose3(), invalid), std::invalid_argument);

  invalid = kPoseVariances;
  invalid[4] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(backend.addPrior(0, gtsam::Pose3(), invalid), std::invalid_argument);

  gtsam::Vector3 gps_variances(1.0, -1.0, 1.0);
  EXPECT_THROW(backend.addGps(0, gtsam::Point3(), gps_variances, 1.0),
               std::invalid_argument);
  EXPECT_THROW(backend.addLoop(0, 1, gtsam::Pose3(), kPoseVariances, 0.0),
               std::invalid_argument);
}

TEST(PoseGraphBackend, OptimizesPriorAndOdometryBatch) {
  PoseGraphBackend backend;
  backend.addPrior(0, gtsam::Pose3(), kPoseVariances);
  backend.addOdometry(0, 1, translated(1.0), translated(1.1), kPoseVariances);

  const auto result = backend.optimize();
  ASSERT_EQ(result.estimates.size(), 2U);
  EXPECT_FALSE(result.loop_constraints_applied);
  EXPECT_NEAR(result.estimates.at<gtsam::Pose3>(0).x(), 0.0, 1e-6);
  EXPECT_NEAR(result.estimates.at<gtsam::Pose3>(1).x(), 1.0, 1e-6);
  EXPECT_EQ(backend.estimates().size(), 2U);
}

TEST(PoseGraphBackend, RejectsDuplicateNodes) {
  PoseGraphBackend backend;
  backend.addPrior(0, gtsam::Pose3(), kPoseVariances);
  EXPECT_THROW(backend.addPrior(0, gtsam::Pose3(), kPoseVariances),
               std::invalid_argument);
  backend.optimize();
  EXPECT_THROW(backend.addPrior(0, gtsam::Pose3(), kPoseVariances),
               std::invalid_argument);

  backend.addOdometry(0, 1, translated(1.0), translated(1.0), kPoseVariances);
  EXPECT_THROW(backend.addOdometry(0, 1, translated(1.0), translated(1.0),
                                   kPoseVariances),
               std::invalid_argument);
}

TEST(PoseGraphBackend, ReportsLoopOnlyForBatchThatAppliesIt) {
  PoseGraphBackend backend;
  backend.addPrior(0, gtsam::Pose3(), kPoseVariances);
  backend.addOdometry(0, 1, translated(1.0), translated(1.0), kPoseVariances);
  backend.optimize();

  backend.addLoop(1, 0, translated(-1.0), kPoseVariances, 1.345);
  const auto loop_result = backend.optimize();
  EXPECT_TRUE(loop_result.loop_constraints_applied);

  backend.addOdometry(1, 2, translated(1.0), translated(2.0), kPoseVariances);
  const auto odometry_result = backend.optimize();
  EXPECT_FALSE(odometry_result.loop_constraints_applied);
  EXPECT_EQ(odometry_result.estimates.size(), 3U);
}

TEST(PoseGraphBackend, AcceptsRobustGpsAltitudeConstraint) {
  PoseGraphBackend backend;
  backend.addPrior(0, gtsam::Pose3(), kPoseVariances);
  backend.addGps(0, gtsam::Point3(0.0, 0.0, 2.0), gtsam::Vector3(100.0, 100.0, 0.01),
                 1.345);
  const auto result = backend.optimize();
  const double optimized_height = result.estimates.at<gtsam::Pose3>(0).z();
  EXPECT_TRUE(std::isfinite(optimized_height));
  EXPECT_GT(optimized_height, 0.0);
  EXPECT_LT(optimized_height, 2.0);
}

}  // namespace
