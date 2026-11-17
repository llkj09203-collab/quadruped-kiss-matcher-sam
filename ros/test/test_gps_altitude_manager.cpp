#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "kiss_matcher_slam/core/gps_altitude_manager.hpp"

namespace {

Eigen::Matrix4d pose(const double x, const double y, const double z) {
  Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
  result(0, 3) = x;
  result(1, 3) = y;
  result(2, 3) = z;
  return result;
}

kiss_matcher_slam::GpsAltitudeConfig enabledConfig() {
  kiss_matcher_slam::GpsAltitudeConfig config;
  config.enabled = true;
  config.time_tolerance_s = 0.2;
  config.xy_variance = 1e8;
  config.z_variance = 4.0;
  config.huber_k = 1.345;
  return config;
}

TEST(GpsAltitudeManager, RejectsInvalidConfiguration) {
  auto config = enabledConfig();
  config.z_variance = 0.0;
  EXPECT_THROW((void)kiss_matcher_slam::GpsAltitudeManager{config}, std::invalid_argument);
  config = enabledConfig();
  config.time_tolerance_s = -1.0;
  EXPECT_THROW((void)kiss_matcher_slam::GpsAltitudeManager{config}, std::invalid_argument);
}

TEST(GpsAltitudeManager, DisabledManagerProducesNoConstraint) {
  kiss_matcher_slam::GpsAltitudeConfig config;
  kiss_matcher_slam::GpsAltitudeManager manager(config);
  manager.update(1.0, 100.0);
  EXPECT_FALSE(manager.constraint(1.0, pose(0.0, 0.0, 2.0)).has_value());
}

TEST(GpsAltitudeManager, RejectsMissingAndStaleSamples) {
  kiss_matcher_slam::GpsAltitudeManager manager(enabledConfig());
  EXPECT_FALSE(manager.constraint(1.0, pose(0.0, 0.0, 2.0)).has_value());
  manager.update(1.0, 100.0);
  EXPECT_FALSE(manager.constraint(1.21, pose(0.0, 0.0, 2.0)).has_value());
}

TEST(GpsAltitudeManager, AnchorsFirstAltitudeAndPreservesLocalXY) {
  kiss_matcher_slam::GpsAltitudeManager manager(enabledConfig());
  manager.update(1.0, 100.0);
  const auto first = manager.constraint(1.0, pose(3.0, 4.0, 2.0));
  ASSERT_TRUE(first.has_value());
  EXPECT_TRUE(first->position.isApprox(Eigen::Vector3d(3.0, 4.0, 2.0), 1e-9));
  EXPECT_TRUE(first->variances.isApprox(Eigen::Vector3d(1e8, 1e8, 4.0), 1e-9));

  manager.update(2.0, 101.5);
  const auto second = manager.constraint(2.0, pose(5.0, 6.0, 9.0));
  ASSERT_TRUE(second.has_value());
  EXPECT_TRUE(second->position.isApprox(Eigen::Vector3d(5.0, 6.0, 3.5), 1e-9));
}

TEST(GpsAltitudeManager, IgnoresOutOfOrderSamplesAndRejectsInvalidInput) {
  kiss_matcher_slam::GpsAltitudeManager manager(enabledConfig());
  manager.update(2.0, 102.0);
  manager.update(1.0, 10.0);
  const auto constraint = manager.constraint(2.0, pose(0.0, 0.0, 2.0));
  ASSERT_TRUE(constraint.has_value());
  EXPECT_NEAR(constraint->position.z(), 2.0, 1e-9);
  EXPECT_THROW(manager.update(3.0, std::numeric_limits<double>::quiet_NaN()),
               std::invalid_argument);
}

}  // namespace
