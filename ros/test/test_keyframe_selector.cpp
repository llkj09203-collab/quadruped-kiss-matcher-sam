#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

#include <Eigen/Geometry>

#include "kiss_matcher_slam/core/keyframe_selector.hpp"

namespace {

using kiss_matcher_slam::KeyframeSelector;
using kiss_matcher_slam::KeyframeSelectorConfig;

TEST(KeyframeSelectorTest, RejectsMotionBelowAllThresholds) {
  KeyframeSelector selector({1.0, 0.2, 0.0});
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  pose(0, 3) = 0.5;
  EXPECT_FALSE(selector.shouldCreate(pose, 1.0, Eigen::Matrix4d::Identity(), 0.0));
}

TEST(KeyframeSelectorTest, AcceptsTranslation) {
  KeyframeSelector selector({1.0, 0.2, 0.0});
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  pose(0, 3) = 1.01;
  EXPECT_TRUE(selector.shouldCreate(pose, 1.0, Eigen::Matrix4d::Identity(), 0.0));
}

TEST(KeyframeSelectorTest, AcceptsRotationWithoutTranslation) {
  KeyframeSelector selector({1.0, 0.2, 0.0});
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  pose.block<3, 3>(0, 0) =
      Eigen::AngleAxisd(0.21, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  EXPECT_TRUE(selector.shouldCreate(pose, 1.0, Eigen::Matrix4d::Identity(), 0.0));
}

TEST(KeyframeSelectorTest, EnforcesMinimumTimeInterval) {
  KeyframeSelector selector({1.0, 0.2, 2.0});
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  pose(0, 3) = 10.0;
  EXPECT_FALSE(selector.shouldCreate(pose, 1.9, Eigen::Matrix4d::Identity(), 0.0));
  EXPECT_TRUE(selector.shouldCreate(pose, 2.0, Eigen::Matrix4d::Identity(), 0.0));
}

TEST(KeyframeSelectorTest, RejectsInvalidAndOutOfOrderInput) {
  KeyframeSelector selector;
  Eigen::Matrix4d invalid = Eigen::Matrix4d::Identity();
  invalid(0, 0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(selector.shouldCreate(invalid, 1.0, Eigen::Matrix4d::Identity(), 0.0));
  EXPECT_FALSE(selector.shouldCreate(
      Eigen::Matrix4d::Identity(), 0.0, Eigen::Matrix4d::Identity(), 1.0));
}

TEST(KeyframeSelectorTest, RejectsNegativeConfiguration) {
  EXPECT_THROW(KeyframeSelector(KeyframeSelectorConfig{-1.0, 0.2, 0.0}),
               std::invalid_argument);
}

}  // namespace
