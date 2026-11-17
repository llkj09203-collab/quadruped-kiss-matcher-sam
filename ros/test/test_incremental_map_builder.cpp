#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "kiss_matcher_slam/core/incremental_map_builder.hpp"

namespace {

kiss_matcher_slam::Keyframe keyframe(const std::size_t id,
                                     const double x,
                                     const float point_x = 0.0f) {
  kiss_matcher_slam::Keyframe result;
  result.id = id;
  result.optimized_pose(0, 3) = x;
  kiss_matcher_slam::SlamPoint point;
  point.x = point_x;
  result.cloud.push_back(point);
  return result;
}

kiss_matcher_slam::IncrementalMapBuilderConfig config() {
  kiss_matcher_slam::IncrementalMapBuilderConfig result;
  result.scan_voxel_resolution = 0.1;
  result.map_voxel_resolution = 0.1;
  return result;
}

TEST(IncrementalMapBuilder, RejectsInvalidConfiguration) {
  auto invalid = config();
  invalid.map_voxel_resolution = 0.0;
  EXPECT_THROW((void)kiss_matcher_slam::IncrementalMapBuilder{invalid},
               std::invalid_argument);
}

TEST(IncrementalMapBuilder, AddsOnlyNewKeyframesAndReturnsCacheUpdates) {
  kiss_matcher_slam::IncrementalMapBuilder builder(config());
  std::vector<kiss_matcher_slam::Keyframe> keyframes{keyframe(0, 0.0)};
  auto first = builder.build(keyframes, false);
  ASSERT_EQ(first.cloud.size(), 1U);
  ASSERT_EQ(first.visualization_cache_updates.size(), 1U);

  keyframes[0].visualization_cloud = first.visualization_cache_updates[0].second;
  keyframes.push_back(keyframe(1, 2.0));
  auto second = builder.build(keyframes, false);
  EXPECT_EQ(second.cloud.size(), 2U);
  ASSERT_EQ(second.visualization_cache_updates.size(), 1U);
  EXPECT_EQ(second.visualization_cache_updates[0].first, 1U);
  EXPECT_EQ(builder.processedKeyframes(), 2U);
}

TEST(IncrementalMapBuilder, RebuildUsesCorrectedPosesForAllKeyframes) {
  auto builder_config = config();
  builder_config.keyframe_clouds_are_voxelized = true;
  kiss_matcher_slam::IncrementalMapBuilder builder(builder_config);
  std::vector<kiss_matcher_slam::Keyframe> keyframes{
      keyframe(0, 0.0), keyframe(1, 2.0)};
  builder.build(keyframes, false);

  keyframes[0].optimized_pose(0, 3) = 10.0;
  keyframes[1].optimized_pose(0, 3) = 12.0;
  const auto rebuilt = builder.build(keyframes, true);
  ASSERT_EQ(rebuilt.cloud.size(), 2U);
  EXPECT_NEAR(rebuilt.cloud.front().x, 10.0, 1e-6);
  EXPECT_NEAR(rebuilt.cloud.back().x, 12.0, 1e-6);
}

TEST(IncrementalMapBuilder, RejectsNonContiguousAndUnexpectedlyShrinkingInput) {
  kiss_matcher_slam::IncrementalMapBuilder builder(config());
  EXPECT_THROW(builder.build({keyframe(1, 0.0)}, false), std::invalid_argument);
  builder.build({keyframe(0, 0.0), keyframe(1, 1.0)}, false);
  EXPECT_THROW(builder.build({keyframe(0, 0.0)}, false), std::invalid_argument);
  EXPECT_NO_THROW(builder.build({keyframe(0, 0.0)}, true));
}

}  // namespace
