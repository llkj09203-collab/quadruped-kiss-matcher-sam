#include <gtest/gtest.h>

#include "kiss_matcher_slam/core/mapping_runtime_state.hpp"

TEST(MappingRuntimeState, OwnsSessionKeyframeSequence) {
  kiss_matcher_slam::MappingRuntimeState state;
  EXPECT_FALSE(state.initialized());
  EXPECT_EQ(state.commitKeyframe(), 0U);
  EXPECT_TRUE(state.initialized());
  EXPECT_EQ(state.commitKeyframe(), 1U);
  EXPECT_EQ(state.nextKeyframeId(), 2U);

  state.reset(7);
  EXPECT_TRUE(state.initialized());
  EXPECT_EQ(state.commitKeyframe(), 7U);
}

TEST(MappingRuntimeState, RefreshRequestsAreConsumedExactlyOnce) {
  kiss_matcher_slam::MappingRuntimeState state;
  state.requestMapRebuild();
  state.requestGraphRefresh();
  EXPECT_TRUE(state.consumeMapRebuildRequest());
  EXPECT_FALSE(state.consumeMapRebuildRequest());
  EXPECT_TRUE(state.consumeGraphRefreshRequest());
  EXPECT_FALSE(state.consumeGraphRefreshRequest());

  state.requestMapRebuild();
  state.reset();
  EXPECT_FALSE(state.consumeMapRebuildRequest());
}
