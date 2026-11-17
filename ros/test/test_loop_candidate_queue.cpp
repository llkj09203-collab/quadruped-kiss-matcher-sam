#include <gtest/gtest.h>

#include "kiss_matcher_slam/core/loop_candidate_queue.hpp"

TEST(LoopCandidateQueue, PreservesCandidateOrder) {
  kiss_matcher_slam::LoopCandidateQueue queue;
  queue.push({4, 1});
  queue.push({8, 2});
  ASSERT_EQ(queue.size(), 2U);
  EXPECT_EQ(queue.tryPop(), kiss_matcher_slam::LoopCandidate(4, 1));
  EXPECT_EQ(queue.tryPop(), kiss_matcher_slam::LoopCandidate(8, 2));
  EXPECT_FALSE(queue.tryPop().has_value());
}

TEST(LoopCandidateQueue, ClearDropsPendingWork) {
  kiss_matcher_slam::LoopCandidateQueue queue;
  queue.push({4, 1});
  queue.clear();
  EXPECT_EQ(queue.size(), 0U);
}
