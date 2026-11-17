#include <atomic>
#include <limits>
#include <stdexcept>
#include <thread>

#include <gtest/gtest.h>

#include "kiss_matcher_slam/core/map_database.hpp"

namespace {

kiss_matcher_slam::Keyframe makeKeyframe(const double timestamp, const double x = 0.0) {
  kiss_matcher_slam::Keyframe keyframe;
  keyframe.timestamp = timestamp;
  keyframe.odometry_pose(0, 3) = x;
  keyframe.optimized_pose(0, 3) = x;
  return keyframe;
}

TEST(MapDatabase, AllocatesContiguousIdsAndReturnsCopies) {
  kiss_matcher_slam::MapDatabase database;
  auto first = makeKeyframe(1.0);
  first.id = 99;

  EXPECT_EQ(database.append(first), 0U);
  EXPECT_EQ(database.append(makeKeyframe(2.0)), 1U);
  EXPECT_EQ(database.size(), 2U);
  EXPECT_EQ(database.latest().id, 1U);

  auto copy = database.at(0);
  copy.timestamp = 42.0;
  EXPECT_DOUBLE_EQ(database.at(0).timestamp, 1.0);
}

TEST(MapDatabase, RejectsInvalidAndOutOfOrderInput) {
  kiss_matcher_slam::MapDatabase database;
  database.append(makeKeyframe(2.0));

  EXPECT_THROW(database.append(makeKeyframe(1.0)), std::invalid_argument);
  EXPECT_THROW(database.append(makeKeyframe(std::numeric_limits<double>::quiet_NaN())),
               std::invalid_argument);
  EXPECT_EQ(database.size(), 1U);
}

TEST(MapDatabase, EmptyAccessThrowsAndClearResetsIds) {
  kiss_matcher_slam::MapDatabase database;
  EXPECT_TRUE(database.empty());
  EXPECT_THROW(database.latest(), std::out_of_range);
  EXPECT_THROW(database.at(0), std::out_of_range);

  database.append(makeKeyframe(1.0));
  database.clear();
  EXPECT_TRUE(database.empty());
  EXPECT_EQ(database.append(makeKeyframe(3.0)), 0U);
}

TEST(MapDatabase, PoseBatchIsAtomic) {
  kiss_matcher_slam::MapDatabase database;
  database.append(makeKeyframe(1.0));
  database.append(makeKeyframe(2.0));

  Eigen::Matrix4d updated = Eigen::Matrix4d::Identity();
  updated(0, 3) = 10.0;
  EXPECT_FALSE(database.updateOptimizedPoses({{0, updated}, {3, updated}}));
  EXPECT_DOUBLE_EQ(database.at(0).optimized_pose(0, 3), 0.0);

  EXPECT_TRUE(database.updateOptimizedPoses({{0, updated}, {1, updated}}));
  EXPECT_DOUBLE_EQ(database.at(0).optimized_pose(0, 3), 10.0);
  EXPECT_DOUBLE_EQ(database.at(1).optimized_pose(0, 3), 10.0);
}

TEST(MapDatabase, ProcessingFlagsCanOnlyBeClaimedOnce) {
  kiss_matcher_slam::MapDatabase database;
  database.append(makeKeyframe(1.0));

  EXPECT_TRUE(database.markPlaceRecognitionProcessed(0));
  EXPECT_FALSE(database.markPlaceRecognitionProcessed(0));
  EXPECT_TRUE(database.markProximitySearchProcessed(0));
  EXPECT_FALSE(database.markProximitySearchProcessed(0));
  EXPECT_FALSE(database.markPlaceRecognitionProcessed(1));
}

TEST(MapDatabase, LatestProcessingClaimIsAtomicAndTracksNewFrames) {
  kiss_matcher_slam::MapDatabase database;
  EXPECT_FALSE(database.claimLatestForPlaceRecognition().has_value());

  database.append(makeKeyframe(1.0));
  ASSERT_TRUE(database.claimLatestForPlaceRecognition().has_value());
  EXPECT_EQ(*database.claimLatestForProximitySearch(), 0U);
  EXPECT_FALSE(database.claimLatestForPlaceRecognition().has_value());

  database.append(makeKeyframe(2.0));
  EXPECT_EQ(*database.claimLatestForPlaceRecognition(), 1U);
  EXPECT_EQ(*database.claimLatestForProximitySearch(), 1U);
}

TEST(MapDatabase, ConcurrentReadersObserveConsistentSnapshots) {
  kiss_matcher_slam::MapDatabase database;
  constexpr std::size_t count = 100;
  std::atomic<bool> writer_done{false};
  std::atomic<bool> inconsistent{false};

  std::thread writer([&] {
    for (std::size_t i = 0; i < count; ++i) database.append(makeKeyframe(i, i));
    writer_done = true;
  });
  std::thread reader([&] {
    while (!writer_done) {
      const auto frames = database.snapshot();
      for (std::size_t i = 0; i < frames.size(); ++i) {
        if (frames[i].id != i || frames[i].timestamp != static_cast<double>(i)) {
          inconsistent = true;
        }
      }
    }
  });

  writer.join();
  reader.join();
  EXPECT_FALSE(inconsistent);
  EXPECT_EQ(database.size(), count);
}

}  // namespace
