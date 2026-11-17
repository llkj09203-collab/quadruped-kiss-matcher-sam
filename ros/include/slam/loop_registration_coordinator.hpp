#pragma once

#include <cstddef>
#include <mutex>
#include <optional>

#include "kiss_matcher_slam/core/loop_candidate_queue.hpp"
#include "kiss_matcher_slam/core/loop_constraint_processor.hpp"
#include "kiss_matcher_slam/core/map_database.hpp"
#include "slam/loop_closure.h"

namespace kiss_matcher_slam {

enum class LoopRegistrationStatus {
  kNoWork,
  kInvalidCandidate,
  kRejectedMatcher,
  kRejectedOverlap,
  kAccepted,
};

struct LoopRegistrationOutcome {
  LoopRegistrationStatus status = LoopRegistrationStatus::kNoWork;
  std::size_t query_id = 0;
  std::size_t match_id = 0;
  double overlap_percent = 0.0;
  double elapsed_ms = 0.0;
};

struct LoopDebugClouds {
  std::size_t query_id = 0;
  pcl::PointCloud<SlamPoint> source;
  pcl::PointCloud<SlamPoint> target;
  pcl::PointCloud<SlamPoint> coarse_aligned;
  pcl::PointCloud<SlamPoint> final_aligned;
  pcl::PointCloud<SlamPoint> debug;
};

// Coordinates one queued registration without depending on ROS publishers,
// timers, or logging. Debug clouds are snapshotted to avoid races with the next
// registration task.
class LoopRegistrationCoordinator {
 public:
  LoopRegistrationCoordinator(LoopCandidateQueue &candidate_queue,
                              MapDatabase &map_database,
                              kiss_matcher::LoopClosure &loop_closure,
                              LoopConstraintProcessor &constraint_processor);

  LoopRegistrationOutcome processNext();
  std::optional<LoopDebugClouds> consumeDebugClouds();

 private:
  LoopCandidateQueue &candidate_queue_;
  MapDatabase &map_database_;
  kiss_matcher::LoopClosure &loop_closure_;
  LoopConstraintProcessor &constraint_processor_;
  std::mutex debug_mutex_;
  std::optional<LoopDebugClouds> pending_debug_clouds_;
};

}  // namespace kiss_matcher_slam
