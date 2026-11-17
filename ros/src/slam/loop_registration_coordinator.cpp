#include "slam/loop_registration_coordinator.hpp"

#include <chrono>
#include <utility>

namespace kiss_matcher_slam {

LoopRegistrationCoordinator::LoopRegistrationCoordinator(
    LoopCandidateQueue &candidate_queue,
    MapDatabase &map_database,
    kiss_matcher::LoopClosure &loop_closure,
    LoopConstraintProcessor &constraint_processor)
    : candidate_queue_(candidate_queue),
      map_database_(map_database),
      loop_closure_(loop_closure),
      constraint_processor_(constraint_processor) {}

LoopRegistrationOutcome LoopRegistrationCoordinator::processNext() {
  const auto candidate = candidate_queue_.tryPop();
  if (!candidate) return {};

  LoopRegistrationOutcome outcome;
  outcome.query_id = candidate->first;
  outcome.match_id = candidate->second;
  const auto keyframes = map_database_.snapshot();
  if (outcome.query_id >= keyframes.size() || outcome.match_id >= keyframes.size() ||
      outcome.query_id == outcome.match_id) {
    outcome.status = LoopRegistrationStatus::kInvalidCandidate;
    return outcome;
  }

  const auto start = std::chrono::steady_clock::now();
  const auto registration =
      loop_closure_.performLoopClosure(keyframes, outcome.query_id, outcome.match_id);
  outcome.elapsed_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
  outcome.overlap_percent = registration.overlapness_;
  if (!registration.is_valid_) {
    outcome.status = registration.overlapness_ == 0.0
                         ? LoopRegistrationStatus::kRejectedMatcher
                         : LoopRegistrationStatus::kRejectedOverlap;
    return outcome;
  }

  constraint_processor_.apply({outcome.query_id,
                               outcome.match_id,
                               registration.pose_,
                               registration.overlapness_},
                              keyframes);

  LoopDebugClouds debug;
  debug.query_id = outcome.query_id;
  debug.source = loop_closure_.getSourceCloud();
  debug.target = loop_closure_.getTargetCloud();
  debug.coarse_aligned = loop_closure_.getCoarseAlignedCloud();
  debug.final_aligned = loop_closure_.getFinalAlignedCloud();
  debug.debug = loop_closure_.getDebugCloud();
  {
    std::lock_guard<std::mutex> lock(debug_mutex_);
    pending_debug_clouds_ = std::move(debug);
  }
  outcome.status = LoopRegistrationStatus::kAccepted;
  return outcome;
}

std::optional<LoopDebugClouds> LoopRegistrationCoordinator::consumeDebugClouds() {
  std::lock_guard<std::mutex> lock(debug_mutex_);
  auto result = std::move(pending_debug_clouds_);
  pending_debug_clouds_.reset();
  return result;
}

}  // namespace kiss_matcher_slam
