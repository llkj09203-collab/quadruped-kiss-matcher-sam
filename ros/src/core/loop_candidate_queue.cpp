#include "kiss_matcher_slam/core/loop_candidate_queue.hpp"

namespace kiss_matcher_slam {

void LoopCandidateQueue::push(LoopCandidate candidate) {
  std::lock_guard<std::mutex> lock(mutex_);
  candidates_.push(std::move(candidate));
}

std::optional<LoopCandidate> LoopCandidateQueue::tryPop() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (candidates_.empty()) return std::nullopt;
  auto candidate = candidates_.front();
  candidates_.pop();
  return candidate;
}

std::size_t LoopCandidateQueue::size() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return candidates_.size();
}

void LoopCandidateQueue::clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  std::queue<LoopCandidate> empty;
  candidates_.swap(empty);
}

}  // namespace kiss_matcher_slam
