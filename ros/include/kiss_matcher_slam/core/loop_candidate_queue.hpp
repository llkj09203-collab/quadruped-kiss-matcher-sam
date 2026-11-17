#pragma once

#include <cstddef>
#include <mutex>
#include <optional>
#include <queue>
#include <utility>

namespace kiss_matcher_slam {

using LoopCandidate = std::pair<std::size_t, std::size_t>;

class LoopCandidateQueue {
 public:
  void push(LoopCandidate candidate);
  std::optional<LoopCandidate> tryPop();
  std::size_t size() const;
  void clear();

 private:
  mutable std::mutex mutex_;
  std::queue<LoopCandidate> candidates_;
};

}  // namespace kiss_matcher_slam
