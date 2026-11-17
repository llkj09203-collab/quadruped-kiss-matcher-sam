#include "kiss_matcher_slam/core/mapping_runtime_state.hpp"

namespace kiss_matcher_slam {

bool MappingRuntimeState::initialized() const noexcept {
  return next_keyframe_id_.load(std::memory_order_acquire) != 0;
}

std::size_t MappingRuntimeState::nextKeyframeId() const noexcept {
  return next_keyframe_id_.load(std::memory_order_acquire);
}

std::size_t MappingRuntimeState::commitKeyframe() noexcept {
  return next_keyframe_id_.fetch_add(1, std::memory_order_acq_rel);
}

void MappingRuntimeState::requestMapRebuild() noexcept {
  map_rebuild_requested_.store(true, std::memory_order_release);
}

bool MappingRuntimeState::consumeMapRebuildRequest() noexcept {
  return map_rebuild_requested_.exchange(false, std::memory_order_acq_rel);
}

void MappingRuntimeState::requestGraphRefresh() noexcept {
  graph_refresh_requested_.store(true, std::memory_order_release);
}

bool MappingRuntimeState::consumeGraphRefreshRequest() noexcept {
  return graph_refresh_requested_.exchange(false, std::memory_order_acq_rel);
}

void MappingRuntimeState::reset(const std::size_t next_keyframe_id) noexcept {
  next_keyframe_id_.store(next_keyframe_id, std::memory_order_release);
  map_rebuild_requested_.store(false, std::memory_order_release);
  graph_refresh_requested_.store(false, std::memory_order_release);
}

}  // namespace kiss_matcher_slam
