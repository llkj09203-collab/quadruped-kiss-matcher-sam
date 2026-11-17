#pragma once

#include <atomic>
#include <cstddef>

namespace kiss_matcher_slam {

class MappingRuntimeState {
 public:
  bool initialized() const noexcept;
  std::size_t nextKeyframeId() const noexcept;
  std::size_t commitKeyframe() noexcept;
  void requestMapRebuild() noexcept;
  bool consumeMapRebuildRequest() noexcept;
  void requestGraphRefresh() noexcept;
  bool consumeGraphRefreshRequest() noexcept;
  void reset(std::size_t next_keyframe_id = 0) noexcept;

 private:
  std::atomic<std::size_t> next_keyframe_id_{0};
  std::atomic<bool> map_rebuild_requested_{false};
  std::atomic<bool> graph_refresh_requested_{false};
};

}  // namespace kiss_matcher_slam
