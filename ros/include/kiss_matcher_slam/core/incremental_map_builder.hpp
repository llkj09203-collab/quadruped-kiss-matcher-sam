#pragma once

#include <cstddef>
#include <mutex>
#include <utility>
#include <vector>

#include <pcl/point_cloud.h>

#include "kiss_matcher_slam/core/keyframe.hpp"

namespace kiss_matcher_slam {

struct IncrementalMapBuilderConfig {
  double scan_voxel_resolution = 0.3;
  double map_voxel_resolution = 1.0;
  bool keyframe_clouds_are_voxelized = false;
};

struct IncrementalMapBuildResult {
  pcl::PointCloud<SlamPoint> cloud;
  std::vector<std::pair<std::size_t, pcl::PointCloud<SlamPoint>>> visualization_cache_updates;
};

// Builds the RViz map incrementally and rebuilds it from optimized poses after
// graph corrections. This map is a visualization product, not the saved map.
class IncrementalMapBuilder {
 public:
  explicit IncrementalMapBuilder(IncrementalMapBuilderConfig config);

  IncrementalMapBuildResult build(const std::vector<Keyframe> &keyframes,
                                  bool rebuild_from_zero);
  std::size_t processedKeyframes() const;

 private:
  static pcl::PointCloud<SlamPoint> voxelize(const pcl::PointCloud<SlamPoint> &cloud,
                                             double resolution);

  IncrementalMapBuilderConfig config_;
  mutable std::mutex mutex_;
  pcl::PointCloud<SlamPoint> accumulated_map_;
  std::size_t next_keyframe_id_ = 0;
};

}  // namespace kiss_matcher_slam
