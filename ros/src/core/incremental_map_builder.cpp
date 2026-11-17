#include "kiss_matcher_slam/core/incremental_map_builder.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

namespace kiss_matcher_slam {

IncrementalMapBuilder::IncrementalMapBuilder(IncrementalMapBuilderConfig config)
    : config_(std::move(config)) {
  if (!std::isfinite(config_.scan_voxel_resolution) ||
      config_.scan_voxel_resolution <= 0.0 ||
      !std::isfinite(config_.map_voxel_resolution) || config_.map_voxel_resolution <= 0.0) {
    throw std::invalid_argument("invalid incremental map voxel resolution");
  }
}

pcl::PointCloud<SlamPoint> IncrementalMapBuilder::voxelize(
    const pcl::PointCloud<SlamPoint> &cloud,
    const double resolution) {
  if (cloud.empty()) return {};
  pcl::PointCloud<SlamPoint>::Ptr input(new pcl::PointCloud<SlamPoint>(cloud));
  pcl::VoxelGrid<SlamPoint> filter;
  filter.setLeafSize(resolution, resolution, resolution);
  filter.setInputCloud(input);
  pcl::PointCloud<SlamPoint> output;
  filter.filter(output);
  return output;
}

IncrementalMapBuildResult IncrementalMapBuilder::build(
    const std::vector<Keyframe> &keyframes,
    const bool rebuild_from_zero) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (rebuild_from_zero) {
    accumulated_map_.clear();
    next_keyframe_id_ = 0;
  }
  if (next_keyframe_id_ > keyframes.size()) {
    throw std::invalid_argument("keyframe collection shrank without requesting a map rebuild");
  }
  for (std::size_t i = 0; i < keyframes.size(); ++i) {
    if (keyframes[i].id != i || !keyframes[i].optimized_pose.allFinite()) {
      throw std::invalid_argument("invalid keyframe collection for incremental map");
    }
  }

  IncrementalMapBuildResult result;
  for (std::size_t i = next_keyframe_id_; i < keyframes.size(); ++i) {
    const pcl::PointCloud<SlamPoint> *scan = &keyframes[i].cloud;
    pcl::PointCloud<SlamPoint> generated_cache;
    if (!config_.keyframe_clouds_are_voxelized) {
      if (keyframes[i].visualization_cloud.empty()) {
        generated_cache = voxelize(keyframes[i].cloud, config_.scan_voxel_resolution);
        result.visualization_cache_updates.emplace_back(i, generated_cache);
        scan = &generated_cache;
      } else {
        scan = &keyframes[i].visualization_cloud;
      }
    }
    pcl::PointCloud<SlamPoint> transformed;
    pcl::transformPointCloud(*scan, transformed, keyframes[i].optimized_pose);
    accumulated_map_ += transformed;
  }
  next_keyframe_id_ = keyframes.size();
  result.cloud = voxelize(accumulated_map_, config_.map_voxel_resolution);
  return result;
}

std::size_t IncrementalMapBuilder::processedKeyframes() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return next_keyframe_id_;
}

}  // namespace kiss_matcher_slam
