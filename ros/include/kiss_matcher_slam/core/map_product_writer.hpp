#pragma once

#include <cstddef>
#include <filesystem>
#include <iosfwd>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "kiss_matcher_slam/core/keyframe.hpp"

namespace kiss_matcher_slam {

struct MapProductConfig {
  std::filesystem::path output_directory = "maps";
  std::string sequence = "sequence";
  std::string map_frame = "map";
  std::string base_frame = "base";
  double save_voxel_resolution = 0.3;
  double keyframe_translation_threshold_m = 1.0;
  double keyframe_rotation_threshold_rad = 0.0;
  bool gps_altitude_enabled = false;
  bool save_keyframes_incrementally = false;
  bool overwrite_existing = false;
};

// Owns the on-disk representation of a mapping session. File operations are
// serialized so timer callbacks and shutdown cannot write a product together.
class MapProductWriter {
 public:
  explicit MapProductWriter(MapProductConfig config);

  void initialize();
  void appendKeyframe(const Keyframe &keyframe);
  void writeOptimizedTrajectory(const std::vector<Keyframe> &keyframes,
                                const std::vector<Eigen::Matrix4d> &optimized_poses);
  std::filesystem::path writeMap(const std::vector<Keyframe> &keyframes,
                                 bool binary = true);
  std::filesystem::path writeMap(const std::vector<Keyframe> &keyframes,
                                 const std::filesystem::path &output_root,
                                 bool binary);
  std::filesystem::path writeKittiDataset(const std::vector<Keyframe> &keyframes,
                                          const std::filesystem::path &output_root);

  const MapProductConfig &config() const { return config_; }
  std::filesystem::path sequenceDirectory() const;

 private:
  std::filesystem::path sequenceDirectory(const std::filesystem::path &root) const;
  static void validateKeyframes(const std::vector<Keyframe> &keyframes);
  static void writeTumPose(std::ostream &stream,
                           double timestamp,
                           const Eigen::Matrix4d &pose,
                           int precision);
  static void requireStream(const std::ios &stream, const std::filesystem::path &path);
  std::filesystem::path writeMapLocked(const std::vector<Keyframe> &keyframes,
                                       const std::filesystem::path &output_root,
                                       bool binary);

  MapProductConfig config_;
  mutable std::mutex mutex_;
  bool initialized_ = false;
  std::size_t next_keyframe_id_ = 0;
};

}  // namespace kiss_matcher_slam
