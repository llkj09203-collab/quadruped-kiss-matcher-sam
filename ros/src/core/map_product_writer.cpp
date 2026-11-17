#include "kiss_matcher_slam/core/map_product_writer.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

namespace kiss_matcher_slam {
namespace fs = std::filesystem;
namespace {
constexpr double kRadiansToDegrees = 57.295779513082320876;

void requirePcdWrite(const int result, const fs::path &path) {
  if (result < 0) throw std::runtime_error("failed to write PCD file: " + path.string());
}

pcl::PointCloud<SlamPoint> buildMapCloud(const std::vector<Keyframe> &keyframes,
                                         const double voxel_resolution) {
  pcl::PointCloud<SlamPoint>::Ptr accumulated(new pcl::PointCloud<SlamPoint>());
  std::size_t point_count = 0;
  for (const auto &keyframe : keyframes) point_count += keyframe.cloud.size();
  accumulated->reserve(point_count);

  for (const auto &keyframe : keyframes) {
    pcl::PointCloud<SlamPoint> transformed;
    pcl::transformPointCloud(keyframe.cloud, transformed, keyframe.optimized_pose);
    *accumulated += transformed;
  }

  pcl::VoxelGrid<SlamPoint> voxel_grid;
  voxel_grid.setLeafSize(voxel_resolution, voxel_resolution, voxel_resolution);
  voxel_grid.setInputCloud(accumulated);
  pcl::PointCloud<SlamPoint> output;
  voxel_grid.filter(output);
  return output;
}

}  // namespace

MapProductWriter::MapProductWriter(MapProductConfig config) : config_(std::move(config)) {
  if (config_.output_directory.empty()) {
    throw std::invalid_argument("map product output directory must not be empty");
  }
  if (config_.sequence.empty() || config_.sequence == "." || config_.sequence == ".." ||
      fs::path(config_.sequence).has_parent_path()) {
    throw std::invalid_argument("map product sequence must be one safe path component");
  }
  if (!std::isfinite(config_.save_voxel_resolution) || config_.save_voxel_resolution <= 0.0 ||
      !std::isfinite(config_.keyframe_translation_threshold_m) ||
      config_.keyframe_translation_threshold_m < 0.0 ||
      !std::isfinite(config_.keyframe_rotation_threshold_rad) ||
      config_.keyframe_rotation_threshold_rad < 0.0) {
    throw std::invalid_argument("invalid map product numeric configuration");
  }
}

fs::path MapProductWriter::sequenceDirectory() const {
  return sequenceDirectory(config_.output_directory);
}

fs::path MapProductWriter::sequenceDirectory(const fs::path &root) const {
  if (root.empty()) throw std::invalid_argument("map product output root must not be empty");
  return root / config_.sequence;
}

void MapProductWriter::requireStream(const std::ios &stream, const fs::path &path) {
  if (!stream) throw std::runtime_error("failed to write map product file: " + path.string());
}

void MapProductWriter::writeTumPose(std::ostream &stream,
                                    const double timestamp,
                                    const Eigen::Matrix4d &pose,
                                    const int precision) {
  const Eigen::Quaterniond quaternion(pose.block<3, 3>(0, 0));
  stream << std::fixed << std::setprecision(precision) << timestamp << " " << pose(0, 3) << " "
         << pose(1, 3) << " " << pose(2, 3) << " " << quaternion.x() << " " << quaternion.y()
         << " " << quaternion.z() << " " << quaternion.w() << "\n";
}

void MapProductWriter::validateKeyframes(const std::vector<Keyframe> &keyframes) {
  for (std::size_t i = 0; i < keyframes.size(); ++i) {
    if (keyframes[i].id != i || !std::isfinite(keyframes[i].timestamp) ||
        !keyframes[i].odometry_pose.allFinite() || !keyframes[i].optimized_pose.allFinite()) {
      throw std::invalid_argument("invalid or non-contiguous keyframe data product");
    }
  }
}

void MapProductWriter::initialize() {
  if (!config_.save_keyframes_incrementally) return;
  std::lock_guard<std::mutex> lock(mutex_);
  const fs::path directory = sequenceDirectory();
  if (fs::exists(directory) && !fs::is_empty(directory)) {
    if (!config_.overwrite_existing) {
      throw std::runtime_error("map product session already exists: " + directory.string());
    }
    fs::remove_all(directory);
  }
  fs::create_directories(directory / "keyframes");

  const fs::path metadata_path = directory / "metadata.yaml";
  std::ofstream metadata(metadata_path, std::ios::trunc);
  metadata << "format_version: 1\n"
           << "sequence: " << config_.sequence << "\n"
           << "map_frame: " << config_.map_frame << "\n"
           << "base_frame: " << config_.base_frame << "\n"
           << "pointcloud_frame: lidar\n"
           << "trajectory_format: tum\n"
           << "keyframe_translation_threshold_m: "
           << config_.keyframe_translation_threshold_m << "\n"
           << "keyframe_rotation_threshold_deg: "
           << config_.keyframe_rotation_threshold_rad * kRadiansToDegrees << "\n"
           << "gps_altitude_enabled: "
           << (config_.gps_altitude_enabled ? "true" : "false") << "\n";
  metadata.flush();
  requireStream(metadata, metadata_path);

  for (const char *filename : {"timestamps.txt", "odometry_tum.txt"}) {
    const fs::path path = directory / filename;
    std::ofstream file(path, std::ios::trunc);
    requireStream(file, path);
  }
  next_keyframe_id_ = 0;
  initialized_ = true;
}

void MapProductWriter::appendKeyframe(const Keyframe &keyframe) {
  if (!config_.save_keyframes_incrementally) return;
  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialized_) throw std::logic_error("map product writer is not initialized");
  if (!std::isfinite(keyframe.timestamp) || !keyframe.odometry_pose.allFinite()) {
    throw std::invalid_argument("invalid keyframe map product");
  }
  if (keyframe.id != next_keyframe_id_) {
    throw std::invalid_argument("incremental keyframes must have contiguous identifiers");
  }

  const fs::path directory = sequenceDirectory();
  std::ostringstream filename;
  filename << std::setw(6) << std::setfill('0') << keyframe.id << ".pcd";
  const fs::path pcd_path = directory / "keyframes" / filename.str();
  requirePcdWrite(pcl::io::savePCDFileBinary<SlamPoint>(pcd_path.string(), keyframe.cloud),
                  pcd_path);

  const fs::path timestamp_path = directory / "timestamps.txt";
  std::ofstream timestamps(timestamp_path, std::ios::app);
  timestamps << keyframe.id << " " << std::fixed << std::setprecision(9) << keyframe.timestamp
             << "\n";
  timestamps.flush();
  requireStream(timestamps, timestamp_path);

  const fs::path odometry_path = directory / "odometry_tum.txt";
  std::ofstream odometry(odometry_path, std::ios::app);
  writeTumPose(odometry, keyframe.timestamp, keyframe.odometry_pose, 9);
  odometry.flush();
  requireStream(odometry, odometry_path);
  ++next_keyframe_id_;
}

void MapProductWriter::writeOptimizedTrajectory(
    const std::vector<Keyframe> &keyframes,
    const std::vector<Eigen::Matrix4d> &optimized_poses) {
  if (!config_.save_keyframes_incrementally || optimized_poses.empty()) return;
  validateKeyframes(keyframes);
  if (keyframes.size() != optimized_poses.size()) {
    throw std::invalid_argument("optimized trajectory and keyframes have different sizes");
  }

  std::lock_guard<std::mutex> lock(mutex_);
  const fs::path path = sequenceDirectory() / "optimized_trajectory_tum.txt";
  std::ofstream output(path, std::ios::trunc);
  output << "# timestamp x y z qx qy qz qw\n";
  for (std::size_t i = 0; i < keyframes.size(); ++i) {
    if (!optimized_poses[i].allFinite()) {
      throw std::invalid_argument("optimized trajectory contains a non-finite pose");
    }
    writeTumPose(output, keyframes[i].timestamp, optimized_poses[i], 9);
  }
  output.flush();
  requireStream(output, path);
}

fs::path MapProductWriter::writeMap(const std::vector<Keyframe> &keyframes, const bool binary) {
  return writeMap(keyframes, config_.output_directory, binary);
}

fs::path MapProductWriter::writeMap(const std::vector<Keyframe> &keyframes,
                                    const fs::path &output_root,
                                    const bool binary) {
  validateKeyframes(keyframes);
  if (keyframes.empty()) throw std::invalid_argument("cannot write a map without keyframes");
  std::lock_guard<std::mutex> lock(mutex_);
  return writeMapLocked(keyframes, output_root, binary);
}

fs::path MapProductWriter::writeMapLocked(const std::vector<Keyframe> &keyframes,
                                          const fs::path &output_root,
                                          const bool binary) {
  const fs::path directory = sequenceDirectory(output_root);
  fs::create_directories(directory);
  const fs::path path = directory / (config_.sequence + "_map.pcd");
  const auto map = buildMapCloud(keyframes, config_.save_voxel_resolution);
  const int result = binary ? pcl::io::savePCDFileBinary<SlamPoint>(path.string(), map)
                            : pcl::io::savePCDFileASCII<SlamPoint>(path.string(), map);
  requirePcdWrite(result, path);
  return path;
}

fs::path MapProductWriter::writeKittiDataset(const std::vector<Keyframe> &keyframes,
                                             const fs::path &output_root) {
  validateKeyframes(keyframes);
  if (keyframes.empty()) throw std::invalid_argument("cannot write an empty KITTI dataset");
  std::lock_guard<std::mutex> lock(mutex_);
  const fs::path directory = sequenceDirectory(output_root);
  const fs::path scans = directory / "scans";
  if (fs::exists(scans) && !fs::is_empty(scans)) {
    if (!config_.overwrite_existing) {
      throw std::runtime_error("KITTI scan directory already exists: " + scans.string());
    }
    fs::remove_all(scans);
  }
  fs::create_directories(scans);

  const fs::path kitti_path = directory / "poses_kitti.txt";
  const fs::path tum_path = directory / "poses_tum.txt";
  std::ofstream kitti(kitti_path, std::ios::trunc);
  std::ofstream tum(tum_path, std::ios::trunc);
  tum << "#timestamp x y z qx qy qz qw\n";

  for (std::size_t i = 0; i < keyframes.size(); ++i) {
    std::ostringstream filename;
    filename << std::setw(6) << std::setfill('0') << i << ".pcd";
    const fs::path scan_path = scans / filename.str();
    requirePcdWrite(
        pcl::io::savePCDFileASCII<SlamPoint>(scan_path.string(), keyframes[i].cloud), scan_path);

    const auto &pose = keyframes[i].optimized_pose;
    kitti << pose(0, 0) << " " << pose(0, 1) << " " << pose(0, 2) << " " << pose(0, 3) << " "
          << pose(1, 0) << " " << pose(1, 1) << " " << pose(1, 2) << " " << pose(1, 3) << " "
          << pose(2, 0) << " " << pose(2, 1) << " " << pose(2, 2) << " " << pose(2, 3) << "\n";
    writeTumPose(tum, keyframes[i].timestamp, pose, 8);
  }
  kitti.flush();
  tum.flush();
  requireStream(kitti, kitti_path);
  requireStream(tum, tum_path);
  return directory;
}

}  // namespace kiss_matcher_slam
