#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include <Eigen/Core>

#include "kiss_matcher_slam/core/keyframe.hpp"
#include "kiss_matcher_slam/core/results.hpp"

namespace kiss_matcher_slam {

struct ScanContextConfig {
  int num_rings = 20;
  int num_sectors = 60;
  int exclude_recent = 30;
  int num_candidates = 10;
  double max_radius_m = 80.0;
  double distance_threshold = 0.18;
  double minimum_time_separation_s = 30.0;
};

// Stateful Scan Context place detector. Descriptor generation, storage and
// retrieval intentionally stay together because they share one configuration
// and one keyframe-indexed lifetime.
class ScanContextLoopDetector {
 public:
  explicit ScanContextLoopDetector(ScanContextConfig config);

  std::optional<PlaceCandidate> detect(const Keyframe &query,
                                       const std::vector<Keyframe> &keyframes);
  std::size_t descriptorCount() const noexcept { return descriptors_.size(); }
  const ScanContextConfig &config() const noexcept { return config_; }

 private:
  struct DistanceResult {
    double distance = 1.0;
    int sector_shift = 0;
  };

  Eigen::MatrixXf makeDescriptor(const pcl::PointCloud<SlamPoint> &cloud) const;
  Eigen::VectorXf makeRingKey(const Eigen::MatrixXf &descriptor) const;
  DistanceResult descriptorDistance(const Eigen::MatrixXf &query,
                                    const Eigen::MatrixXf &candidate) const;
  void appendMissingDescriptors(const std::vector<Keyframe> &keyframes);

  ScanContextConfig config_;
  std::vector<Eigen::MatrixXf> descriptors_;
  std::vector<Eigen::VectorXf> ring_keys_;
};

}  // namespace kiss_matcher_slam
