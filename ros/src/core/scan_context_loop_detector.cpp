#include "kiss_matcher_slam/core/scan_context_loop_detector.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace kiss_matcher_slam {
namespace {
constexpr double kTwoPi = 6.28318530717958647692;
}

ScanContextLoopDetector::ScanContextLoopDetector(ScanContextConfig config)
    : config_(std::move(config)) {
  if (config_.num_rings <= 0 || config_.num_sectors <= 0 || config_.exclude_recent < 0 ||
      config_.num_candidates <= 0 || !std::isfinite(config_.max_radius_m) ||
      config_.max_radius_m <= 0.0 || !std::isfinite(config_.distance_threshold) ||
      config_.distance_threshold < 0.0 || config_.distance_threshold > 1.0 ||
      !std::isfinite(config_.minimum_time_separation_s) ||
      config_.minimum_time_separation_s < 0.0) {
    throw std::invalid_argument("invalid Scan Context configuration");
  }
}

Eigen::MatrixXf ScanContextLoopDetector::makeDescriptor(
    const pcl::PointCloud<SlamPoint> &cloud) const {
  Eigen::MatrixXf descriptor =
      Eigen::MatrixXf::Constant(config_.num_rings, config_.num_sectors, -1000.0f);
  const double ring_step = config_.max_radius_m / static_cast<double>(config_.num_rings);
  const double sector_step = kTwoPi / static_cast<double>(config_.num_sectors);

  for (const auto &point : cloud.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
    const double radius = std::hypot(point.x, point.y);
    if (radius < 1e-3 || radius > config_.max_radius_m) continue;
    double angle = std::atan2(point.y, point.x);
    if (angle < 0.0) angle += kTwoPi;
    const int ring = std::min(config_.num_rings - 1, static_cast<int>(radius / ring_step));
    const int sector =
        std::min(config_.num_sectors - 1, static_cast<int>(angle / sector_step));
    descriptor(ring, sector) = std::max(descriptor(ring, sector), point.z);
  }
  return descriptor.unaryExpr([](const float value) { return value < -999.0f ? 0.0f : value; });
}

Eigen::VectorXf ScanContextLoopDetector::makeRingKey(const Eigen::MatrixXf &descriptor) const {
  return descriptor.rowwise().mean();
}

ScanContextLoopDetector::DistanceResult ScanContextLoopDetector::descriptorDistance(
    const Eigen::MatrixXf &query, const Eigen::MatrixXf &candidate) const {
  DistanceResult best;
  for (int shift = 0; shift < config_.num_sectors; ++shift) {
    double similarity_sum = 0.0;
    int valid_columns = 0;
    for (int col = 0; col < config_.num_sectors; ++col) {
      const auto query_column = query.col(col);
      const auto candidate_column = candidate.col((col + shift) % config_.num_sectors);
      const double denominator = query_column.norm() * candidate_column.norm();
      if (denominator > 1e-6) {
        similarity_sum += query_column.dot(candidate_column) / denominator;
        ++valid_columns;
      }
    }
    if (valid_columns == 0) continue;
    const double distance = 1.0 - similarity_sum / static_cast<double>(valid_columns);
    if (distance < best.distance) best = {distance, shift};
  }
  return best;
}

void ScanContextLoopDetector::appendMissingDescriptors(
    const std::vector<Keyframe> &keyframes) {
  while (descriptors_.size() < keyframes.size()) {
    const auto index = descriptors_.size();
    if (keyframes[index].id != index) {
      throw std::invalid_argument("Scan Context keyframes must have contiguous IDs");
    }
    auto descriptor = makeDescriptor(keyframes[index].cloud);
    ring_keys_.push_back(makeRingKey(descriptor));
    descriptors_.push_back(std::move(descriptor));
  }
}

std::optional<PlaceCandidate> ScanContextLoopDetector::detect(
    const Keyframe &query, const std::vector<Keyframe> &keyframes) {
  using Clock = std::chrono::steady_clock;
  const auto total_start = Clock::now();
  last_stats_ = {};
  last_stats_.history_size = keyframes.size();
  if (keyframes.empty()) return std::nullopt;
  const auto descriptor_count_before = descriptors_.size();
  const auto descriptor_start = Clock::now();
  appendMissingDescriptors(keyframes);
  last_stats_.descriptor_build_ms =
      std::chrono::duration<double, std::milli>(Clock::now() - descriptor_start).count();
  last_stats_.descriptors_built = descriptors_.size() - descriptor_count_before;

  if (query.id >= descriptors_.size() || query.id >= keyframes.size() ||
      query.id <= static_cast<std::size_t>(config_.exclude_recent)) {
    last_stats_.total_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - total_start).count();
    return std::nullopt;
  }

  const auto retrieval_start = Clock::now();

  struct Candidate {
    std::size_t id;
    double ring_distance;
  };
  std::vector<Candidate> candidates;
  const std::size_t search_end = query.id - static_cast<std::size_t>(config_.exclude_recent);
  for (std::size_t id = 0; id < search_end; ++id) {
    if (query.timestamp - keyframes[id].timestamp < config_.minimum_time_separation_s) continue;
    candidates.push_back({id, (ring_keys_[query.id] - ring_keys_[id]).norm()});
  }
  last_stats_.eligible_candidates = candidates.size();
  if (candidates.empty()) {
    last_stats_.retrieval_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - retrieval_start).count();
    last_stats_.total_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - total_start).count();
    return std::nullopt;
  }

  const std::size_t candidate_count =
      std::min(candidates.size(), static_cast<std::size_t>(config_.num_candidates));
  last_stats_.shortlisted_candidates = candidate_count;
  std::partial_sort(candidates.begin(),
                    candidates.begin() + candidate_count,
                    candidates.end(),
                    [](const Candidate &lhs, const Candidate &rhs) {
                      return lhs.ring_distance < rhs.ring_distance;
                    });

  PlaceCandidate best;
  for (std::size_t i = 0; i < candidate_count; ++i) {
    ++last_stats_.full_descriptor_comparisons;
    const auto result = descriptorDistance(descriptors_[query.id], descriptors_[candidates[i].id]);
    if (result.distance < best.descriptor_distance) {
      best.keyframe_id = candidates[i].id;
      best.descriptor_distance = result.distance;
      int signed_shift = result.sector_shift;
      if (signed_shift > config_.num_sectors / 2) signed_shift -= config_.num_sectors;
      best.yaw_offset_rad = signed_shift * kTwoPi / config_.num_sectors;
    }
  }

  last_stats_.retrieval_ms =
      std::chrono::duration<double, std::milli>(Clock::now() - retrieval_start).count();
  last_stats_.total_ms =
      std::chrono::duration<double, std::milli>(Clock::now() - total_start).count();
  if (best.descriptor_distance >= config_.distance_threshold) return std::nullopt;
  return best;
}

}  // namespace kiss_matcher_slam
