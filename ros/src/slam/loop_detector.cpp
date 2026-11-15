#include "slam/loop_detector.h"

#include <algorithm>
#include <cmath>

using namespace kiss_matcher;

LoopDetector::LoopDetector(const LoopDetectorConfig &config, const rclcpp::Logger &logger)
    : config_(config), logger_(logger) {}

LoopDetector::~LoopDetector() {}

Eigen::MatrixXf LoopDetector::makeDescriptor(const pcl::PointCloud<PointType> &cloud) const {
  Eigen::MatrixXf descriptor =
      Eigen::MatrixXf::Constant(config_.num_rings_, config_.num_sectors_, -1000.0f);
  const double ring_step = config_.max_radius_ / static_cast<double>(config_.num_rings_);
  const double sector_step = 2.0 * M_PI / static_cast<double>(config_.num_sectors_);

  for (const auto &point : cloud.points) {
    const double radius = std::hypot(point.x, point.y);
    if (radius < 1e-3 || radius > config_.max_radius_) continue;
    double angle = std::atan2(point.y, point.x);
    if (angle < 0.0) angle += 2.0 * M_PI;
    const int ring = std::min(config_.num_rings_ - 1, static_cast<int>(radius / ring_step));
    const int sector =
        std::min(config_.num_sectors_ - 1, static_cast<int>(angle / sector_step));
    descriptor(ring, sector) = std::max(descriptor(ring, sector), point.z);
  }
  descriptor = descriptor.unaryExpr([](float value) { return value < -999.0f ? 0.0f : value; });
  return descriptor;
}

Eigen::VectorXf LoopDetector::makeRingKey(const Eigen::MatrixXf &descriptor) const {
  return descriptor.rowwise().mean();
}

double LoopDetector::descriptorDistance(const Eigen::MatrixXf &query,
                                        const Eigen::MatrixXf &candidate) const {
  double best_distance = 1.0;
  for (int shift = 0; shift < config_.num_sectors_; ++shift) {
    double similarity_sum = 0.0;
    int valid_columns = 0;
    for (int col = 0; col < config_.num_sectors_; ++col) {
      const auto query_column = query.col(col);
      const auto candidate_column = candidate.col((col + shift) % config_.num_sectors_);
      const double denominator = query_column.norm() * candidate_column.norm();
      if (denominator > 1e-6) {
        similarity_sum += query_column.dot(candidate_column) / denominator;
        ++valid_columns;
      }
    }
    if (valid_columns > 0) {
      best_distance =
          std::min(best_distance, 1.0 - similarity_sum / static_cast<double>(valid_columns));
    }
  }
  return best_distance;
}

LoopIdxPairs LoopDetector::fetchLoopCandidates(const PoseGraphNode &query_frame,
                                               const std::vector<PoseGraphNode> &keyframes) {
  LoopIdxPairs loop_idx_pairs;
  if (!config_.enable_ || keyframes.empty()) return loop_idx_pairs;

  while (descriptors_.size() < keyframes.size()) {
    const auto &keyframe = keyframes[descriptors_.size()];
    auto descriptor = makeDescriptor(keyframe.scan_);
    ring_keys_.push_back(makeRingKey(descriptor));
    descriptors_.push_back(std::move(descriptor));
  }

  if (query_frame.idx_ >= descriptors_.size() ||
      query_frame.idx_ <= static_cast<size_t>(config_.exclude_recent_)) {
    return loop_idx_pairs;
  }

  struct Candidate {
    size_t idx;
    double ring_distance;
  };
  std::vector<Candidate> candidates;
  const size_t search_end = query_frame.idx_ - static_cast<size_t>(config_.exclude_recent_);
  for (size_t idx = 0; idx < search_end; ++idx) {
    if (query_frame.timestamp_ - keyframes[idx].timestamp_ < config_.min_time_separation_) continue;
    candidates.push_back({idx, (ring_keys_[query_frame.idx_] - ring_keys_[idx]).norm()});
  }
  if (candidates.empty()) return loop_idx_pairs;

  const size_t candidate_count =
      std::min(candidates.size(), static_cast<size_t>(std::max(1, config_.num_candidates_)));
  std::partial_sort(candidates.begin(),
                    candidates.begin() + candidate_count,
                    candidates.end(),
                    [](const Candidate &lhs, const Candidate &rhs) {
                      return lhs.ring_distance < rhs.ring_distance;
                    });

  size_t best_idx = std::numeric_limits<size_t>::max();
  double best_distance = std::numeric_limits<double>::max();
  for (size_t i = 0; i < candidate_count; ++i) {
    const auto idx = candidates[i].idx;
    const double distance = descriptorDistance(descriptors_[query_frame.idx_], descriptors_[idx]);
    if (distance < best_distance) {
      best_distance = distance;
      best_idx = idx;
    }
  }

  if (best_idx != std::numeric_limits<size_t>::max() &&
      best_distance < config_.distance_threshold_) {
    loop_idx_pairs.emplace_back(query_frame.idx_, best_idx);
    if (config_.verbose_) {
      RCLCPP_INFO(logger_,
                  "Scan Context candidate: query=%zu match=%zu distance=%.3f",
                  query_frame.idx_,
                  best_idx,
                  best_distance);
    }
  }
  return loop_idx_pairs;
}
