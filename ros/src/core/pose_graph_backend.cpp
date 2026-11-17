#include "kiss_matcher_slam/core/pose_graph_backend.hpp"

#include <cmath>
#include <stdexcept>

#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

namespace kiss_matcher_slam {
namespace {

gtsam::ISAM2Params makeIsamParameters(const PoseGraphBackendConfig &config) {
  if (!std::isfinite(config.relinearization_threshold) ||
      config.relinearization_threshold <= 0.0 || config.relinearization_skip <= 0 ||
      config.extra_loop_updates < 0) {
    throw std::invalid_argument("invalid pose-graph backend configuration");
  }
  gtsam::ISAM2Params parameters;
  parameters.relinearizeThreshold = config.relinearization_threshold;
  parameters.relinearizeSkip = config.relinearization_skip;
  return parameters;
}

void validateHuber(const double huber_k) {
  if (!std::isfinite(huber_k) || huber_k <= 0.0) {
    throw std::invalid_argument("Huber parameter must be finite and positive");
  }
}

}  // namespace

PoseGraphBackend::PoseGraphBackend(PoseGraphBackendConfig config)
    : config_(config), isam_(makeIsamParameters(config)) {}

gtsam::noiseModel::Diagonal::shared_ptr PoseGraphBackend::diagonalNoise(
    const std::vector<double> &variances) {
  if (variances.size() != 6) {
    throw std::invalid_argument("pose variance arrays must contain exactly 6 values");
  }
  gtsam::Vector6 values;
  for (std::size_t i = 0; i < 6; ++i) {
    if (!std::isfinite(variances[i]) || variances[i] <= 0.0) {
      throw std::invalid_argument("pose variances must be finite and positive");
    }
    values(static_cast<Eigen::Index>(i)) = variances[i];
  }
  return gtsam::noiseModel::Diagonal::Variances(values);
}

void PoseGraphBackend::addPrior(const std::size_t id,
                                const gtsam::Pose3 &pose,
                                const std::vector<double> &variances) {
  auto noise = diagonalNoise(variances);
  std::lock_guard<std::mutex> lock(mutex_);
  if (pending_initial_estimates_.exists(id) || estimates_.exists(id)) {
    throw std::invalid_argument("pose-graph node already exists");
  }
  pending_graph_.add(gtsam::PriorFactor<gtsam::Pose3>(id, pose, noise));
  pending_initial_estimates_.insert(id, pose);
}

void PoseGraphBackend::addOdometry(const std::size_t from_id,
                                   const std::size_t to_id,
                                   const gtsam::Pose3 &relative_pose,
                                   const gtsam::Pose3 &initial_pose,
                                   const std::vector<double> &variances) {
  if (from_id == to_id) throw std::invalid_argument("odometry edge must connect two nodes");
  auto noise = diagonalNoise(variances);
  std::lock_guard<std::mutex> lock(mutex_);
  if (pending_initial_estimates_.exists(to_id) || estimates_.exists(to_id)) {
    throw std::invalid_argument("pose-graph node already exists");
  }
  pending_graph_.add(
      gtsam::BetweenFactor<gtsam::Pose3>(from_id, to_id, relative_pose, noise));
  pending_initial_estimates_.insert(to_id, initial_pose);
}

void PoseGraphBackend::addLoop(const std::size_t from_id,
                               const std::size_t to_id,
                               const gtsam::Pose3 &relative_pose,
                               const std::vector<double> &variances,
                               const double huber_k) {
  if (from_id == to_id) throw std::invalid_argument("loop edge must connect two nodes");
  validateHuber(huber_k);
  auto noise = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(huber_k), diagonalNoise(variances));
  std::lock_guard<std::mutex> lock(mutex_);
  pending_graph_.add(
      gtsam::BetweenFactor<gtsam::Pose3>(from_id, to_id, relative_pose, noise));
  pending_loop_constraint_ = true;
}

void PoseGraphBackend::addGps(const std::size_t id,
                              const gtsam::Point3 &position,
                              const gtsam::Vector3 &variances,
                              const double huber_k) {
  validateHuber(huber_k);
  for (Eigen::Index i = 0; i < variances.size(); ++i) {
    if (!std::isfinite(variances(i)) || variances(i) <= 0.0) {
      throw std::invalid_argument("GPS variances must be finite and positive");
    }
  }
  auto noise = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(huber_k),
      gtsam::noiseModel::Diagonal::Variances(variances));
  std::lock_guard<std::mutex> lock(mutex_);
  pending_graph_.add(gtsam::GPSFactor(id, position, noise));
}

PoseGraphOptimizationResult PoseGraphBackend::optimize() {
  std::lock_guard<std::mutex> lock(mutex_);
  const bool apply_loop = pending_loop_constraint_;
  isam_.update(pending_graph_, pending_initial_estimates_);
  isam_.update();
  if (apply_loop) {
    for (int i = 0; i < config_.extra_loop_updates; ++i) isam_.update();
  }
  pending_graph_.resize(0);
  pending_initial_estimates_.clear();
  pending_loop_constraint_ = false;
  estimates_ = isam_.calculateEstimate();
  return {estimates_, apply_loop};
}

gtsam::Values PoseGraphBackend::estimates() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return estimates_;
}

}  // namespace kiss_matcher_slam
