#include "kiss_matcher_slam/core/gps_altitude_manager.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace kiss_matcher_slam {

GpsAltitudeManager::GpsAltitudeManager(GpsAltitudeConfig config)
    : config_(std::move(config)) {
  if (!std::isfinite(config_.time_tolerance_s) || config_.time_tolerance_s < 0.0 ||
      !std::isfinite(config_.xy_variance) || config_.xy_variance <= 0.0 ||
      !std::isfinite(config_.z_variance) || config_.z_variance <= 0.0 ||
      !std::isfinite(config_.huber_k) || config_.huber_k <= 0.0) {
    throw std::invalid_argument("invalid GPS altitude configuration");
  }
}

void GpsAltitudeManager::update(const double timestamp, const double altitude) {
  if (!config_.enabled) return;
  if (!std::isfinite(timestamp) || !std::isfinite(altitude)) {
    throw std::invalid_argument("GPS altitude sample must be finite");
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (latest_sample_ && timestamp < latest_sample_->timestamp) return;
  latest_sample_ = Sample{timestamp, altitude};
}

std::optional<GpsAltitudeConstraint> GpsAltitudeManager::constraint(
    const double keyframe_timestamp,
    const Eigen::Matrix4d &keyframe_pose) {
  if (!config_.enabled) return std::nullopt;
  if (!std::isfinite(keyframe_timestamp) || !keyframe_pose.allFinite()) {
    throw std::invalid_argument("keyframe data for GPS altitude must be finite");
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (!latest_sample_ ||
      std::abs(latest_sample_->timestamp - keyframe_timestamp) > config_.time_tolerance_s) {
    return std::nullopt;
  }
  if (!altitude_origin_) altitude_origin_ = latest_sample_->altitude - keyframe_pose(2, 3);

  GpsAltitudeConstraint result;
  result.position = Eigen::Vector3d(
      keyframe_pose(0, 3), keyframe_pose(1, 3), latest_sample_->altitude - *altitude_origin_);
  result.variances =
      Eigen::Vector3d(config_.xy_variance, config_.xy_variance, config_.z_variance);
  result.huber_k = config_.huber_k;
  return result;
}

}  // namespace kiss_matcher_slam
