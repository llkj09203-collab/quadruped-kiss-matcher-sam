#pragma once

#include <mutex>
#include <optional>

#include <Eigen/Core>

namespace kiss_matcher_slam {

struct GpsAltitudeConfig {
  bool enabled = false;
  double time_tolerance_s = 0.2;
  double xy_variance = 1e8;
  double z_variance = 4.0;
  double huber_k = 1.345;
};

struct GpsAltitudeConstraint {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d variances = Eigen::Vector3d::Ones();
  double huber_k = 1.345;
};

// Owns the latest valid altitude sample and the session-local altitude origin.
// Latitude/longitude intentionally remain unused: this is a Z-only stabilizer.
class GpsAltitudeManager {
 public:
  explicit GpsAltitudeManager(GpsAltitudeConfig config);

  void update(double timestamp, double altitude);
  std::optional<GpsAltitudeConstraint> constraint(double keyframe_timestamp,
                                                  const Eigen::Matrix4d &keyframe_pose);
  bool enabled() const { return config_.enabled; }

 private:
  struct Sample {
    double timestamp = 0.0;
    double altitude = 0.0;
  };

  GpsAltitudeConfig config_;
  mutable std::mutex mutex_;
  std::optional<Sample> latest_sample_;
  std::optional<double> altitude_origin_;
};

}  // namespace kiss_matcher_slam
