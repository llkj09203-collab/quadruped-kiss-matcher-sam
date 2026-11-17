#pragma once

#include <Eigen/Core>

namespace kiss_matcher_slam {

struct KeyframeSelectorConfig {
  double translation_threshold_m = 1.0;
  double rotation_threshold_rad = 0.17453292519943295;
  double minimum_time_interval_s = 0.0;
};

class KeyframeSelector {
 public:
  explicit KeyframeSelector(KeyframeSelectorConfig config = {});

  bool shouldCreate(const Eigen::Matrix4d &current_pose,
                    double current_timestamp,
                    const Eigen::Matrix4d &previous_pose,
                    double previous_timestamp) const;

  const KeyframeSelectorConfig &config() const { return config_; }

 private:
  KeyframeSelectorConfig config_;
};

}  // namespace kiss_matcher_slam
