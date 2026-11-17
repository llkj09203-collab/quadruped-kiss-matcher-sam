#pragma once

#include <cstddef>
#include <mutex>
#include <vector>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

namespace kiss_matcher_slam {

struct PoseGraphBackendConfig {
  double relinearization_threshold = 0.01;
  int relinearization_skip = 1;
  int extra_loop_updates = 3;
};

struct PoseGraphOptimizationResult {
  gtsam::Values estimates;
  bool loop_constraints_applied = false;
};

// Owns the complete incremental GTSAM state. Constraint producers may run
// from different ROS callbacks; all graph mutation and optimization is
// serialized internally.
class PoseGraphBackend {
 public:
  explicit PoseGraphBackend(PoseGraphBackendConfig config = {});

  void addPrior(std::size_t id,
                const gtsam::Pose3 &pose,
                const std::vector<double> &variances);
  void addOdometry(std::size_t from_id,
                   std::size_t to_id,
                   const gtsam::Pose3 &relative_pose,
                   const gtsam::Pose3 &initial_pose,
                   const std::vector<double> &variances);
  void addLoop(std::size_t from_id,
               std::size_t to_id,
               const gtsam::Pose3 &relative_pose,
               const std::vector<double> &variances,
               double huber_k);
  void addGps(std::size_t id,
              const gtsam::Point3 &position,
              const gtsam::Vector3 &variances,
              double huber_k);

  PoseGraphOptimizationResult optimize();
  gtsam::Values estimates() const;

 private:
  static gtsam::noiseModel::Diagonal::shared_ptr diagonalNoise(
      const std::vector<double> &variances);

  PoseGraphBackendConfig config_;
  mutable std::mutex mutex_;
  gtsam::ISAM2 isam_;
  gtsam::NonlinearFactorGraph pending_graph_;
  gtsam::Values pending_initial_estimates_;
  gtsam::Values estimates_;
  bool pending_loop_constraint_ = false;
};

}  // namespace kiss_matcher_slam
