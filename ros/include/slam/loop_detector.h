#pragma once

#ifndef KISS_MATCHER_LOOP_DETECTOR_H
#define KISS_MATCHER_LOOP_DETECTOR_H

///// C++ common headers
#include <iostream>
#include <limits>
#include <memory>
#include <tuple>
#include <vector>

#include <Eigen/Eigen>
#include <kiss_matcher/KISSMatcher.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <small_gicp/pcl/pcl_point.hpp>
#include <small_gicp/pcl/pcl_point_traits.hpp>
#include <small_gicp/pcl/pcl_registration.hpp>

#include "rclcpp/rclcpp.hpp"
#include "slam/loop_types.hpp"
#include "slam/pose_graph_node.hpp"
#include "slam/utils.hpp"

#define LOOP_CANDIDATE_NOT_FOUND -1

namespace kiss_matcher {

struct LoopDetectorConfig {
  bool verbose_ = false;
  bool enable_ = true;
  int num_rings_ = 20;
  int num_sectors_ = 60;
  int exclude_recent_ = 30;
  int num_candidates_ = 10;
  double max_radius_ = 80.0;
  double distance_threshold_ = 0.18;
  double min_time_separation_ = 30.0;
};

class LoopDetector {
 private:
  LoopDetectorConfig config_;
  rclcpp::Logger logger_;
  std::vector<Eigen::MatrixXf> descriptors_;
  std::vector<Eigen::VectorXf> ring_keys_;

  Eigen::MatrixXf makeDescriptor(const pcl::PointCloud<PointType> &cloud) const;
  Eigen::VectorXf makeRingKey(const Eigen::MatrixXf &descriptor) const;
  double descriptorDistance(const Eigen::MatrixXf &query,
                            const Eigen::MatrixXf &candidate) const;

 public:
  explicit LoopDetector(const LoopDetectorConfig &config, const rclcpp::Logger &logger);
  ~LoopDetector();
  LoopIdxPairs fetchLoopCandidates(const PoseGraphNode &query_frame,
                                   const std::vector<PoseGraphNode> &keyframes);
};
}  // namespace kiss_matcher
#endif  // KISS_MATCHER_LOOP_DETECTOR_H
