#pragma once

#ifndef KISS_MATCHER_POSE_GRAPH_NODE_HPP
#define KISS_MATCHER_POSE_GRAPH_NODE_HPP

#include <string>
#include <utility>

#include "kiss_matcher_slam/core/keyframe.hpp"
#include "slam/utils.hpp"

namespace kiss_matcher {

struct PoseGraphNode : public kiss_matcher_slam::Keyframe {
  // Transitional aliases keep the ROS adapter behavior stable while the core
  // algorithms migrate to the shared Keyframe model in later commits.
  pcl::PointCloud<PointType> &scan_ = cloud;
  pcl::PointCloud<PointType> &voxelized_scan_ = visualization_cloud;
  Eigen::Matrix4d &pose_ = odometry_pose;
  Eigen::Matrix4d &pose_corrected_ = optimized_pose;
  double &timestamp_ = timestamp;
  size_t &idx_ = id;
  bool &nnsearch_processed_ = proximity_search_processed;

  PoseGraphNode() = default;
  explicit PoseGraphNode(const kiss_matcher_slam::Keyframe &other)
      : kiss_matcher_slam::Keyframe(other) {}
  PoseGraphNode(const PoseGraphNode &other)
      : kiss_matcher_slam::Keyframe(other) {}
  PoseGraphNode(PoseGraphNode &&other) noexcept
      : kiss_matcher_slam::Keyframe(std::move(other)) {}
  PoseGraphNode &operator=(const PoseGraphNode &other) {
    kiss_matcher_slam::Keyframe::operator=(other);
    return *this;
  }
  PoseGraphNode &operator=(PoseGraphNode &&other) noexcept {
    kiss_matcher_slam::Keyframe::operator=(std::move(other));
    return *this;
  }
  inline PoseGraphNode(const nav_msgs::msg::Odometry &odom,
                       const sensor_msgs::msg::PointCloud2 &scan,
                       const size_t idx,
                       const double voxel_size,
                       const bool store_voxelized_scan = false,
                       const bool is_wrt_lidar_frame   = false) {
    tf2::Quaternion q;
    q.setX(odom.pose.pose.orientation.x);
    q.setY(odom.pose.pose.orientation.y);
    q.setZ(odom.pose.pose.orientation.z);
    q.setW(odom.pose.pose.orientation.w);

    tf2::Matrix3x3 rot_tf(q);
    Eigen::Matrix3d rot;
    matrixTF2ToEigen(rot_tf, rot);

    pose_.block<3, 3>(0, 0) = rot;
    pose_(0, 3)             = odom.pose.pose.position.x;
    pose_(1, 3)             = odom.pose.pose.position.y;
    pose_(2, 3)             = odom.pose.pose.position.z;

    // This will be updated after pose graph optimization
    pose_corrected_ = pose_;

    pcl::PointCloud<PointType> scan_tmp;
    pcl::fromROSMsg(scan, scan_tmp);

    if (store_voxelized_scan) {
      auto voxelized_scan = *voxelize(scan_tmp, voxel_size);
      scan_tmp            = voxelized_scan;
    }
    if (is_wrt_lidar_frame) {
      scan_ = scan_tmp;
    } else {
      pcl::PointCloud<PointType> pcd_wrt_lidar_frame = transformPcd(scan_tmp, pose_.inverse());
      scan_                                          = pcd_wrt_lidar_frame;
    }

    // ROS2 stamp.sec / stamp.nanosec
    timestamp_ = odom.header.stamp.sec + 1e-9 * odom.header.stamp.nanosec;
    idx_       = idx;
  }
};
}  // namespace kiss_matcher

#endif  // KISS_MATCHER_POSE_GRAPH_NODE_HPP
