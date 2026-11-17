#pragma once

#ifndef KISS_MATCHER_POSE_GRAPH_MANAGER_H
#define KISS_MATCHER_POSE_GRAPH_MANAGER_H

#include <chrono>
#include <cmath>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>

// message_filters in ROS2
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

// TF2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
// #include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <tf2_eigen/tf2_eigen.hpp>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/Values.h>

#include "../tictoc.hpp"
#include "kiss_matcher_slam/core/keyframe_selector.hpp"
#include "kiss_matcher_slam/core/map_database.hpp"
#include "kiss_matcher_slam/core/pose_graph_backend.hpp"
#include "kiss_matcher_slam/core/scan_context_loop_detector.hpp"
#include "slam/loop_closure.h"
#include "slam/pose_graph_node.hpp"
#include "slam/utils.hpp"

// #include <pose_graph_tools_msgs/msg/pose_graph.hpp>

namespace fs = std::filesystem;
using namespace std::chrono;
typedef message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry,
                                                        sensor_msgs::msg::PointCloud2>
    NodeSyncPolicy;

class PoseGraphManager : public rclcpp::Node {
 public:
  PoseGraphManager() = delete;
  explicit PoseGraphManager(const rclcpp::NodeOptions &options);
  ~PoseGraphManager();

 private:
  void appendKeyframePose(const kiss_matcher::PoseGraphNode &node);

  void callbackNode(const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg,
                    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &scan_msg);
  /**** Timer functions ****/
  // void loopPubTimerFunc();
  void buildMap();
  void detectLoopClosureByScanContext();
  void detectLoopClosureByNNSearch();

  void visualizeCurrentData(const Eigen::Matrix4d &current_odom,
                            const rclcpp::Time &timestamp,
                            const std::string &frame_id);
  void visualizePoseGraph();

  void performRegistration();

  void visualizeLoopClosureClouds();

  visualization_msgs::msg::Marker visualizeLoopMarkers(const gtsam::Values &corrected_poses) const;
  visualization_msgs::msg::Marker visualizeLoopDetectionRadius(
      const geometry_msgs::msg::Point &latest_position) const;

  void saveFlagCallback(const std_msgs::msg::String::ConstSharedPtr &msg);
  void gpsCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);
  void saveKeyframeIncrementally(const kiss_matcher::PoseGraphNode &node);
  void writeTrajectoryProducts();

  std::string map_frame_;
  std::string base_frame_;
  std::string package_path_;
  std::string seq_name_;
  std::string output_directory_;

  std::mutex realtime_pose_mutex_;
  std::mutex lc_mutex_;
  std::mutex vis_mutex_;
  std::mutex gps_mutex_;

  Eigen::Matrix4d last_corrected_pose_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d odom_delta_          = Eigen::Matrix4d::Identity();
  kiss_matcher::PoseGraphNode current_frame_;
  kiss_matcher_slam::MapDatabase map_database_;

  bool is_initialized_           = false;
  bool need_map_update_          = false;
  bool need_graph_vis_update_    = false;
  bool need_lc_cloud_vis_update_ = false;

  kiss_matcher_slam::PoseGraphBackend pose_graph_backend_;

  kiss_matcher_slam::KeyframeSelector keyframe_selector_;
  double scan_voxel_res_;
  double map_voxel_res_;
  double save_voxel_res_;
  double loop_pub_delayed_time_;
  double loop_detection_radius_;  // Only for visualization
  int sub_key_num_;

  std::vector<double> prior_variances_;
  std::vector<double> odom_variances_;
  std::vector<double> loop_variances_;
  double loop_huber_k_ = 1.345;
  bool adaptive_loop_noise_ = true;

  bool use_gps_altitude_ = false;
  double gps_time_tolerance_ = 0.2;
  double gps_xy_variance_ = 1e8;
  double gps_z_variance_ = 4.0;
  double gps_huber_k_ = 1.345;
  bool gps_origin_initialized_ = false;
  double gps_altitude_origin_ = 0.0;
  sensor_msgs::msg::NavSatFix::ConstSharedPtr latest_gps_;

  size_t succeeded_query_idx_ = 0;
  std::vector<std::pair<size_t, size_t>> vis_loop_edges_;
  // pose_graph_tools_msgs::msg::PoseGraph loop_msgs_;
  std::queue<LoopIdxPair> loop_idx_pair_queue_;

  kiss_matcher::TicToc timer_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  pcl::PointCloud<pcl::PointXYZ> odoms_, corrected_odoms_;
  nav_msgs::msg::Path odom_path_, corrected_path_;

  bool store_voxelized_scan_ = false;

  bool save_map_bag_         = false;
  bool save_map_pcd_         = false;
  bool save_in_kitti_format_ = false;
  bool save_keyframes_incrementally_ = false;
  double last_lc_time_       = 0.0;

  std::shared_ptr<kiss_matcher::LoopClosure> loop_closure_;

  // NOTE(hlim): We do not provide a loop detector implementation directly,
  // but you can plug in your own detector via this interface.
  bool scan_context_enabled_ = true;
  bool scan_context_verbose_ = false;
  std::unique_ptr<kiss_matcher_slam::ScanContextLoopDetector> scan_context_detector_;

  pcl::PointCloud<PointType>::Ptr map_cloud_;

  // ROS2 interface
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr corrected_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr loop_detection_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr loop_detection_radius_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr realtime_pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr optimized_odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_src_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_tgt_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_coarse_aligned_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_fine_aligned_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_cloud_pub_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_save_flag_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps_;

  // rclcpp::Publisher<pose_graph_tools_msgs::msg::PoseGraph>::SharedPtr loop_closures_pub_;

  // message_filters
  std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> sub_odom_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> sub_scan_;
  std::shared_ptr<message_filters::Synchronizer<NodeSyncPolicy>> sub_node_;

  // Timers
  rclcpp::TimerBase::SharedPtr hydra_loop_timer_;
  rclcpp::TimerBase::SharedPtr map_timer_;
  rclcpp::TimerBase::SharedPtr scan_context_timer_;
  rclcpp::TimerBase::SharedPtr loop_nnsearch_timer_;
  rclcpp::TimerBase::SharedPtr graph_vis_timer_;
  rclcpp::TimerBase::SharedPtr lc_reg_timer_;
  rclcpp::TimerBase::SharedPtr lc_vis_timer_;
};

#endif  // KISS_MATCHER_POSE_GRAPH_MANAGER_H
