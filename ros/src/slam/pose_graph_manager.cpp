#include "slam/pose_graph_manager.h"

#include <filesystem>

using namespace kiss_matcher;

PoseGraphManager::PoseGraphManager(const rclcpp::NodeOptions &options)
    : rclcpp::Node("km_sam", options) {
  double loop_pub_hz;
  double loop_detector_hz;
  double loop_nnsearch_hz;
  bool enable_loop_nnsearch;
  double map_update_hz;
  double vis_hz;

  LoopClosureConfig lc_config;
  kiss_matcher_slam::ScanContextConfig scan_context_config;
  auto &gc = lc_config.gicp_config_;

  map_frame_             = declare_parameter<std::string>("map_frame", "map");
  base_frame_            = declare_parameter<std::string>("base_frame", "base");
  loop_pub_hz            = declare_parameter<double>("loop_pub_hz", 0.1);
  loop_detector_hz       = declare_parameter<double>("loop_detector_hz", 1.0);
  loop_nnsearch_hz       = declare_parameter<double>("loop_nnsearch_hz", 1.0);
  enable_loop_nnsearch   = declare_parameter<bool>("loop.enable_nnsearch", false);
  loop_pub_delayed_time_ = declare_parameter<double>("loop_pub_delayed_time", 60.0);
  map_update_hz          = declare_parameter<double>("map_update_hz", 0.2);
  vis_hz                 = declare_parameter<double>("vis_hz", 0.5);

  store_voxelized_scan_            = declare_parameter<bool>("store_voxelized_scan", false);
  lc_config.voxel_res_             = declare_parameter<double>("voxel_resolution", 0.3);
  lc_config.matcher_config_ = KISSMatcherConfig(lc_config.voxel_res_, false);
  scan_voxel_res_                  = lc_config.voxel_res_;
  map_voxel_res_                   = declare_parameter<double>("map_voxel_resolution", 1.0);
  save_voxel_res_                  = declare_parameter<double>("save_voxel_resolution", 0.3);
  kiss_matcher_slam::KeyframeSelectorConfig keyframe_config;
  keyframe_config.translation_threshold_m =
      declare_parameter<double>("keyframe.keyframe_threshold", 1.0);
  keyframe_config.rotation_threshold_rad =
      declare_parameter<double>("keyframe.rotation_threshold_deg", 10.0) * M_PI / 180.0;
  keyframe_config.minimum_time_interval_s =
      declare_parameter<double>("keyframe.minimum_time_interval", 0.0);
  keyframe_selector_ = kiss_matcher_slam::KeyframeSelector(keyframe_config);
  lc_config.num_submap_keyframes_  = declare_parameter<int>("keyframe.num_submap_keyframes", 5);
  lc_config.verbose_               = declare_parameter<bool>("loop.verbose", false);
  lc_config.is_multilayer_env_     = declare_parameter<bool>("loop.is_multilayer_env", false);
  lc_config.loop_detection_radius_ = declare_parameter<double>("loop.loop_detection_radius", 15.0);
  lc_config.loop_detection_timediff_threshold_ =
      declare_parameter<double>("loop.loop_detection_timediff_threshold", 10.0);

  gc.num_threads_               = declare_parameter<int>("local_reg.num_threads", 8);
  gc.correspondence_randomness_ = declare_parameter<int>("local_reg.correspondences_number", 20);
  gc.max_num_iter_              = declare_parameter<int>("local_reg.max_num_iter", 32);
  gc.scale_factor_for_corr_dist_ =
      declare_parameter<double>("local_reg.scale_factor_for_corr_dist", 5.0);
  gc.overlap_threshold_ = declare_parameter<double>("local_reg.overlap_threshold", 90.0);

  lc_config.enable_global_registration_ = declare_parameter<bool>("global_reg.enable", false);
  lc_config.num_inliers_threshold_ =
      declare_parameter<int>("global_reg.num_inliers_threshold", 100);

  scan_context_verbose_ = declare_parameter<bool>("scan_context.verbose", false);
  scan_context_enabled_ = declare_parameter<bool>("scan_context.enable", true);
  scan_context_config.num_rings = declare_parameter<int>("scan_context.num_rings", 20);
  scan_context_config.num_sectors = declare_parameter<int>("scan_context.num_sectors", 60);
  scan_context_config.exclude_recent = declare_parameter<int>("scan_context.exclude_recent", 30);
  scan_context_config.num_candidates = declare_parameter<int>("scan_context.num_candidates", 10);
  scan_context_config.max_radius_m = declare_parameter<double>("scan_context.max_radius", 80.0);
  scan_context_config.distance_threshold =
      declare_parameter<double>("scan_context.distance_threshold", 0.18);
  scan_context_config.minimum_time_separation_s =
      declare_parameter<double>("scan_context.min_time_separation", 30.0);

  prior_variances_ = declare_parameter<std::vector<double>>(
      "pose_graph.prior_variances", {1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2});
  odom_variances_ = declare_parameter<std::vector<double>>(
      "pose_graph.odom_variances", {1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2});
  loop_variances_ = declare_parameter<std::vector<double>>(
      "pose_graph.loop_variances", {1e-3, 1e-3, 1e-3, 1e-2, 1e-2, 1e-2});
  loop_huber_k_ = declare_parameter<double>("pose_graph.loop_huber_k", 1.345);
  adaptive_loop_noise_ = declare_parameter<bool>("pose_graph.adaptive_loop_noise", true);

  use_gps_altitude_ = declare_parameter<bool>("gps_altitude.enable", false);
  gps_time_tolerance_ = declare_parameter<double>("gps_altitude.time_tolerance", 0.2);
  gps_xy_variance_ = declare_parameter<double>("gps_altitude.xy_variance", 1e8);
  gps_z_variance_ = declare_parameter<double>("gps_altitude.z_variance", 4.0);
  gps_huber_k_ = declare_parameter<double>("gps_altitude.huber_k", 1.345);

  save_map_bag_         = declare_parameter<bool>("result.save_map_bag", false);
  save_map_pcd_         = declare_parameter<bool>("result.save_map_pcd", false);
  save_in_kitti_format_ = declare_parameter<bool>("result.save_in_kitti_format", false);
  seq_name_             = declare_parameter<std::string>("result.seq_name", "sequence");
  output_directory_ = declare_parameter<std::string>("result.output_directory", "maps");
  save_keyframes_incrementally_ =
      declare_parameter<bool>("result.save_keyframes_incrementally", false);
  overwrite_existing_product_ =
      declare_parameter<bool>("result.overwrite_existing", false);

  rclcpp::QoS qos(1);
  qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  loop_closure_          = std::make_shared<LoopClosure>(lc_config, this->get_logger());
  loop_detection_radius_ = lc_config.loop_detection_radius_;

  scan_context_detector_ =
      std::make_unique<kiss_matcher_slam::ScanContextLoopDetector>(scan_context_config);

  odom_path_.header.frame_id      = map_frame_;
  corrected_path_.header.frame_id = map_frame_;

  // NOTE(hlim): To make this node compatible with being launched under different namespaces,
  // I deliberately avoided adding a '/' in front of the topic names.
  path_pub_           = this->create_publisher<nav_msgs::msg::Path>("path/original", qos);
  corrected_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("path/corrected", qos);
  map_pub_            = this->create_publisher<sensor_msgs::msg::PointCloud2>("global_map", qos);
  scan_pub_           = this->create_publisher<sensor_msgs::msg::PointCloud2>("curr_scan", qos);
  loop_detection_pub_ =
      this->create_publisher<visualization_msgs::msg::Marker>("loop_detection", qos);
  loop_detection_radius_pub_ =
      this->create_publisher<visualization_msgs::msg::Marker>("loop_detection_radius", qos);

  // loop_closures_pub_ =
  // this->create_publisher<pose_graph_tools_msgs::msg::PoseGraph>("/hydra_ros_node/external_loop_closures",
  // 10);
  realtime_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose_stamped", qos);
  optimized_odom_pub_ =
      this->create_publisher<nav_msgs::msg::Odometry>("odometry/optimized", qos);
  debug_src_pub_     = this->create_publisher<sensor_msgs::msg::PointCloud2>("lc/src", qos);
  debug_tgt_pub_     = this->create_publisher<sensor_msgs::msg::PointCloud2>("lc/tgt", qos);
  debug_coarse_aligned_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("lc/coarse_alignment", qos);
  debug_fine_aligned_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("lc/fine_alignment", qos);
  debug_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("lc/debug_cloud", qos);

  sub_odom_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(this, "/odom");
  sub_scan_ =
      std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, "/cloud");

  sub_node_ = std::make_shared<message_filters::Synchronizer<NodeSyncPolicy>>(
      NodeSyncPolicy(10), *sub_odom_, *sub_scan_);
  sub_node_->registerCallback(std::bind(
      &PoseGraphManager::callbackNode, this, std::placeholders::_1, std::placeholders::_2));

  sub_save_flag_ = this->create_subscription<std_msgs::msg::String>(
      "save_dir", 1, std::bind(&PoseGraphManager::saveFlagCallback, this, std::placeholders::_1));
  if (use_gps_altitude_) {
    sub_gps_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "gps/fix", rclcpp::SensorDataQoS(),
        std::bind(&PoseGraphManager::gpsCallback, this, std::placeholders::_1));
  }

  kiss_matcher_slam::MapProductConfig product_config;
  product_config.output_directory = output_directory_;
  product_config.sequence = seq_name_;
  product_config.map_frame = map_frame_;
  product_config.base_frame = base_frame_;
  product_config.save_voxel_resolution = save_voxel_res_;
  product_config.keyframe_translation_threshold_m =
      keyframe_selector_.config().translation_threshold_m;
  product_config.keyframe_rotation_threshold_rad =
      keyframe_selector_.config().rotation_threshold_rad;
  product_config.gps_altitude_enabled = use_gps_altitude_;
  product_config.save_keyframes_incrementally = save_keyframes_incrementally_;
  product_config.overwrite_existing = overwrite_existing_product_;
  map_product_writer_ =
      std::make_unique<kiss_matcher_slam::MapProductWriter>(std::move(product_config));
  map_product_writer_->initialize();

  // hydra_loop_timer_ = this->create_wall_timer(
  //   std::chrono::duration<double>(1.0 / loop_pub_hz),
  //   std::bind(&PoseGraphManager::loopPubTimerFunc, this));

  map_cloud_.reset(new pcl::PointCloud<PointType>());
  map_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / map_update_hz),
                                       std::bind(&PoseGraphManager::buildMap, this));

  scan_context_timer_ =
      this->create_wall_timer(std::chrono::duration<double>(1.0 / loop_detector_hz),
                              std::bind(&PoseGraphManager::detectLoopClosureByScanContext, this));

  if (enable_loop_nnsearch) {
    loop_nnsearch_timer_ =
        this->create_wall_timer(std::chrono::duration<double>(1.0 / loop_nnsearch_hz),
                                std::bind(&PoseGraphManager::detectLoopClosureByNNSearch, this));
  }

  graph_vis_timer_ =
      this->create_wall_timer(std::chrono::duration<double>(1.0 / vis_hz),
                              std::bind(&PoseGraphManager::visualizePoseGraph, this));

  lc_reg_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / 100.0),
                                          std::bind(&PoseGraphManager::performRegistration, this));

  // 20 Hz is enough as long as it's faster than the full registration process.
  lc_vis_timer_ =
      this->create_wall_timer(std::chrono::duration<double>(1.0 / 20.0),
                              std::bind(&PoseGraphManager::visualizeLoopClosureClouds, this));

  if (!lc_config.is_multilayer_env_) {
    RCLCPP_WARN(
        get_logger(),
        "'loop.is_multilayer_env' is set to `false`. "
        "This setting is recommended for outdoor environments to ignore the effect of Z-drift. "
        "However, if you're running SLAM in an indoor multi-layer environment, "
        "consider setting it to true to enable full 3D NN search for loop candidates.");
  }
  RCLCPP_INFO(this->get_logger(), "Main class, starting node...");
}

PoseGraphManager::~PoseGraphManager() {
  if (save_map_bag_) {
    RCLCPP_INFO(this->get_logger(), "NOTE(hlim): skipping final bag save in ROS2 example code.");
  }
  const auto keyframes = map_database_.snapshot();
  if (save_map_pcd_ && !keyframes.empty()) {
    try {
      map_product_writer_->writeMap(keyframes, true);
      writeTrajectoryProducts();
      RCLCPP_INFO(this->get_logger(),
                  "Mapping data product saved to %s",
                  map_product_writer_->sequenceDirectory().c_str());
    } catch (const std::exception &error) {
      RCLCPP_ERROR(this->get_logger(), "Failed to save mapping data product: %s", error.what());
    }
  }
}

void PoseGraphManager::gpsCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {
  if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX ||
      !std::isfinite(msg->altitude)) {
    return;
  }
  std::lock_guard<std::mutex> lock(gps_mutex_);
  latest_gps_ = msg;
}

void PoseGraphManager::saveKeyframeIncrementally(const PoseGraphNode &node) {
  map_product_writer_->appendKeyframe(node);
}

void PoseGraphManager::writeTrajectoryProducts() {
  const auto estimates = pose_graph_backend_.estimates();
  if (estimates.empty()) return;
  const auto keyframes = map_database_.snapshot();
  std::vector<Eigen::Matrix4d> optimized_poses;
  optimized_poses.reserve(estimates.size());
  for (size_t i = 0; i < estimates.size(); ++i) {
    optimized_poses.push_back(gtsamToEigen(estimates.at<gtsam::Pose3>(i)));
  }
  map_product_writer_->writeOptimizedTrajectory(keyframes, optimized_poses);
}

void PoseGraphManager::appendKeyframePose(const PoseGraphNode &node) {
  odoms_.points.emplace_back(node.pose_(0, 3), node.pose_(1, 3), node.pose_(2, 3));

  corrected_odoms_.points.emplace_back(
      node.pose_corrected_(0, 3), node.pose_corrected_(1, 3), node.pose_corrected_(2, 3));

  odom_path_.poses.emplace_back(eigenToPoseStamped(node.pose_, map_frame_));
  corrected_path_.poses.emplace_back(eigenToPoseStamped(node.pose_corrected_, map_frame_));
  return;
}

void PoseGraphManager::callbackNode(const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg,
                                    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &scan_msg) {
  static size_t latest_keyframe_idx = 0;

  // NOTE(hlim): For clarification, 'current' refers to the real-time incoming messages,
  // while 'latest' indicates the last keyframe stored in map_database_.
  Eigen::Matrix4d current_odom = current_frame_.pose_;
  current_frame_               = PoseGraphNode(
      *odom_msg, *scan_msg, latest_keyframe_idx, scan_voxel_res_, store_voxelized_scan_);

  kiss_matcher::TicToc total_timer;
  kiss_matcher::TicToc local_timer;

  visualizeCurrentData(current_odom, odom_msg->header.stamp, scan_msg->header.frame_id);

  if (!is_initialized_) {
    map_database_.append(current_frame_);
    appendKeyframePose(current_frame_);
    saveKeyframeIncrementally(current_frame_);

    pose_graph_backend_.addPrior(
        latest_keyframe_idx, eigenToGtsam(current_frame_.pose_), prior_variances_);
    ++latest_keyframe_idx;
    is_initialized_ = true;

    RCLCPP_INFO(this->get_logger(), "The first node comes. Initialization complete.");

  } else {
    const auto t_keyframe_processing = local_timer.toc();
    const auto latest_keyframe = map_database_.latest();
    if (keyframe_selector_.shouldCreate(current_frame_.pose_corrected_,
                                        current_frame_.timestamp_,
                                        latest_keyframe.optimized_pose,
                                        latest_keyframe.timestamp)) {
      map_database_.append(current_frame_);
      saveKeyframeIncrementally(current_frame_);

      gtsam::Pose3 pose_from = eigenToGtsam(latest_keyframe.optimized_pose);
      gtsam::Pose3 pose_to   = eigenToGtsam(current_frame_.pose_corrected_);

      pose_graph_backend_.addOdometry(latest_keyframe_idx - 1,
                                      latest_keyframe_idx,
                                      pose_from.between(pose_to),
                                      pose_to,
                                      odom_variances_);

      if (use_gps_altitude_) {
        sensor_msgs::msg::NavSatFix::ConstSharedPtr gps;
        {
          std::lock_guard<std::mutex> gps_lock(gps_mutex_);
          gps = latest_gps_;
        }
        if (gps) {
          const double gps_stamp = rclcpp::Time(gps->header.stamp).seconds();
          if (std::abs(gps_stamp - current_frame_.timestamp_) <= gps_time_tolerance_) {
            if (!gps_origin_initialized_) {
              gps_altitude_origin_ = gps->altitude - pose_to.z();
              gps_origin_initialized_ = true;
            }
            gtsam::Vector3 gps_variances;
            gps_variances << gps_xy_variance_, gps_xy_variance_, gps_z_variance_;
            const gtsam::Point3 measurement(
                pose_to.x(), pose_to.y(), gps->altitude - gps_altitude_origin_);
            pose_graph_backend_.addGps(
                latest_keyframe_idx, measurement, gps_variances, gps_huber_k_);
          }
        }
      }

      ++latest_keyframe_idx;
      {
        std::lock_guard<std::mutex> lock(vis_mutex_);
        appendKeyframePose(current_frame_);
      }

      local_timer.tic();
      const auto optimization = pose_graph_backend_.optimize();
      const auto t_optim = local_timer.toc();

      {
        std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
        last_corrected_pose_ =
            gtsamToEigen(optimization.estimates.at<gtsam::Pose3>(optimization.estimates.size() - 1));
        odom_delta_ = Eigen::Matrix4d::Identity();
      }
      if (optimization.loop_constraints_applied) {
        std::vector<kiss_matcher_slam::MapDatabase::PoseUpdate> pose_updates;
        pose_updates.reserve(optimization.estimates.size());
        for (size_t i = 0; i < optimization.estimates.size(); ++i) {
          pose_updates.emplace_back(
              i, gtsamToEigen(optimization.estimates.at<gtsam::Pose3>(i)));
        }
        if (!map_database_.updateOptimizedPoses(pose_updates)) {
          throw std::runtime_error("pose graph result does not match the map database");
        }
      }
      writeTrajectoryProducts();

      const auto t_total = total_timer.toc();

      RCLCPP_INFO(
          this->get_logger(),
          "# of Keyframes: %zu. Timing (msec) → Total: %.1f | Keyframe: %.1f | Optim.: %.1f",
          map_database_.size(),
          t_total,
          t_keyframe_processing,
          t_optim);
    }
  }
}

// void PoseGraphManager::loopPubTimerFunc()
// {
//   if (loop_msgs_.edges.empty()) {
//     RCLCPP_WARN(this->get_logger(),
//       "`loop_msgs_.edges` is empty. Skipping loop closure publishing.");
//     return;
//   }
//   if (last_lc_time_ + loop_pub_delayed_time_ < this->now().seconds()) {
//     loop_closures_pub_->publish(loop_msgs_);
//     loop_msgs_.nodes.clear();
//     loop_msgs_.edges.clear();
//     RCLCPP_INFO(this->get_logger(), "`loop_msgs_` is successfully published!");
//     return;
//   }
// }

void PoseGraphManager::buildMap() {
  static size_t start_idx = 0;

  if (map_pub_->get_subscription_count() > 0) {
    if (need_map_update_) {
      map_cloud_->clear();
      start_idx = 0;
    }

    auto keyframes = map_database_.snapshot();
    if (keyframes.empty()) return;

    // NOTE(hlim): Building the full map causes RViz delay when keyframes > 500.
    // Since the map is for visualization only, we apply a heuristic to reduce cost.
    for (size_t i = start_idx; i < keyframes.size(); ++i) {
      if (!store_voxelized_scan_ && keyframes[i].visualization_cloud.empty()) {
        keyframes[i].visualization_cloud = *voxelize(keyframes[i].cloud, scan_voxel_res_);
        if (!map_database_.setVisualizationCloud(i, keyframes[i].visualization_cloud)) {
          throw std::runtime_error("failed to cache a keyframe visualization cloud");
        }
      }
      const auto &scan = store_voxelized_scan_ ? keyframes[i].cloud
                                                : keyframes[i].visualization_cloud;
      *map_cloud_ += transformPcd(scan, keyframes[i].optimized_pose);
    }
    start_idx = keyframes.size();

    const auto &voxelized_map = voxelize(map_cloud_, map_voxel_res_);
    map_pub_->publish(
        toROSMsg(*voxelized_map, map_frame_, toRclcppTime(keyframes.back().timestamp)));
  }

  if (need_map_update_) {
    need_map_update_ = false;
  }
}

void PoseGraphManager::detectLoopClosureByScanContext() {
  if (!scan_context_enabled_ || !is_initialized_ || map_database_.empty()) return;
  const auto claimed_id = map_database_.claimLatestForPlaceRecognition();
  if (!claimed_id) return;

  auto stored_keyframes = map_database_.snapshot();
  stored_keyframes.resize(*claimed_id + 1);
  const auto &query = stored_keyframes.back();

  const auto candidate = scan_context_detector_->detect(query, stored_keyframes);
  if (candidate) {
    loop_idx_pair_queue_.emplace(query.id, candidate->keyframe_id);
    if (scan_context_verbose_) {
      RCLCPP_INFO(get_logger(),
                  "Scan Context candidate: query=%zu match=%zu distance=%.3f yaw=%.3f rad",
                  query.id,
                  candidate->keyframe_id,
                  candidate->descriptor_distance,
                  candidate->yaw_offset_rad);
    }
  }
}

void PoseGraphManager::detectLoopClosureByNNSearch() {
  if (!is_initialized_ || map_database_.empty()) return;
  const auto claimed_id = map_database_.claimLatestForProximitySearch();
  if (!claimed_id) return;

  auto stored_keyframes = map_database_.snapshot();
  stored_keyframes.resize(*claimed_id + 1);
  const auto &query = stored_keyframes.back();

  const auto &loop_idx_pairs = loop_closure_->fetchLoopCandidates(query, stored_keyframes);

  for (const auto &loop_candidate : loop_idx_pairs) {
    loop_idx_pair_queue_.push(loop_candidate);
  }
}

void PoseGraphManager::performRegistration() {
  kiss_matcher::TicToc reg_timer;
  if (loop_idx_pair_queue_.empty()) {
    return;
  }
  const auto [query_idx, match_idx] = loop_idx_pair_queue_.front();
  loop_idx_pair_queue_.pop();

  const auto stored_keyframes = map_database_.snapshot();
  if (query_idx >= stored_keyframes.size() || match_idx >= stored_keyframes.size()) {
    RCLCPP_ERROR(get_logger(), "Discarding loop candidate with an invalid keyframe index");
    return;
  }
  const RegOutput &reg_output =
      loop_closure_->performLoopClosure(stored_keyframes, query_idx, match_idx);

  if (reg_output.is_valid_) {
    RCLCPP_INFO(this->get_logger(), "LC accepted. Overlapness: %.3f", reg_output.overlapness_);
    gtsam::Pose3 pose_from =
        eigenToGtsam(reg_output.pose_ * stored_keyframes[query_idx].optimized_pose);
    gtsam::Pose3 pose_to = eigenToGtsam(stored_keyframes[match_idx].optimized_pose);

    std::vector<double> loop_variances = loop_variances_;
    if (adaptive_loop_noise_) {
      const double confidence = std::clamp(reg_output.overlapness_ / 100.0, 0.1, 1.0);
      const double scale = 1.0 / (confidence * confidence);
      for (auto &variance : loop_variances) variance *= scale;
    }
    pose_graph_backend_.addLoop(
        query_idx, match_idx, pose_from.between(pose_to), loop_variances, loop_huber_k_);

    vis_loop_edges_.emplace_back(query_idx, match_idx);
    succeeded_query_idx_      = query_idx;
    need_lc_cloud_vis_update_ = true;
    need_map_update_          = true;
    need_graph_vis_update_    = true;

    // --------------------------------------------------
    // TODO(hlim): resurrect pose_graph_tools_msgs
    // pose_graph_tools_msgs::msg::PoseGraphEdge edge;
    // double lidar_end_time_compensation = 0.1;
    // edge.header.stamp = this->now();
    // edge.robot_from = 0;
    // edge.robot_to = 0;
    // edge.type = 1;

    // edge.key_to = static_cast<uint64_t>(
    //   (keyframes_.back().timestamp_ - lidar_end_time_compensation) * 1e9);
    // edge.key_from = static_cast<uint64_t>(
    //   (keyframes_[closest_keyframe_idx].timestamp_ - lidar_end_time_compensation) * 1e9);

    // Eigen::Matrix4d pose_inv = pose_to.matrix().inverse() * pose_from.matrix();
    // edge.pose = poseEigToPoseGeo(pose_inv);
    // loop_msgs_.edges.emplace_back(edge);
    // last_lc_time_ = this->now().seconds();
    // --------------------------------------------------
  } else {
    if (reg_output.overlapness_ == 0.0) {
      RCLCPP_WARN(this->get_logger(), "LC rejected. KISS-Matcher failed");
    } else {
      RCLCPP_WARN(this->get_logger(), "LC rejected. Overlapness: %.3f", reg_output.overlapness_);
    }
  }
  RCLCPP_INFO(this->get_logger(), "Reg: %.1f msec", reg_timer.toc());
}

void PoseGraphManager::visualizeCurrentData(const Eigen::Matrix4d &current_odom,
                                            const rclcpp::Time &timestamp,
                                            const std::string &frame_id) {
  // NOTE(hlim): Instead of visualizing only when adding keyframes (node-wise), which can feel
  // choppy, we visualize the current frame every cycle to ensure smoother, real-time visualization.
  {
    std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
    odom_delta_                    = odom_delta_ * current_odom.inverse() * current_frame_.pose_;
    current_frame_.pose_corrected_ = last_corrected_pose_ * odom_delta_;

    geometry_msgs::msg::PoseStamped ps =
        eigenToPoseStamped(current_frame_.pose_corrected_, map_frame_);
    realtime_pose_pub_->publish(ps);

    nav_msgs::msg::Odometry optimized_odom;
    optimized_odom.header = ps.header;
    optimized_odom.child_frame_id = base_frame_.empty() ? frame_id : base_frame_;
    optimized_odom.pose.pose = ps.pose;
    optimized_odom_pub_->publish(optimized_odom);

    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp    = timestamp;
    transform_stamped.header.frame_id = map_frame_;
    transform_stamped.child_frame_id  = base_frame_.empty() ? frame_id : base_frame_;
    Eigen::Quaterniond q(current_frame_.pose_corrected_.block<3, 3>(0, 0));
    transform_stamped.transform.translation.x = current_frame_.pose_corrected_(0, 3);
    transform_stamped.transform.translation.y = current_frame_.pose_corrected_(1, 3);
    transform_stamped.transform.translation.z = current_frame_.pose_corrected_(2, 3);
    transform_stamped.transform.rotation.x    = q.x();
    transform_stamped.transform.rotation.y    = q.y();
    transform_stamped.transform.rotation.z    = q.z();
    transform_stamped.transform.rotation.w    = q.w();
    tf_broadcaster_->sendTransform(transform_stamped);
  }

  scan_pub_->publish(toROSMsg(
      transformPcd(current_frame_.scan_, current_frame_.pose_corrected_), map_frame_, timestamp));
  if (!corrected_path_.poses.empty()) {
    loop_detection_radius_pub_->publish(
        visualizeLoopDetectionRadius(corrected_path_.poses.back().pose.position));
  }
}

void PoseGraphManager::visualizePoseGraph() {
  if (!is_initialized_) {
    return;
  }

  if (need_graph_vis_update_) {
    gtsam::Values corrected_esti_copied;
    pcl::PointCloud<pcl::PointXYZ> corrected_odoms;
    nav_msgs::msg::Path corrected_path;

    corrected_esti_copied = pose_graph_backend_.estimates();
    for (size_t i = 0; i < corrected_esti_copied.size(); ++i) {
      gtsam::Pose3 pose_ = corrected_esti_copied.at<gtsam::Pose3>(i);
      corrected_odoms.points.emplace_back(
          pose_.translation().x(), pose_.translation().y(), pose_.translation().z());

      corrected_path.poses.push_back(gtsamToPoseStamped(pose_, map_frame_));
    }
    if (!vis_loop_edges_.empty()) {
      loop_detection_pub_->publish(visualizeLoopMarkers(corrected_esti_copied));
    }
    {
      std::lock_guard<std::mutex> lock(vis_mutex_);
      corrected_odoms_      = corrected_odoms;
      corrected_path_.poses = corrected_path.poses;
    }
    need_graph_vis_update_ = false;
  }

  {
    std::lock_guard<std::mutex> lock(vis_mutex_);
    path_pub_->publish(odom_path_);
    corrected_path_pub_->publish(corrected_path_);
  }
}

void PoseGraphManager::visualizeLoopClosureClouds() {
  if (!need_lc_cloud_vis_update_) {
    return;
  }

  const auto keyframes = map_database_.snapshot();
  if (succeeded_query_idx_ >= keyframes.size()) {
    RCLCPP_ERROR(this->get_logger(),
                 "Invalid successful loop-closure keyframe index: %zu (keyframe count: %zu)",
                 succeeded_query_idx_,
                 keyframes.size());
    need_lc_cloud_vis_update_ = false;
    return;
  }
  const auto query_timestamp = toRclcppTime(keyframes[succeeded_query_idx_].timestamp);

  debug_src_pub_->publish(toROSMsg(loop_closure_->getSourceCloud(), map_frame_, query_timestamp));
  debug_tgt_pub_->publish(toROSMsg(loop_closure_->getTargetCloud(), map_frame_, query_timestamp));
  debug_fine_aligned_pub_->publish(
      toROSMsg(loop_closure_->getFinalAlignedCloud(), map_frame_, query_timestamp));
  debug_coarse_aligned_pub_->publish(
      toROSMsg(loop_closure_->getCoarseAlignedCloud(), map_frame_, query_timestamp));
  debug_cloud_pub_->publish(toROSMsg(loop_closure_->getDebugCloud(), map_frame_, query_timestamp));
  need_lc_cloud_vis_update_ = false;
}

visualization_msgs::msg::Marker PoseGraphManager::visualizeLoopMarkers(
    const gtsam::Values &corrected_poses) const {
  visualization_msgs::msg::Marker edges;
  edges.type               = visualization_msgs::msg::Marker::LINE_LIST;
  edges.scale.x            = 0.12f;
  edges.header.frame_id    = map_frame_;
  edges.pose.orientation.w = 1.0f;
  edges.color.r            = 1.0f;
  edges.color.g            = 1.0f;
  edges.color.b            = 1.0f;
  edges.color.a            = 1.0f;

  for (size_t i = 0; i < vis_loop_edges_.size(); ++i) {
    if (vis_loop_edges_[i].first >= corrected_poses.size() ||
        vis_loop_edges_[i].second >= corrected_poses.size()) {
      continue;
    }
    gtsam::Pose3 pose  = corrected_poses.at<gtsam::Pose3>(vis_loop_edges_[i].first);
    gtsam::Pose3 pose2 = corrected_poses.at<gtsam::Pose3>(vis_loop_edges_[i].second);

    geometry_msgs::msg::Point p, p2;
    p.x  = pose.translation().x();
    p.y  = pose.translation().y();
    p.z  = pose.translation().z();
    p2.x = pose2.translation().x();
    p2.y = pose2.translation().y();
    p2.z = pose2.translation().z();

    edges.points.push_back(p);
    edges.points.push_back(p2);
  }
  return edges;
}

visualization_msgs::msg::Marker PoseGraphManager::visualizeLoopDetectionRadius(
    const geometry_msgs::msg::Point &latest_position) const {
  visualization_msgs::msg::Marker sphere;
  sphere.header.frame_id = map_frame_;
  sphere.id              = 100000;  // arbitrary number
  sphere.type            = visualization_msgs::msg::Marker::SPHERE;
  sphere.pose.position.x = latest_position.x;
  sphere.pose.position.y = latest_position.y;
  sphere.pose.position.z = latest_position.z;
  sphere.scale.x         = 2 * loop_detection_radius_;
  sphere.scale.y         = 2 * loop_detection_radius_;
  sphere.scale.z         = 2 * loop_detection_radius_;
  // Use transparanet cyan color
  sphere.color.r = 0.0;
  sphere.color.g = 0.824;
  sphere.color.b = 1.0;
  sphere.color.a = 0.5;

  return sphere;
}

void PoseGraphManager::saveFlagCallback(const std_msgs::msg::String::ConstSharedPtr &msg) {
  const std::filesystem::path save_dir =
      !msg->data.empty() ? std::filesystem::path(msg->data)
                         : std::filesystem::path(output_directory_);
  const auto keyframes = map_database_.snapshot();
  if (keyframes.empty()) {
    RCLCPP_WARN(this->get_logger(), "No keyframes available; skipping map product save");
    return;
  }

  try {
    if (save_in_kitti_format_) {
      const auto directory = map_product_writer_->writeKittiDataset(keyframes, save_dir);
      RCLCPP_INFO(this->get_logger(),
                  "Scans and poses saved in PCD, KITTI and TUM format below %s",
                  directory.c_str());
    }
    if (save_map_bag_) {
      RCLCPP_INFO(this->get_logger(),
                  "NOTE(hlim): rosbag2 saving not directly implemented; skipping.");
    }
    if (save_map_pcd_) {
      const auto path = map_product_writer_->writeMap(keyframes, save_dir, false);
      RCLCPP_INFO(this->get_logger(), "Accumulated map cloud saved to %s", path.c_str());
    }
  } catch (const std::exception &error) {
    RCLCPP_ERROR(this->get_logger(),
                 "Map product save request failed without changing the running map: %s",
                 error.what());
  }
}

// ----------------------------------------------------------------------

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  auto node = std::make_shared<PoseGraphManager>(options);

  // To allow timer callbacks to run concurrently using multiple threads
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
