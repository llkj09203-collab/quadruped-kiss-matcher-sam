#pragma once

#include <cstddef>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace kiss_matcher_slam {

using SlamPoint = pcl::PointXYZI;

struct Keyframe {
  std::size_t id = 0;
  double timestamp = 0.0;
  Eigen::Matrix4d odometry_pose = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d optimized_pose = Eigen::Matrix4d::Identity();
  pcl::PointCloud<SlamPoint> cloud;
  pcl::PointCloud<SlamPoint> visualization_cloud;
  bool proximity_search_processed = false;
  bool place_recognition_processed = false;
};

}  // namespace kiss_matcher_slam
