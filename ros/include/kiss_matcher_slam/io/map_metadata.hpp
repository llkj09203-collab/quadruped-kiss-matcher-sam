#pragma once

#include <cstdint>
#include <string>

namespace kiss_matcher_slam {

struct MapMetadata {
  std::uint32_t format_version = 1;
  std::string map_id;
  std::string version;
  std::string parent_version;
  std::string map_frame = "map";
  std::string odom_frame = "odom";
  std::string base_frame = "base_link";
  std::string lidar_frame = "livox_frame";
  std::string created_at;
  bool gps_altitude_enabled = false;
};

}  // namespace kiss_matcher_slam
