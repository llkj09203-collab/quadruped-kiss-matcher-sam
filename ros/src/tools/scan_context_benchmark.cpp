#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <pcl/io/pcd_io.h>

#include "kiss_matcher_slam/core/scan_context_loop_detector.hpp"

namespace {

std::vector<double> loadTimestamps(const std::filesystem::path &path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot open timestamps file: " + path.string());
  std::vector<double> timestamps;
  std::size_t id = 0;
  double timestamp = 0.0;
  while (input >> id >> timestamp) {
    if (id != timestamps.size()) {
      throw std::runtime_error("timestamps must use contiguous keyframe IDs");
    }
    timestamps.push_back(timestamp);
  }
  if (timestamps.empty()) throw std::runtime_error("timestamps file is empty");
  return timestamps;
}

std::string keyframeName(const std::size_t id) {
  std::ostringstream name;
  name << std::setw(6) << std::setfill('0') << id << ".pcd";
  return name.str();
}

double percentile(std::vector<double> values, const double ratio) {
  if (values.empty()) return 0.0;
  std::sort(values.begin(), values.end());
  const auto index = static_cast<std::size_t>(ratio * static_cast<double>(values.size() - 1));
  return values[index];
}

}  // namespace

int main(int argc, char **argv) {
  if (argc != 3) {
    std::cerr << "Usage: scan_context_benchmark <map-product-directory> <output.csv>\n";
    return 2;
  }

  try {
    const std::filesystem::path session(argv[1]);
    const std::filesystem::path output_path(argv[2]);
    const auto timestamps = loadTimestamps(session / "timestamps.txt");

    kiss_matcher_slam::ScanContextConfig config;
    config.num_rings = 20;
    config.num_sectors = 60;
    config.exclude_recent = 30;
    config.num_candidates = 10;
    config.max_radius_m = 80.0;
    config.distance_threshold = 0.18;
    config.sector_search_ratio = 0.1;
    config.minimum_time_separation_s = 30.0;
    kiss_matcher_slam::ScanContextLoopDetector detector(config);

    std::ofstream output(output_path);
    if (!output) throw std::runtime_error("cannot create output file: " + output_path.string());
    output << "query_id,match_id,distance,yaw_rad,eligible,shortlisted,comparisons,shift_evaluations,"
              "descriptor_ms,retrieval_ms,total_ms\n";
    output << std::fixed << std::setprecision(9);

    std::vector<kiss_matcher_slam::Keyframe> history;
    history.reserve(timestamps.size());
    std::vector<double> retrieval_times;
    std::size_t detected_loops = 0;
    for (std::size_t id = 0; id < timestamps.size(); ++id) {
      kiss_matcher_slam::Keyframe frame;
      frame.id = id;
      frame.timestamp = timestamps[id];
      const auto cloud_path = session / "keyframes" / keyframeName(id);
      if (pcl::io::loadPCDFile(cloud_path.string(), frame.cloud) != 0) {
        throw std::runtime_error("cannot load keyframe: " + cloud_path.string());
      }
      history.push_back(std::move(frame));

      const auto candidate = detector.detect(history.back(), history);
      const auto &stats = detector.lastStats();
      retrieval_times.push_back(stats.retrieval_ms);
      if (candidate) ++detected_loops;
      output << id << ',';
      if (candidate) {
        output << candidate->keyframe_id << ',' << candidate->descriptor_distance << ','
               << candidate->yaw_offset_rad;
      } else {
        output << "-1,nan,nan";
      }
      output << ',' << stats.eligible_candidates << ',' << stats.shortlisted_candidates << ','
             << stats.full_descriptor_comparisons << ',' << stats.full_shift_evaluations << ','
             << stats.descriptor_build_ms << ',' << stats.retrieval_ms << ',' << stats.total_ms
             << '\n';
    }

    const double mean = std::accumulate(retrieval_times.begin(), retrieval_times.end(), 0.0) /
                        static_cast<double>(retrieval_times.size());
    std::cout << "keyframes=" << history.size() << " detected_loops=" << detected_loops
              << " retrieval_mean_ms=" << mean
              << " retrieval_p50_ms=" << percentile(retrieval_times, 0.50)
              << " retrieval_p95_ms=" << percentile(retrieval_times, 0.95)
              << " retrieval_max_ms=" << percentile(retrieval_times, 1.0) << '\n';
  } catch (const std::exception &error) {
    std::cerr << "scan_context_benchmark: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
