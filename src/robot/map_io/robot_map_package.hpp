#pragma once

#include "robot/runtime/map_provider.hpp"

#include <filesystem>

namespace sim::map_io {

// A Robot Map package is a versioned directory containing a JSON manifest,
// checksummed binary grid layers, and vector semantic features. SLAM session
// state (pose graph, keyframes, raw scans) is intentionally a separate artifact.
void save_robot_map_package(const runtime::MapSnapshot& map,
 const std::filesystem::path& package_path);

runtime::MapSnapshot load_robot_map_package(const std::filesystem::path& package_path,
 runtime::TimePoint publication_time = {});

} // namespace sim::map_io
