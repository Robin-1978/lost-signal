#pragma once

#include "robot/runtime/map_provider.hpp"
#include "robot/runtime/sensor_messages.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace sim {

// A small, deterministic known-map localizer for a planar spinning LiDAR.
// It estimates map->base_footprint around an odometric prior; it does not
// build a map, use simulator truth, or associate dynamic actor identities.
struct KnownMapLocalizationConfig {
 double search_translation_m{0.75};
 double search_heading_rad{0.35};
 double translation_step_m{0.05};
 double heading_step_rad{0.035};
 double minimum_overlap{0.30};
 std::size_t minimum_returns{8};
 double sensor_x_m{0.0};
 double sensor_y_m{0.0};
 double sensor_yaw_rad{0.0};
};

struct KnownMapLocalizationResult {
 bool accepted{false};
 PoseObservation pose;
 double score{0.0};
 double overlap{0.0};
 std::size_t usable_returns{0};
 std::string rejection_reason;
};

class KnownMapLocalizer final {
 public:
 explicit KnownMapLocalizer(KnownMapLocalizationConfig config = {})
 : config_(config) {}

 void reset() {}

 KnownMapLocalizationResult localize(
 const Scan& scan, const Vec2& odom_position, double odom_heading_rad,
 const runtime::MapSnapshot& map, double timestamp_s) const;

 private:
 KnownMapLocalizationConfig config_;
};

} // namespace sim
