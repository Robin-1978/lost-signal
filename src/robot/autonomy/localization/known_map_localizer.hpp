#pragma once

#include "robot/autonomy/localization/localization_provider.hpp"

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
 double minimum_score_margin{0.01};
 std::size_t minimum_returns{8};
 double sensor_x_m{0.0};
 double sensor_y_m{0.0};
 double sensor_yaw_rad{0.0};
};

using KnownMapLocalizationResult = autonomy::LocalizationResult;

class KnownMapLocalizer final : public autonomy::ILocalizationProvider {
 public:
 explicit KnownMapLocalizer(KnownMapLocalizationConfig config = {})
 : config_(config) {}

 void reset() {}

 KnownMapLocalizationResult localize(
 const Scan& scan, const Vec2& odom_position, double odom_heading_rad,
 const runtime::MapSnapshot& map, double timestamp_s) const override;

 private:
 KnownMapLocalizationConfig config_;
};

} // namespace sim
