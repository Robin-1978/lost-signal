#pragma once

#include "robot/autonomy/localization/localization_provider.hpp"

#include <cstddef>

namespace sim {

// Bounded global scan matcher for a small known map. This is a startup and
// recovery provider, not a replacement for continuous odometry or a claim of
// full AMCL probabilistic filtering.
struct GlobalMapLocalizationConfig {
 double grid_step_m{0.35};
 double heading_step_rad{0.35};
 double minimum_overlap{0.30};
 double minimum_score_margin{0.03};
 double ambiguity_distance_m{0.75};
 double ambiguity_heading_rad{0.70};
 std::size_t minimum_returns{12};
 std::size_t maximum_returns{96};
 std::size_t maximum_evaluations{1500000};
 double sensor_x_m{0.0};
 double sensor_y_m{0.0};
 double sensor_yaw_rad{0.0};
};

class GlobalMapLocalizer final : public autonomy::ILocalizationProvider {
 public:
 explicit GlobalMapLocalizer(GlobalMapLocalizationConfig config = {})
 : config_(config) {}

 autonomy::LocalizationResult localize(
 const Scan& scan, const Vec2& odom_position, double odom_heading_rad,
 const runtime::MapSnapshot& map, double timestamp_s) const override;

 private:
 GlobalMapLocalizationConfig config_;
};

} // namespace sim
