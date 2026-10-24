#pragma once

#include <cstddef>

#include "robot/autonomy/planning/global_planner.hpp"

namespace lost_signal::robot::autonomy {

// Geometry-only post processing for car-like routes. It changes headings,
// never positions or gear decisions, so the collision swept by Hybrid-A*
// remains the authority. The limits are chassis/config values, not a
// scenario-specific threshold.
struct RouteSmoothingConfig {
 double wheel_base_m = 0.56;
 double max_steer_rad = 0.55;
 double max_heading_step_rad = 0.35;
 double max_curvature_delta_per_m = 1.5;
};

// Returns false for malformed routes or a path whose fixed positions cannot
// be represented by the configured bicycle envelope. A false result is a
// fail-closed planning failure, never a request to use a holonomic fallback.
bool smooth_ackermann_route(Route& route, const RouteSmoothingConfig& config);

} // namespace lost_signal::robot::autonomy
