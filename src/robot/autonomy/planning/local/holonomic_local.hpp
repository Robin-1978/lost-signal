#pragma once

#include "robot/autonomy/planning/local/dwa.hpp"

#include <optional>

namespace sim {

struct HolonomicLocalConfig {
 double max_speed_mps{2.0};
 double min_forward_speed_mps{0.0};
 double max_yaw_radps{2.0};
 double max_accel_mps2{3.0};
 double max_yaw_accel_radps2{6.0};
 double predict_time_s{1.5};
 double dt_s{0.1};
 double frame_dt_s{0.1};
 double safety_margin_m{0.0};
 double robot_radius_m{0.38};
 std::optional<Vec2> terminal_position;
 std::optional<double> terminal_heading_rad;
 double terminal_position_tolerance_m{0.0};
 double terminal_max_speed_mps{0.0};
 double terminal_heading_tolerance_rad{0.12};
 double terminal_max_yaw_rate_radps{0.05};
};

DwaResult holonomic_local_compute(Vec2 position, double heading_rad, double current_forward_mps,
 double current_left_mps, double current_yaw_radps, Vec2 goal, const GridMap& map,
 DwaWeights weights, const std::vector<std::vector<DynamicObjectState>>& predicted_frames,
 const HolonomicLocalConfig& config, StaticFeasibility terrain_feasible = {},
 const DwaReference& reference = {});

} // namespace sim
