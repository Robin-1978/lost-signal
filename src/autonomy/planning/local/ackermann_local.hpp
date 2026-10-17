#pragma once

#include "autonomy/planning/local/dwa.hpp"
#include "autonomy/planning/path.hpp"

namespace sim {

struct AckermannLocalConfig {
 double wheel_base_m{0.6};
 double max_steer_rad{0.55};
 double max_steer_rate_radps{2.0};
 double steering_response_s{0.0};
 double max_accel_mps2{3.0};
 double max_speed_mps{2.0};
 // A positive behavior floor forbids stopping/reversing during an already
 // committed maneuver. Zero preserves ordinary local-planner behavior.
 double min_forward_speed_mps{0.0};
 double max_reverse_speed_mps{0.25};
 TravelDirection preferred_direction{TravelDirection::Unspecified};
 double predict_time_s{2.0};
 double dt_s{0.1};
 double frame_dt_s{0.1};
 double safety_margin_m{0.0};
 double robot_radius_m{0.38};
};

// Car-like local trajectory sampler. DwaResult is reused as a visualization
// envelope; DwaCmd::w carries steering angle (delta), not yaw rate.
DwaResult ackermann_local_compute(Vec2 position, double heading_rad, double current_speed_mps,
 double current_steer_rad, double current_steer_rate_radps,
 Vec2 goal, const GridMap& map, DwaWeights weights,
 const std::vector<std::vector<DynamicObjectState>>& predicted_frames,
 const AckermannLocalConfig& config, StaticFeasibility terrain_feasible = {});

} // namespace sim
