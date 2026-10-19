#pragma once

#include "robot/platform/chassis_caps.hpp"

namespace sim {

// Measured chassis motion at the instant a safety stop is requested. `turn`
// is yaw rate for YawRate/HolonomicTwist chassis and steering angle for an
// Ackermann chassis.
struct SafeStopMotion {
 double forward_mps{0.0};
 double lateral_mps{0.0};
 double turn{0.0};
};

struct SafeStopLimits {
 double update_interval_s{0.0};
 double brake_decel_mps2{0.0};
 double max_speed_mps{0.0};
 double max_turn{0.0};
 double max_turn_rate{0.0};
 double differential_track_m{0.0};
 double max_wheel_speed_mps{0.0};
 double longitudinal_grade{0.0};
 double center_of_mass_x_m{0.0};
 double center_of_mass_height_m{0.0};
 double stability_margin_fraction{0.70};
};

struct SafeStopCommand {
 double forward_mps{0.0};
 double lateral_mps{0.0};
 double turn{0.0};
 double applied_linear_decel_limit_mps2{0.0};
};

// Maximum symmetric command-space acceleration that is both available from
// the declared brake plant and compatible with the current longitudinal
// support/COM envelope. Planners use the same value when reserving a stopping
// tail, so validation and execution cannot assume different brake authority.
double safe_stop_linear_decel_limit(const ChassisCaps& chassis,
 const SafeStopLimits& limits);

// Produce the closest zero-motion command reachable during one control
// interval. The limiter uses the assembled support polygon and combined COM;
// it does not inspect simulator contacts or alter the physical state.
SafeStopCommand stage_safe_stop(const ChassisCaps& chassis,
 const SafeStopMotion& motion, const SafeStopLimits& limits);

} // namespace sim
