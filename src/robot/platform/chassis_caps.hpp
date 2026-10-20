#pragma once

#include "robot/core/math.hpp"

#include <vector>

namespace sim {

enum class ChassisCommandSpace { YawRate, SteeringAngle, HolonomicTwist };

// Geometry and limits owned by the assembled chassis. Consumers use this
// contract instead of inferring kinematics from drive.type strings or robot
// JSON. A zero steering field means that capability does not exist.
struct ChassisGeometry {
 double support_width_m{0.5};
 double support_length_m{0.5};
 // Longitudinal support bounds in base_link. Keeping the asymmetric bounds
 // matters for three-point differential chassis: an axle near the front and
 // a rear caster have very different acceleration and braking tip margins.
 double support_min_x_m{-0.25};
 double support_max_x_m{0.25};
 // Convex ground-contact polygon in base_link coordinates. Width/length and
 // extrema remain convenient scalar capabilities, while this polygon keeps
 // diagonal tip-over physics for three-point and asymmetric platforms.
 std::vector<Vec2> support_polygon;
 // Longitudinal offset from base_link to the driven axle. Zero for chassis
 // whose kinematic reference already lies on the axle/module center.
 double drive_axle_x_m{0.0};
 double steering_wheel_base_m{0.0};
 double max_steer_rad{0.0};
 double max_steer_rate_radps{0.0};
 double steering_response_s{0.0};
 double minimum_turn_radius_m{0.0};
};

// What a chassis can physically do. Recovery and planning logic must branch
// on this instead of assuming differential drive: a diff drive spins in
// place, an ackermann cannot (it reverses with steering instead).
struct ChassisCaps {
 bool can_spin{true};
 ChassisCommandSpace command_space{ChassisCommandSpace::YawRate};
 ChassisGeometry geometry;
 double max_yaw_rate_radps{2.0};
 double drive_force_units{2.0};
 double planning_clearance_margin_m{0.0};
 double rigid_body_cruise_scale{1.0};
 bool tread_slip_observable{false};

 double turn_command_limit() const {
 return command_space == ChassisCommandSpace::SteeringAngle
 ? geometry.max_steer_rad
 : max_yaw_rate_radps;
 }
};

} // namespace sim
