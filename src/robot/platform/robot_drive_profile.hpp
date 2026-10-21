#pragma once

#include <nlohmann/json.hpp>

#include <string>

namespace sim {

// Deployable drive-train and chassis capability contract.
//
// These are hardware facts: what the motors can deliver, how the wheels are
// geared, how far apart they are, what the body weighs, and what the brakes can
// hold. They are parsed by ROBOT so a native bring-up reads the same file as the
// simulator without linking any game content type. Game-only metadata (titles,
// unlock gates, component slots, presentation profiles) stays in the catalog.
struct SteeringDefinition {
 double integral_limit{1.2};
 // Ackermann hardware. Unused by differential cars; required content for
 // ackermann robots (the servo derivation and yaw capability come from these).
 double max_steer_rad{0.6};
 double max_steer_rate_radps{2.5};
 double servo_response_s{0.4};
 double stall_torque_Nm{8.0};
};

struct MotorDefinition {
 double max_wheel_speed_mps{0.0};
 double max_wheel_accel_mps2{0.0};
 double stall_force_N{0.0};
 double free_speed_mps{0.0};
 double pwm_limit{1.0};
 double integral_limit{2.0};
 // Per-wheel equivalent translational mass (wheel + tire + reflected rotor).
 // Hardware data: the contact patch integrates the tread against this.
 double wheel_equiv_mass_kg{3.0};
 // Controller and low-speed drivetrain assumptions are authored data, not
 // hidden tuning constants in the simulation backend.
 double controller_bandwidth_radps{6.0};
 double controller_integral_time_s{5.0};
 double controller_derivative_time_s{0.005};
 double static_friction_feedforward_duty{0.08};
 double coulomb_smoothing_speed_mps{0.05};
 double viscous_drag_N_per_mps{0.6};
};

struct EncoderDefinition {
 int counts_per_revolution{0};
};

struct ServiceBrakeDefinition {
 bool enabled{false};
 double maximum_torque_Nm{0.0};
 double apply_time_s{0.12};
 double release_time_s{0.08};
 double low_speed_smoothing_radps{0.6};
};

struct RobotDriveProfile {
 std::string drive_type;
 double wheel_radius_m{0.0};
 double wheel_base_m{0.0};
 // Maximum validated chassis speed for autonomous operation. The equipped
 // motor may have a higher wheel-speed capability; this limit captures the
 // complete running gear, passive supports and stability evidence.
 double rated_speed_mps{0.0};
 // Calibrated separation used by skid-steer inverse/forward kinematics.
 // For ordinary differential wheels this equals wheel_base_m. Tracks need a
 // larger effective value because lateral scrub moves their instantaneous
 // centres of rotation away from the geometric centreline.
 double effective_track_width_m{0.0};
 double mass_kg{0.0};
 double yaw_inertia_kgm2{0.0};
 double max_yaw_accel_radps2{8.0};
 // Tracked spin capability (vendor spec). Only read by tracked robots;
 // wheeled chassis derive yaw authority from geometry instead.
 double tracked_max_yaw_rate_rads{1.5};
 // Track contact length (vendor spec). Only read by tracked robots.
 double track_length_m{0.7};
 // Axle track width (vendor spec). Only read by ackermann robots (linkage).
 double track_width_m{0.58};
 MotorDefinition motor;
 SteeringDefinition steering;
 double energy_base_w{0.0};
 double energy_per_v_w{0.0};
 double energy_per_w_w{0.0};
 EncoderDefinition encoder;
 ServiceBrakeDefinition service_brake;
};

// Parses the deployable `drive` block of a robot definition. `source` names the
// document in error messages (a path on a native deployment, a content id in
// the simulator). Unknown keys are rejected so a renamed hardware field cannot
// silently keep a compiled-in default.
RobotDriveProfile parse_robot_drive(const nlohmann::json& drive, const std::string& source);

// Range-checks a populated profile. This is the same contract the simulator
// enforces, so a profile that boots a real robot is also a valid sim profile.
void validate_robot_drive(const RobotDriveProfile& profile, const std::string& source);

} // namespace sim

