#pragma once

#include "robot/platform/differential_kinematics.hpp"
#include "robot/platform/ackermann_kinematics.hpp"
#include "sim/hardware/vehicle/motor_drive.hpp"
#include "robot/control/pid.hpp"

#include <algorithm>
#include <cmath>

namespace sim {

// Ackermann actuator parameters. Everything comes from robot JSON once
// ackermann content exists; there are no per-vehicle hand-tuned gains because
// both loops are derived (drive like a wheel, servo by response spec).
struct AckermannParams {
 double wheel_base_m{2.0};
 double wheel_radius_m{0.1};
 // Track width (left-right wheel distance): the linkage input is the single
 // servo angle, but each front wheel needs its own output angle below.
 double track_width_m{0.58};
 double max_steer_rad{0.6};
 double max_steer_rate_radps{2.5};
 double servo_response_s{0.4};
 double max_drive_speed_mps{2.4};
 double max_drive_accel_mps2{4.0};
 double stall_force_N{54.0};
 double free_speed_mps{3.0};
 double mass_kg{24.0};
 double yaw_inertia_kgm2{2.4};
 double pwm_limit{1.0};
 double integral_limit{2.0};
 int encoder_counts_per_revolution{2048};
 double wheel_equiv_mass_kg{3.0};
 double steering_stall_torque_Nm{8.0};
};

// Ackermann kinematics pair (uniform per-chassis contract, plan §4 item 14):
// inverse maps body twist to steer demand, forward maps executed steer back
// to yaw. Same module, consistent by construction; the property test pins
// forward(inverse(v, w)) == w wherever the speed floor does not intervene.
// The 0.5 m/s floor is CONTROL policy (a car cannot turn standing still),
// not kinematics: below it the pair is intentionally not inverse.
struct SteerSolution {
 double delta_rad{0.0};
 bool feasible{false};
};
SteerSolution steer_for_twist(double target_w, double target_v, double wheel_base_m, double max_steer_rad);
double yaw_for_steer(double body_v, double delta_rad, double wheel_base_m);

// Ackermann chassis: one drive motor (speed PID -> PWM -> DC force, same
// physics as a differential wheel) plus one steering servo (angle command -> position P loop ->
// rate-limited first-order angle). Outputs differ from differential on purpose:
// drive PWM + steer PWM/angle instead of two wheel PWMs. Bicycle kinematics
// (w = v * tan(delta) / L) means no spinning at standstill.
class AckermannDrive : public ChassisModel {
 public:
 AckermannDrive(AckermannParams params, WheelPidGains drive_gains, double servo_kp);

 void reset();
 void set_body_dynamics(const ChassisBodyDynamics& dynamics) override;
 void set_actuator_feedback(const ChassisActuatorFeedback& feedback) override;
 DiffState step(const DiffState& state, const DiffCmd& command, double dt,
                const GroundContact& ground) override;
 bool accepts_actuator_duty() const override { return true; }
 DiffState step_actuator_duty(const DiffState& state, const DiffCmd& command,
 const ChassisActuatorDutyCommand& control, double dt,
 const GroundContact& ground) override;
 const ChassisTelemetry& telemetry() const override { return telemetry_; }
 ChassisCaps caps() const override {
 ChassisCaps output;
 output.can_spin = false;
 output.command_space = ChassisCommandSpace::SteeringAngle;
 output.geometry.support_width_m = params_.track_width_m;
 output.geometry.support_length_m = params_.wheel_base_m;
 output.geometry.steering_wheel_base_m = params_.wheel_base_m;
 output.geometry.max_steer_rad = params_.max_steer_rad;
 output.geometry.max_steer_rate_radps = params_.max_steer_rate_radps;
 output.geometry.steering_response_s = params_.servo_response_s;
 output.geometry.minimum_turn_radius_m =
 params_.wheel_base_m / std::tan(std::max(0.01, params_.max_steer_rad));
 output.max_yaw_rate_radps = params_.max_drive_speed_mps * std::tan(params_.max_steer_rad) /
 std::max(0.05, params_.wheel_base_m);
 output.drive_force_units = 1.0;
 output.planning_clearance_margin_m = 0.15;
 output.tread_slip_observable = true;
 return output;
 }
 double steer_angle() const { return delta_rad_; }

 private:
 double encoder_measure(double wheel_mps, double dt) const;

 AckermannParams params_;
 PID drive_pid_;
 PID servo_pid_;
 double delta_rad_{0.0};
 double servo_rate_state_{0.0};
 double drive_wheel_{0.0};
 ChassisTelemetry telemetry_;
 ChassisActuatorFeedback feedback_;
};

} // namespace sim
