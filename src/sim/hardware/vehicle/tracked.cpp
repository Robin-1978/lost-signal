#include "sim/hardware/vehicle/tracked.hpp"
#include "sim/hardware/vehicle/chassis_factory.hpp"
#include "sim/hardware/actuators/electric_drive.hpp"

#include "robot/core/math.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace sim {
namespace {

// Shared contact constants live in hw/dynamics; only the track-specific
// scrub numbers stay here. Coulomb smoothing width matches the wheel models
// so standstill behavior is identical across chassis.
constexpr double kGravity = 9.81;
constexpr double kCoulombWidthMps = 0.05;
constexpr double kViscousNPerMps = 0.6;
// Scrub damping: tracks resist yaw at roughly the Coulomb level with a
// small viscous tail so spin settles instead of ringing at the 20 ms step.
// The viscous number is a method constant; the Coulomb arm comes from the
// track length in JSON (moment ~= mu*N*L/4 for a uniform patch).
constexpr double kYawViscousNmsPerRadps = 2.0;

} // namespace

TrackedDrive::TrackedDrive(TrackedParams params, WheelPidGains drive_gains)
    : params_(params),
left_pid_(drive_gains.kp, drive_gains.ki, drive_gains.kd, params.pwm_limit, params.integral_limit,
 AntiWindupMode::ConditionalIntegration),
right_pid_(drive_gains.kp, drive_gains.ki, drive_gains.kd, params.pwm_limit, params.integral_limit,
 AntiWindupMode::ConditionalIntegration) {
 telemetry_.differential = true;
}

void TrackedDrive::reset() {
 left_pid_.reset();
 right_pid_.reset();
 left_tread_mps_ = 0.0;
 right_tread_mps_ = 0.0;
 telemetry_ = {};
 telemetry_.differential = true;
}

void TrackedDrive::set_body_dynamics(const ChassisBodyDynamics& dynamics) {
 params_.mass_kg = std::max(0.5, dynamics.mass_kg);
 params_.yaw_inertia_kgm2 = std::max(0.05, dynamics.yaw_inertia_kgm2);
}

void TrackedDrive::set_actuator_feedback(const ChassisActuatorFeedback& feedback) {
 feedback_ = feedback;
}

double TrackedDrive::encoder_measure(double wheel_mps, double dt) const {
 const double circumference = 2.0 * kPi * params_.wheel_radius_m;
 const double revolutions = wheel_mps * dt / circumference;
 const double counts = std::round(revolutions * static_cast<double>(params_.encoder_counts_per_revolution));
 return counts * circumference / (static_cast<double>(params_.encoder_counts_per_revolution) * dt);
}

DiffState TrackedDrive::step(const DiffState& state, const DiffCmd& command, double dt,
                             const GroundContact& ground) {
 auto& left = telemetry_.left;
 auto& right = telemetry_.right;

 DifferentialKinematics::twist_to_wheels(command.target_v, command.target_w, params_.effective_track_width_m, left.target_mps,
                          right.target_mps);
 left.target_mps = clamp(left.target_mps, -params_.max_drive_speed_mps, params_.max_drive_speed_mps);
 right.target_mps = clamp(right.target_mps, -params_.max_drive_speed_mps, params_.max_drive_speed_mps);
 // Encoders read tread: when a track breaks away the encoder reports the
 // slide, exactly what makes tracked odometry lie in turns.
 left.measured_mps = encoder_measure(left_tread_mps_, dt);
 right.measured_mps = encoder_measure(right_tread_mps_, dt);
 left.pwm = left_pid_.update(left.target_mps - left.measured_mps, dt);
 right.pwm = right_pid_.update(right.target_mps - right.measured_mps, dt);
 left.pid_p = left_pid_.p();
 left.pid_i = left_pid_.i();
 left.pid_d = left_pid_.d();
 right.pid_p = right_pid_.p();
 right.pid_i = right_pid_.i();
 right.pid_d = right_pid_.d();

 double body_left = 0.0;
 double body_right = 0.0;
 DifferentialKinematics::twist_to_wheels(state.v, state.w, params_.effective_track_width_m, body_left, body_right);
 const double weight_per_track = params_.mass_kg * kGravity / 2.0;
 const double traction_limit = std::max(0.0, ground.mu_eff) * weight_per_track;
 const double resist_coeff = std::max(0.0, ground.rolling_resist);
 auto resistance = [&](double body_track_mps) {
  return resist_coeff * weight_per_track * std::tanh(body_track_mps / kCoulombWidthMps) +
         kViscousNPerMps * body_track_mps;
 };
 auto drive_track = [&](double pwm, double tread_mps, double body_track_mps, WheelMotorTelemetry& telemetry) {
  const WheelContactResult contact = step_driven_wheel(
    tread_mps, body_track_mps, pwm,
    {params_.stall_force_N, params_.free_speed_mps, traction_limit, params_.wheel_equiv_mass_kg}, dt);
  telemetry.slipping = contact.slipping;
  return std::make_pair(contact.wheel_mps, contact.body_force_N - resistance(body_track_mps));
 };
 const auto [tread_left, force_left] = drive_track(left.pwm, left_tread_mps_, body_left, left);
 const auto [tread_right, force_right] = drive_track(right.pwm, right_tread_mps_, body_right, right);
 left_tread_mps_ = tread_left;
 right_tread_mps_ = tread_right;

 // Yaw scrub: turning drags the whole patch sideways. Coulomb moment from
 // the track length, viscous tail for settle. This is why tracked spin is
 // slower than differential spin at the same command.
 const double scrub_coulomb =
   std::max(0.0, ground.mu_eff) * params_.mass_kg * kGravity * std::max(0.1, params_.track_length_m) / 4.0;
 const double torque = (force_right - force_left) * params_.wheel_base_m / 2.0 -
                       scrub_coulomb * std::tanh(state.w / kCoulombWidthMps) -
                       kYawViscousNmsPerRadps * state.w;
 left.longitudinal_force_N = force_left;
 right.longitudinal_force_N = force_right;
 telemetry_.extra_yaw_torque_Nm = -scrub_coulomb * std::tanh(state.w / kCoulombWidthMps) -
 kYawViscousNmsPerRadps * state.w;
 telemetry_.desired_yaw_rate_radps = command.target_w;
 telemetry_.yaw_rate_tracking_time_s = 0.35;
 const double alpha = torque / std::max(0.05, params_.yaw_inertia_kgm2);
 const double accel = (force_left + force_right) / std::max(0.5, params_.mass_kg);
 const double accel_cap = std::min(params_.max_drive_accel_mps2, std::max(0.5, ground.accel_limit_mps2));
 DiffState next = state;
 next.v = state.v + std::clamp(accel * dt, -accel_cap * dt, accel_cap * dt);
 next.w = state.w + alpha * dt;
 DifferentialKinematics::twist_to_wheels(next.v, next.w, params_.effective_track_width_m, left.actual_mps, right.actual_mps);
 left.actual_mps = clamp(left.actual_mps, -params_.max_drive_speed_mps, params_.max_drive_speed_mps);
 right.actual_mps = clamp(right.actual_mps, -params_.max_drive_speed_mps, params_.max_drive_speed_mps);
 DifferentialKinematics::wheels_to_twist(left.actual_mps, right.actual_mps, params_.effective_track_width_m, next.v, next.w);
 DifferentialKinematics::wheels_to_twist(left.measured_mps, right.measured_mps, params_.effective_track_width_m,
 telemetry_.measured_forward_mps, telemetry_.measured_yaw_radps);
 telemetry_.measured_lateral_mps = 0.0;
 telemetry_.traction_saturated = left.slipping || right.slipping;
 next.x = state.x + next.v * std::cos(state.theta) * dt;
 next.y = state.y + next.v * std::sin(state.theta) * dt;
 next.theta = wrap_angle(state.theta + next.w * dt);
 return next;
}

DiffState TrackedDrive::step_actuator_duty(const DiffState& state, const DiffCmd& command,
                                           const ChassisActuatorDutyCommand& control, double dt,
                                           const GroundContact&) {
 constexpr std::array<const char*, 3> kLeftChannels{"front_left_drive", "middle_left_drive",
                                                    "rear_left_drive"};
 constexpr std::array<const char*, 3> kRightChannels{"front_right_drive", "middle_right_drive",
                                                     "rear_right_drive"};
 const auto mean_velocity = [&](const auto& channels) {
  double sum = 0.0;
  for (const auto* channel : channels) {
   const auto* sample = feedback_.find(channel);
   if (!sample)
    throw std::runtime_error("articulated tracked plant has incomplete road-wheel feedback");
   sum += sample->velocity_radps;
  }
  return sum / static_cast<double>(channels.size());
 };
 const double left_velocity_radps = mean_velocity(kLeftChannels);
 const double right_velocity_radps = mean_velocity(kRightChannels);
 const double left_duty = clamp(control.value("left"), -params_.pwm_limit, params_.pwm_limit);
 const double right_duty = clamp(control.value("right"), -params_.pwm_limit, params_.pwm_limit);
 const auto torque = [&](double duty, double velocity_radps) {
  const double speed_mps = velocity_radps * params_.wheel_radius_m;
  return hardware::dc_motor_force_N(duty, speed_mps, params_.stall_force_N, params_.free_speed_mps) *
         params_.wheel_radius_m;
 };

 telemetry_ = {};
 telemetry_.differential = true;
 telemetry_.left.target_mps = command.target_v - command.target_w * params_.effective_track_width_m * 0.5;
 telemetry_.right.target_mps = command.target_v + command.target_w * params_.effective_track_width_m * 0.5;
 telemetry_.left.actual_mps = left_velocity_radps * params_.wheel_radius_m;
 telemetry_.right.actual_mps = right_velocity_radps * params_.wheel_radius_m;
 telemetry_.left.measured_mps = telemetry_.left.actual_mps;
 telemetry_.right.measured_mps = telemetry_.right.actual_mps;
 telemetry_.left.pwm = left_duty;
 telemetry_.right.pwm = right_duty;
 telemetry_.measured_forward_mps = 0.5 * (telemetry_.left.actual_mps + telemetry_.right.actual_mps);
 telemetry_.measured_yaw_radps = (telemetry_.right.actual_mps - telemetry_.left.actual_mps) /
                                 std::max(0.05, params_.effective_track_width_m);
 telemetry_.desired_yaw_rate_radps = command.target_w;
 // One physical motor is authored at each road wheel. The logical side duty
 // is a bus command, so every motor receives the same duty and retains its
 // own declared stall torque; dividing here would model one motor split by a
 // fictional ideal gearbox and cannot overcome real six-contact scrub.
 const double left_torque = torque(left_duty, left_velocity_radps);
 const double right_torque = torque(right_duty, right_velocity_radps);
 for (const auto* channel : kLeftChannels)
  telemetry_.joint_efforts.push_back({channel, left_torque});
 for (const auto* channel : kRightChannels)
  telemetry_.joint_efforts.push_back({channel, right_torque});

 // This planar value is shadow evidence only. In articulated mode Bullet
 // integrates the two joint torques, six rolling contacts, gravity and scrub.
 DiffState next = state;
 next.v = telemetry_.measured_forward_mps;
 next.w = telemetry_.measured_yaw_radps;
 next.x += next.v * std::cos(state.theta) * dt;
 next.y += next.v * std::sin(state.theta) * dt;
 next.theta = wrap_angle(state.theta + next.w * dt);
 return next;
}

} // namespace sim

// Chassis creator for the factory registry (see motor_drive.cpp).
namespace sim {

BuiltChassis create_tracked_chassis(const sim::ChassisBuildParams& params,
                                 const sim::WheelPidGains& gains) {
  // Twin tracks share the differential speed loop and yaw-rate output units;
  // only the yaw authority comes from the vendor spin spec instead of
  // DiffDrive geometry, because scrub decides what a track can hold.
  sim::BuiltChassis out;
  out.model = std::make_unique<sim::TrackedDrive>(
    sim::TrackedParams{
      params.wheel_base_m,
     params.effective_track_width_m,
      params.wheel_radius_m,
      params.track_length_m,
      params.max_wheel_speed_mps,
      std::min(params.max_wheel_accel_mps2, params.behavior_accel_max),
      params.stall_force_N,
      params.free_speed_mps,
      params.mass_kg,
      params.yaw_inertia_kgm2,
      params.tracked_max_yaw_rate_rads,
      params.pwm_limit,
      params.integral_limit,
      params.encoder_counts_per_revolution,
      params.wheel_equiv_mass_kg,
 params.track_width_m,
    },
    gains);
  return out;
}

} // namespace sim

