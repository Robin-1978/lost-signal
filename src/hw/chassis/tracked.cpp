#include "tracked.hpp"

#include "algo/math.hpp"

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
      left_pid_(drive_gains.kp, drive_gains.ki, drive_gains.kd, params.pwm_limit, params.integral_limit),
      right_pid_(drive_gains.kp, drive_gains.ki, drive_gains.kd, params.pwm_limit, params.integral_limit) {
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

 DiffDrive::twistToWheels(command.target_v, command.target_w, params_.wheel_base_m, left.target_mps,
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
 DiffDrive::twistToWheels(state.v, state.w, params_.wheel_base_m, body_left, body_right);
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
                       scrub_coulomb * std::tanh(state.w / kCoulombWidthMps) - kYawViscousNmsPerRadps * state.w;
 const double alpha = torque / std::max(0.05, params_.yaw_inertia_kgm2);
 const double accel = (force_left + force_right) / std::max(0.5, params_.mass_kg);
 const double accel_cap = std::min(params_.max_drive_accel_mps2, std::max(0.5, ground.accel_limit_mps2));
 DiffState next = state;
 next.v = state.v + std::clamp(accel * dt, -accel_cap * dt, accel_cap * dt);
 next.w = state.w + alpha * dt;
 DiffDrive::twistToWheels(next.v, next.w, params_.wheel_base_m, left.actual_mps, right.actual_mps);
 left.actual_mps = clamp(left.actual_mps, -params_.max_drive_speed_mps, params_.max_drive_speed_mps);
 right.actual_mps = clamp(right.actual_mps, -params_.max_drive_speed_mps, params_.max_drive_speed_mps);
 DiffDrive::wheelsToTwist(left.actual_mps, right.actual_mps, params_.wheel_base_m, next.v, next.w);
 next.x = state.x + next.v * std::cos(state.theta) * dt;
 next.y = state.y + next.v * std::sin(state.theta) * dt;
 next.theta = wrap_angle(state.theta + next.w * dt);
 return next;
}

} // namespace sim
