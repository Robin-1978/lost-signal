#include "sim/hardware/vehicle/holonomic.hpp"
#include "sim/hardware/vehicle/chassis_factory.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

namespace sim {
namespace {

constexpr std::array<double, 4> kXSign{1.0, 1.0, -1.0, -1.0};
constexpr std::array<double, 4> kYSign{1.0, -1.0, 1.0, -1.0};

double angle_delta(double target, double current) {
 return wrap_angle(target - current);
}

} // namespace

std::array<HolonomicWheelCommand, 4> swerve_inverse(double forward_mps, double left_mps,
 double yaw_radps, double wheel_base_m, double track_width_m,
 const std::array<double, 4>& previous_steer_rad) {
 std::array<HolonomicWheelCommand, 4> output;
 for (std::size_t index = 0; index < output.size(); ++index) {
 const double x = kXSign[index] * wheel_base_m * 0.5;
 const double y = kYSign[index] * track_width_m * 0.5;
 const double wheel_x = forward_mps - yaw_radps * y;
 const double wheel_y = left_mps + yaw_radps * x;
 double speed = std::hypot(wheel_x, wheel_y);
 double steer = speed > 1e-9 ? std::atan2(wheel_y, wheel_x) : previous_steer_rad[index];
 // A swerve module can reverse the wheel and rotate 180 degrees less. This
 // prevents slow steering flips around the +/-pi boundary.
 if (std::abs(angle_delta(steer, previous_steer_rad[index])) > kPi * 0.5) {
 steer = wrap_angle(steer + kPi);
 speed = -speed;
 }
 output[index] = {speed, steer};
 }
 return output;
}

std::array<double, 4> mecanum_inverse(double forward_mps, double left_mps, double yaw_radps,
 double wheel_base_m, double track_width_m) {
 const double radius = (wheel_base_m + track_width_m) * 0.5;
 return {forward_mps - left_mps - radius * yaw_radps,
 forward_mps + left_mps + radius * yaw_radps,
 forward_mps + left_mps - radius * yaw_radps,
 forward_mps - left_mps + radius * yaw_radps};
}

void mecanum_forward(const std::array<double, 4>& wheel, double wheel_base_m,
 double track_width_m, double& forward_mps, double& left_mps, double& yaw_radps) {
 const double radius = std::max(0.05, (wheel_base_m + track_width_m) * 0.5);
 forward_mps = (wheel[0] + wheel[1] + wheel[2] + wheel[3]) * 0.25;
 left_mps = (-wheel[0] + wheel[1] + wheel[2] - wheel[3]) * 0.25;
 yaw_radps = (-wheel[0] + wheel[1] - wheel[2] + wheel[3]) / (4.0 * radius);
}

HolonomicDrive::HolonomicDrive(HolonomicParams params) : params_(params) {
 telemetry_.differential = false;
 telemetry_.holonomic = true;
 telemetry_.wheel_steer_count = params_.kind == HolonomicKind::Swerve ? 4 : 0;
}

void HolonomicDrive::set_body_dynamics(const ChassisBodyDynamics& dynamics) {
 params_.mass_kg = std::max(0.5, dynamics.mass_kg);
 params_.yaw_inertia_kgm2 = std::max(0.05, dynamics.yaw_inertia_kgm2);
}

DiffState HolonomicDrive::step(const DiffState& state, const DiffCmd& command, double dt,
 const GroundContact& ground) {
 double target_forward = command.target_v;
 double target_left = command.target_lateral_v;
 double target_yaw = command.target_w;
 std::array<double, 4> wheel_speeds{};
 if (params_.kind == HolonomicKind::Mecanum) {
 wheel_speeds = mecanum_inverse(target_forward, target_left, target_yaw,
 params_.wheel_base_m, params_.track_width_m);
 } else {
 const auto modules = swerve_inverse(target_forward, target_left, target_yaw,
 params_.wheel_base_m, params_.track_width_m, telemetry_.wheel_steer_rad);
 for (std::size_t index = 0; index < modules.size(); ++index) {
 wheel_speeds[index] = modules[index].speed_mps;
 telemetry_.wheel_steer_rad[index] = modules[index].steer_rad;
 }
 }
 double peak = 0.0;
 for (double speed : wheel_speeds)
 peak = std::max(peak, std::abs(speed));
 if (peak > params_.max_wheel_speed_mps) {
 const double scale = params_.max_wheel_speed_mps / peak;
 target_forward *= scale;
 target_left *= scale;
 target_yaw *= scale;
 for (double& speed : wheel_speeds)
 speed *= scale;
 }
 const double accel_limit = std::min(params_.max_accel_mps2,
 std::max(0.1, ground.mu_eff * 9.81));
 DiffState next = state;
 next.v += clamp(target_forward - state.v, -accel_limit * dt, accel_limit * dt);
 next.lateral_v += clamp(target_left - state.lateral_v, -accel_limit * dt, accel_limit * dt);
 next.w += clamp(target_yaw - state.w, -params_.max_yaw_accel_radps2 * dt,
 params_.max_yaw_accel_radps2 * dt);
 const double heading_mid = state.theta + next.w * dt * 0.5;
 next.x += (next.v * std::cos(heading_mid) - next.lateral_v * std::sin(heading_mid)) * dt;
 next.y += (next.v * std::sin(heading_mid) + next.lateral_v * std::cos(heading_mid)) * dt;
 next.theta = wrap_angle(state.theta + next.w * dt);
 telemetry_.body_forward_force_N = params_.mass_kg * (next.v - state.v) / std::max(1e-6, dt);
 telemetry_.body_lateral_force_N =
 params_.mass_kg * (next.lateral_v - state.lateral_v) / std::max(1e-6, dt);
 telemetry_.extra_yaw_torque_Nm =
 params_.yaw_inertia_kgm2 * (next.w - state.w) / std::max(1e-6, dt);
 telemetry_.desired_yaw_rate_radps = target_yaw;
 telemetry_.measured_forward_mps = next.v;
 telemetry_.measured_lateral_mps = next.lateral_v;
 telemetry_.measured_yaw_radps = next.w;
 telemetry_.traction_saturated = false;
 for (std::size_t index = 0; index < wheel_speeds.size(); ++index) {
 telemetry_.modules[index].target_mps = wheel_speeds[index];
 telemetry_.modules[index].measured_mps = wheel_speeds[index];
 telemetry_.modules[index].actual_mps = wheel_speeds[index];
 telemetry_.modules[index].pwm = clamp(wheel_speeds[index] / params_.max_wheel_speed_mps, -1.0, 1.0);
 }
 return next;
}

namespace {
BuiltChassis create_holonomic(const ChassisBuildParams& params, HolonomicKind kind) {
 BuiltChassis output;
 output.model = std::make_unique<HolonomicDrive>(HolonomicParams{kind, params.wheel_base_m,
 params.track_width_m, params.max_wheel_speed_mps,
 std::min(params.max_wheel_accel_mps2, params.behavior_accel_max),
 params.max_yaw_accel_radps2, params.mass_kg, params.yaw_inertia_kgm2,
 params.behavior_w_max});
 return output;
}
} // namespace

BuiltChassis create_swerve_chassis(const ChassisBuildParams& params, const WheelPidGains&) {
 return create_holonomic(params, HolonomicKind::Swerve);
}

BuiltChassis create_mecanum_chassis(const ChassisBuildParams& params, const WheelPidGains&) {
 return create_holonomic(params, HolonomicKind::Mecanum);
}

} // namespace sim
