#include "sim/hardware/vehicle/holonomic.hpp"
#include "sim/hardware/vehicle/chassis_factory.hpp"
#include "sim/hardware/actuators/electric_drive.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

namespace sim {
namespace {

constexpr std::array<const char*, 4> kDriveChannels{"front_left_drive", "front_right_drive",
                                                    "rear_left_drive", "rear_right_drive"};
constexpr std::array<const char*, 4> kSteerChannels{"front_left_steer", "front_right_steer",
                                                    "rear_left_steer", "rear_right_steer"};
} // namespace

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

DiffState HolonomicDrive::step_actuator_duty(const DiffState& state, const DiffCmd& command,
                                             const ChassisActuatorDutyCommand& control, double dt,
                                             const GroundContact&) {
 std::array<double, 4> wheel_mps{};
 std::array<double, 4> steer_rad{};
 telemetry_ = {};
 telemetry_.differential = false;
 telemetry_.holonomic = true;
 telemetry_.wheel_steer_count = params_.kind == HolonomicKind::Swerve ? 4 : 0;
 const auto motor_torque = [&](double duty, double omega) {
  return hardware::dc_motor_force_N(duty, omega * params_.wheel_radius_m, params_.stall_force_N,
                                    params_.free_speed_mps) *
         params_.wheel_radius_m;
 };
 for (std::size_t i = 0; i < 4; ++i) {
  const auto* drive = feedback_.find(kDriveChannels[i]);
  if (!drive)
   throw std::runtime_error("articulated holonomic plant has incomplete drive feedback");
  const double duty = clamp(control.value(kDriveChannels[i]), -params_.pwm_limit, params_.pwm_limit);
  wheel_mps[i] = drive->velocity_radps * params_.wheel_radius_m;
  telemetry_.modules[i].pwm = duty;
  telemetry_.modules[i].actual_mps = wheel_mps[i];
  telemetry_.modules[i].measured_mps = wheel_mps[i];
  telemetry_.joint_efforts.push_back({kDriveChannels[i], motor_torque(duty, drive->velocity_radps)});
  if (params_.kind == HolonomicKind::Swerve) {
   const auto* steer = feedback_.find(kSteerChannels[i]);
   if (!steer)
    throw std::runtime_error("articulated swerve plant has incomplete steering feedback");
   const double steer_duty = clamp(control.value(kSteerChannels[i]), -1.0, 1.0);
   steer_rad[i] = steer->position_rad;
   telemetry_.wheel_steer_rad[i] = steer_rad[i];
   const double curve = steer_duty * steer->velocity_radps > 0.0
                          ? std::max(0.0, 1.0 - std::abs(steer->velocity_radps) / 8.0)
                          : 1.0;
   telemetry_.joint_efforts.push_back(
     {kSteerChannels[i], steer_duty * params_.steering_stall_torque_Nm * curve});
  }
 }
 if (params_.kind == HolonomicKind::Swerve)
  swerve_forward(wheel_mps, steer_rad, params_.wheel_base_m, params_.track_width_m,
                 telemetry_.measured_forward_mps, telemetry_.measured_lateral_mps,
                 telemetry_.measured_yaw_radps);
 else
  mecanum_forward(wheel_mps, params_.wheel_base_m, params_.track_width_m, telemetry_.measured_forward_mps,
                  telemetry_.measured_lateral_mps, telemetry_.measured_yaw_radps);
 DiffState next = state;
 next.v = telemetry_.measured_forward_mps;
 next.lateral_v = telemetry_.measured_lateral_mps;
 next.w = telemetry_.measured_yaw_radps;
 next.x += (next.v * std::cos(state.theta) - next.lateral_v * std::sin(state.theta)) * dt;
 next.y += (next.v * std::sin(state.theta) + next.lateral_v * std::cos(state.theta)) * dt;
 next.theta = wrap_angle(state.theta + next.w * dt);
 (void)command;
 return next;
}

namespace {
BuiltChassis create_holonomic(const ChassisBuildParams& params, HolonomicKind kind) {
 BuiltChassis output;
 output.model = std::make_unique<HolonomicDrive>(HolonomicParams{kind, params.wheel_base_m,
 params.track_width_m, params.max_wheel_speed_mps,
 std::min(params.max_wheel_accel_mps2, params.behavior_accel_max),
 params.max_yaw_accel_radps2, params.mass_kg, params.yaw_inertia_kgm2,
 params.behavior_w_max, params.wheel_radius_m,
   params.stall_force_N, params.free_speed_mps, params.pwm_limit, params.steering_stall_torque_Nm});
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
