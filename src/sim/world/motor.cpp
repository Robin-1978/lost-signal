#include "sim/world/motor.hpp"

#include <algorithm>
#include <cmath>

#include "robot/platform/drive_profile.hpp"
#include "sim/world/world.hpp"

namespace lost_signal::sim::world {

double dc_motor_force_N(double normalized_duty, double wheel_mps,
                        double stall_force_N, double free_speed_mps) {
  const double duty = std::clamp(normalized_duty, -1.0, 1.0);
  if (std::abs(duty) <= 1e-12) return 0.0;
  const bool motoring = duty * wheel_mps > 0.0;
  const double curve =
      motoring
          ? std::max(0.0,
                     1.0 - std::abs(wheel_mps) / std::max(0.05, free_speed_mps))
          : 1.0;
  return duty * std::max(0.0, stall_force_N) * curve;
}

MotorParams motor_params_from_profile(
    const robot::platform::DriveProfile& profile) {
  MotorParams out;
  out.free_speed_mps = profile.free_speed_mps;
  out.stall_force_N = profile.stall_force_N;
  out.wheel_equiv_mass_kg = profile.wheel_equiv_mass_kg;
  out.vehicle_mass_kg = profile.vehicle_mass_kg;
  out.rolling_resist = profile.rolling_resist;
  out.viscous_drag_N_per_mps = profile.viscous_drag_N_per_mps;
  out.coulomb_smoothing_mps = profile.coulomb_smoothing_mps;
  out.wheel_radius_m = profile.wheel_radius_m;
  out.track_width_m = profile.track_width_m;
  out.wheel_base_m = profile.wheel_base_m;
  out.max_steer_rad = profile.max_steer_rad;
  out.max_steer_rate_radps = profile.max_steer_rate_radps;
  out.steer_time_constant_s = profile.steer_time_constant_s;
  out.encoder_counts_per_revolution = profile.encoder_counts_per_revolution;
  out.steer_stall_torque_Nm = profile.steer_stall_torque_Nm;
  out.tire_friction_mu = profile.tire_friction_mu;
  out.yaw_scrub_coulomb_Nm = profile.yaw_scrub_coulomb_Nm;
  out.yaw_scrub_viscous_Nm_per_radps = profile.yaw_scrub_viscous_Nm_per_radps;
  // Chassis hard geometry: verbatim copy, plus the legacy support default
  // (empty profile list = fore/aft caster pair at +-0.9*wheel_base/2).
  out.body_half_height_m = profile.body_half_height_m;
  out.body_half_length_m = profile.body_half_length_m;
  out.body_half_width_m = profile.body_half_width_m;
  out.drive_axle_x_m = profile.drive_axle_x_m;
  out.tire_half_width_m = profile.tire_half_width_m;
  out.wheel_mass_kg = profile.wheel_mass_kg;
  out.knuckle_mass_kg = profile.knuckle_mass_kg;
  // Effective COM: 0 = follow the axle (single rule, same as the loader).
  out.com_height_m = profile.com_height_m > 0.0
                         ? profile.com_height_m
                         : profile.wheel_radius_m;
  out.payload_mass_kg = profile.payload_mass_kg;
  out.payload_height_m = profile.payload_height_m;
  out.support_count = 0;
  if (profile.supports.empty() && profile.chassis_kind == "tracked") {
 // Tracks are the support surface; adding a free caster pair would inject
 // an unmodelled yaw scrub torque into skid-steer pivot tests.
 out.support_count = -1;
 } else if (profile.supports.empty()) {
    const double sx = 0.9 * profile.wheel_base_m / 2;
    MotorParams::SupportParams b;
    b.kind = "caster";
    b.mass_kg = 0.1;
    b.radius_m = 0.03;
    b.friction_mu = 0.05;
    b.trail_m = 0.02;
    b.swivel_damping_Nms = 0.005;
    b.swivel_friction_Nm = 0.0;
    b.rolling_friction_m = 0.0005;
    b.x_m = sx;
    out.supports[out.support_count++] = b;
    b.x_m = -sx;
    out.supports[out.support_count++] = b;
  } else {
    for (const auto& s : profile.supports) {
      if (out.support_count >= MotorParams::kMaxSupports) break;
      MotorParams::SupportParams b;
      b.kind = s.kind;
      b.x_m = s.x_m;
      b.y_m = s.y_m;
      b.trail_m = s.trail_m;
      b.radius_m = s.radius_m;
      b.mass_kg = s.mass_kg;
      b.friction_mu = s.friction_mu;
      b.swivel_damping_Nms = s.swivel_damping_Nms;
      b.swivel_friction_Nm = s.swivel_friction_Nm;
      b.rolling_friction_m = s.rolling_friction_m;
      out.supports[out.support_count++] = b;
    }
  }
  return out;
}

}  // namespace lost_signal::sim::world
