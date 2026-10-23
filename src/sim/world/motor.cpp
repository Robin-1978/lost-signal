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
  return out;
}

}  // namespace lost_signal::sim::world
