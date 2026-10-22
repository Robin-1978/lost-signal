#pragma once

#include <string>

namespace lost_signal::robot::platform {

// Hardware capability contract shared by native bring-up and SIM: one set
// of numbers means sim and native close the same loops against the same
// plant. Geometry matches the chassis factory defaults; drive motors are
// the DC force curve (stall force + free speed) integrated against the
// per-wheel equivalent mass, so the speed lag is emergent, never fitted;
// steers are velocity servos. JSON file loading lives on the SIM side
// (sim/eval/drive_profile_io), which merges a file over these defaults —
// ROBOT never parses files.
struct DriveProfile {
  std::string chassis_kind = "differential";
  double wheel_radius_m = 0.1;
  double wheel_base_m = 0.56;
  double track_width_m = 0.5;
  double free_speed_mps = 3.0;
  double stall_force_N = 54.0;
  double wheel_equiv_mass_kg = 3.0;
  // Rolling contact, shared with the ROBOT feedforward form (single
  // contract: one file configures both sides).
  double vehicle_mass_kg = 24.0;
  double rolling_resist = 0.01;
  double viscous_drag_N_per_mps = 0.6;
  double coulomb_smoothing_mps = 0.05;
  double max_steer_rad = 0.55;
  double max_steer_rate_radps = 2.0;
  double steer_time_constant_s = 0.08;
  int encoder_counts_per_revolution = 2048;
  // Steering servo stall + tire-road friction (engine contact pair and
  // slip-flag threshold). Defaults mirror main's Ackermann/GroundContact.
  double steer_stall_torque_Nm = 8.0;
  double tire_friction_mu = 0.7;

  // Factory defaults per chassis kind (same numbers make_chassis builds
  // with). False when kind is unknown: no silent default hardware.
  static bool for_chassis(const std::string& kind, DriveProfile& out);
};

}  // namespace lost_signal::robot::platform
