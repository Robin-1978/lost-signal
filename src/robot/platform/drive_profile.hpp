#pragma once

#include <string>

namespace lost_signal::robot::platform {

// Hardware capability contract shared by native bring-up and SIM: one set
// of numbers means sim and native close the same loops against the same
// plant. Geometry matches the chassis factory defaults; motors are
// first-order (free speed + time constant); steers are velocity servos.
// JSON file loading lives on the SIM side (sim/eval/drive_profile_io),
// which merges a file over these defaults — ROBOT never parses files.
struct DriveProfile {
  std::string chassis_kind = "differential";
  double wheel_radius_m = 0.1;
  double wheel_base_m = 0.56;
  double track_width_m = 0.5;
  double free_speed_mps = 3.0;
  double motor_time_constant_s = 0.12;
  double max_steer_rad = 0.55;
  double max_steer_rate_radps = 2.0;
  double steer_time_constant_s = 0.08;
  int encoder_counts_per_revolution = 2048;

  // Factory defaults per chassis kind (same numbers make_chassis builds
  // with). False when kind is unknown: no silent default hardware.
  static bool for_chassis(const std::string& kind, DriveProfile& out);
};

}  // namespace lost_signal::robot::platform
