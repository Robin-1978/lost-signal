#pragma once

// SIM-only configuration (never ROBOT input, never native-deployable):
// engine/orchestration knobs with no hardware meaning — multi-rate
// cadence, judge radius, delivery shaping + IMU realism seeds, lidar
// geometry. The hardware contract (plant numbers, mounts, limits) lives
// in the robot config; this file only tunes how the simulation runs.
// All sections optional; absent sections keep current code defaults.

#include <cstdint>
#include <optional>
#include <string>

#include "sim/providers/sensor_suite.hpp"

namespace lost_signal::sim::eval {

inline constexpr char kSimConfigFormat[] = "lost-signal-sim/1";

struct SimConfig {
 double scan_period_s = 0.02;
 double motion_period_s = 0.02; // encoder/twist sample cadence
 double imu_period_s = 0.02; // IMU sample cadence
 double decision_period_s = 0.02;  // ROBOT decision cadence in simulation time
  double record_period_s = 0.02;
  double judge_radius_m = 0.4;
  providers::DeliveryConfig delivery;      // all zeros = passthrough
  providers::ImuRealismConfig imu_realism;  // all zeros = exact truth
  int lidar_beams = 72;
  float lidar_max_range_m = 10.0f;
  int lidar_rings = 16;  // 3D producer ring count; azimuths reuse beams
  bool lidar_composite = false;  // multi-line composite scan (opt-in front end)
};

// Fail-closed load: untouched output on any mismatch.
bool load_sim_config(const std::string& path, SimConfig& config,
                     std::string* error);
// Tool default chain, symmetric to the robot config: explicit path wins;
// otherwise ./configs/sim/default.json when it exists; otherwise nullopt
// and the caller keeps code defaults. Never fails hard.
std::optional<SimConfig> load_default_sim_config(
    const std::string& explicit_path, std::string* used_path,
    std::string* error);

}  // namespace lost_signal::sim::eval
