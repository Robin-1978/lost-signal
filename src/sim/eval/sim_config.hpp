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

struct ObjectiveAcceptanceConfig {
 int version = 2;
 double position_tolerance_m = 0.25;
 double heading_tolerance_rad = 0.2;
 double maximum_linear_speed_mps = 0.08;
 double maximum_yaw_rate_radps = 0.08;
 // Two consecutive post-physics samples reject a one-frame crossing while
 // fitting the ROBOT arrival policy's final braking/settling transition.
 // This is SIM evidence only; it never changes ROBOT completion.
 std::uint32_t stable_steps = 2;
 // After ROBOT reports Done, SIM continues observing the stopped plant for
 // this bounded interval. This lets actuator/brake lag complete before the
 // independent truth verdict; it never reopens the ROBOT goal or feeds truth
 // into control. Expiry records the live failure reasons.
 double terminal_observation_s = 0.5;
};

struct SimConfig {
 double scan_period_s = 0.02;
 double motion_period_s = 0.02; // encoder/twist sample cadence
 double imu_period_s = 0.02; // IMU sample cadence
 // Control/safety tick cadence. The coordinator's expensive planning branch
 // has its own cadence below, so raw-evidence safety can remain at 50 Hz.
 double decision_period_s = 0.02;
 double planning_period_s = 0.02;
  double record_period_s = 0.02;
  double judge_radius_m = 0.4;
 ObjectiveAcceptanceConfig objective_acceptance;
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
