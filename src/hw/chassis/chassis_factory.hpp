#pragma once

#include "hw/chassis/motor_drive.hpp"

#include <functional>
#include <map>
#include <memory>
#include <string>

namespace sim {

// Chassis factory registry (extension seam for player hardware).
// New chassis type = implement ChassisModel + one REGISTER_CHASSIS line in
// its own translation unit + a drive.type string in robot JSON. No factory
// switch to extend, no mission/UI edits. The registry maps type strings to
// creators; unknown types fail loudly at construction (same contract as the
// retired if/else chain, now data-driven).
//
// Layering: creators take neutral numbers only (ChassisBuildParams), never
// game types — hw stays independent of game. The caller (mission) translates
// RobotDefinition + equipped specs once; per-chassis assembly rules
// (servo derivation, yaw authority) live with their implementations.

// Every number a chassis implementation needs, resolved by the caller
// (equipped-component overrides applied upstream, like today).
struct ChassisBuildParams {
  double wheel_base_m{0.5};
  double wheel_radius_m{0.1};
  double track_width_m{0.58};
  double track_length_m{0.7};
  double max_wheel_speed_mps{2.4};
  double max_wheel_accel_mps2{6.0};
  double stall_force_N{54.0};
  double free_speed_mps{3.0};
  double pwm_limit{1.0};
  double integral_limit{2.0};
  double wheel_equiv_mass_kg{3.0};
  int encoder_counts_per_revolution{2048};
  double mass_kg{26.0};
  double yaw_inertia_kgm2{2.8};
  double max_steer_rad{0.6};
  double max_steer_rate_radps{2.5};
  double servo_response_s{0.4};
  double steering_integral_limit{1.2};
  double tracked_max_yaw_rate_rads{1.5};
  // Mission behavior inputs (not hardware): differential yaw cap source and
  // the accel clamp both chassis share with the behavior layer.
  double behavior_w_max{2.0};
  double behavior_accel_max{6.0};
};

struct BuiltChassis {
  std::unique_ptr<ChassisModel> model;
  double yaw_cap_rads{2.0};
};

using ChassisCreator = std::function<BuiltChassis(const ChassisBuildParams&, const WheelPidGains&)>;

class ChassisFactory {
 public:
  static void register_creator(const std::string& drive_type, ChassisCreator creator);
  // Throws std::runtime_error on unknown drive_type (loud, never fallback).
  static BuiltChassis create(const std::string& drive_type, const ChassisBuildParams& params,
                             const WheelPidGains& gains);
  // Explicit builtin registration (one call line per chassis type below).
  // Static registrars were tried and reverted: archive linkers drop
  // unreferenced object files, so self-registration is platform luck.
  // Explicit beats magic; adding a type = creator in its own TU + one line.
  static void register_builtin_chassis();

 private:
  static std::map<std::string, ChassisCreator>& registry();
};

// One per chassis type, defined beside its implementation. Declared here so
// register_builtin_chassis has exactly one line per type, nothing hidden.
BuiltChassis create_differential_chassis(const ChassisBuildParams& params,
                                         const WheelPidGains& gains);
BuiltChassis create_ackermann_chassis(const ChassisBuildParams& params,
                                      const WheelPidGains& gains);
BuiltChassis create_tracked_chassis(const ChassisBuildParams& params,
                                    const WheelPidGains& gains);

} // namespace sim
