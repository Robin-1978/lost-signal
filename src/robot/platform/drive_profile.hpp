#pragma once

#include <string>
#include <vector>

namespace lost_signal::robot::platform {

// Hardware capability contract shared by native bring-up and SIM: one set
// of numbers means sim and native close the same loops against the same
// plant. Geometry matches the chassis factory defaults; drive motors are
// the DC force curve (stall force + free speed) integrated against the
// per-wheel equivalent mass, so the speed lag is emergent, never fitted;
// steers are velocity servos. The hardware contract file
// ("lost-signal-robot/1") loads on both sides: native via
// robot::platform::load_robot_config, SIM-side via sim/eval translation
// (drive_profile_io merges legacy flat files the same way). ROBOT never
// reads SIM truth — only this contract.
//
// Adapting to a DIFFERENT real machine is config-only, never code:
// every per-machine number lives in "drive" + "chassis" (units, frames
// and defaults below; configs/robot/*.json are the worked examples).
// What stays in code on purpose: engine solver tuning (floor friction,
// torsional/rolling contact constants — MuJoCo-side, machine-independent)
// and the fail-closed guards (a bad machine file refuses with frozen
// truth instead of building a different plant). Rules of thumb:
//  - omit a chassis block (or any key) = documented default, never zero;
//  - "supports": [...] fully explicit wins; omitted = legacy ball pair;
//  - com_height_m/payload_height_m 0 = follow-axle / chassis-COM;
//  - belly line = com - half_h must stay > 0 (loader AND build enforce);
//  - deck conserves the vehicle total: deck = vehicle - wheels - supports.
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
  // Skid-steer yaw scrub (tracked only, main's TrackedDrive scrub model):
  // Coulomb + viscous torque opposing yaw from track scrub. Zero for
  // wheel chassis (differential pivots freely); tracked sets both.
  double yaw_scrub_coulomb_Nm = 0.0;
  double yaw_scrub_viscous_Nm_per_radps = 0.0;

  // Chassis hard geometry (SIM plant assembly reads every field; nothing
  // is hardcoded per layout anymore). Defaults reproduce the legacy plant
  // with a 0.05 m shell half-height, centered axle and 0.04 m tire half-width,
  // fore/aft ball pair resolved at conversion time from wheel_base_m).
  double body_half_height_m = 0.05;  // collision shell, centered at axle
  double body_half_length_m = 0.28;  // collision/visual shell, X forward
  double body_half_width_m = 0.25;   // collision/visual shell, Y left
  double drive_axle_x_m = 0.0;       // drive axle line offset in base frame
                                     // (diff pair + ackermann rears shift;
                                     // 4-wheel layouts ignore it)
  double tire_half_width_m = 0.04;   // cylinder tread half width
  // Center-of-mass height above ground (deck ballast center). 0 = follow
  // the axle (wheel_radius_m — legacy behavior, COM at axle height).
  // Explicit values reposition the ballast; the belly guard below applies
  // to the effective height in both cases.
  double com_height_m = 0.0;
  // Static payload (declared loadout, e.g. cargo/sensor mast — NOT game
  // items; there is no runtime collection path). Folded master-style:
  // total = vehicle + payload, COM = weighted average, inertia by
  // parallel axis (payload modeled as a point mass at its height — exact
  // for mass and COM, approximate for intrinsic inertia, documented at
  // the fold). Traction load (mu * N) uses the total, like the engine.
  double payload_mass_kg = 0.0;    // 0 = unladen (legacy behavior)
  double payload_height_m = 0.0;   // above ground; 0 = chassis COM, i.e.
                                   // pure ballast (COM height unchanged)
  // Ackermann idler trail (wheel center behind the kingpin — zero trail
  // is neutrally stable and solver noise walks it into a wedge; trail
  // makes straight-ahead the stable equilibrium).
  double idler_trail_m = 0.02;
  // Static mass budget. The contact shell is massless; an internal ballast
  // is positioned so the complete vehicle (all wheels, knuckles and supports)
  // has exactly vehicle_mass_kg and com_height_m. wheel_mass_kg is the exact
  // mass of every drive or idler wheel, never a separate density estimate.
  double wheel_mass_kg = 0.2;
  double knuckle_mass_kg = 0.1;      // steering kingpin (steered layouts)
  // Unpowered supports, base frame, explicit (empty = legacy fore/aft ball
  // pair at +-0.9*wheel_base/2 — the real differential base doctrine).
  struct SupportSpec {
    std::string kind = "ball";  // only "ball" (fail closed otherwise)
    double x_m = 0.0;
    double radius_m = 0.03;
    double mass_kg = 0.1;
    double friction_mu = 0.05;  // near-zero tangential by authoring: normal
    double torsional = 0.005;   // load only, deliberately ungripped
    double rolling = 0.0005;
  };
  std::vector<SupportSpec> supports;

  // Factory defaults per chassis kind (same numbers make_chassis builds
  // with). False when kind is unknown: no silent default hardware.
  static bool for_chassis(const std::string& kind, DriveProfile& out);
};

}  // namespace lost_signal::robot::platform
