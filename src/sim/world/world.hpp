#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "robot/iface/motion.hpp"
#include "sim/world/actor_behavior.hpp"
#include "sim/world/mujoco_backend.hpp"

namespace lost_signal::sim::world {

// SIM-owned ground truth. Must never be included from src/robot/.
// Physics, actors, contacts, and scoring inputs live here.
//
// Joint layout mirrors the actuator mode that produced it: diff 2 drives,
// ackermann 2 drives + 2 steers, swerve 4 + 4, mecanum 4 drives.
// drive_count == 0 means the plant has not yet received a command.
struct WorldTruth {
  double body_x_m = 0.0;
  double body_y_m = 0.0;
  double body_yaw_rad = 0.0;
  double forward_mps = 0.0;
  double lateral_mps = 0.0;   // holonomic body-left velocity
  double yaw_rate_radps = 0.0;  // derived from joints, like a driver
  double body_z_m = 0.0;      // engine height above the ground plane
  double body_roll_rad = 0.0;   // engine roll
  double body_pitch_rad = 0.0;  // engine pitch
  std::uint8_t drive_count = 0;
  std::uint8_t steer_count = 0;
  std::array<double, 4> drive_radps = {};  // mechanical rad/s
  std::array<double, 4> drive_angle_rad = {};  // continuous joint angle
  std::array<double, 4> drive_duty = {};
  // Per-wheel traction saturation flag (engine truth, offline only):
  // motor force exceeded mu * load. Reported, never clamped (main's duty
  // path is pure motor curve; clamping lives on its PID path, not here).
  std::array<bool, 4> drive_slipping = {};
  std::array<double, 4> steer_rad = {};
  std::array<double, 4> steer_duty = {};
  std::array<double, 4> steer_vel_radps = {};
  std::uint64_t step = 0;

  // Diff conveniences for legacy tests (drive_count == 2).
  double left_radps() const { return drive_count > 0 ? drive_radps[0] : 0.0; }
  double right_radps() const { return drive_count > 1 ? drive_radps[1] : 0.0; }
};

// Authored scenario geometry (static) and moving actors (dynamic).
// Raycast sources, never ROBOT inputs: the suite turns them into measured
// scans (full raycast) and a static reference (static geometry only).
// A box is a true 3D prism: footprint [x0,x1]x[y0,y1] extruded to z in
// [0, height_m] above the ground plane. The engine builds that prism,
// the lidar raycasts it from the true 3D sensor pose; the ROBOT only
// ever sees the resulting ranges and must perceive the shape itself.
struct StaticBox {
  double x0 = 0.0, y0 = 0.0, x1 = 0.0, y1 = 0.0;
  double height_m = 0.0;  // prism top above ground; <= 0 = flat paint
                          // (floor marking, no volume: no contact, no ray
                          // hit, no map occupancy, no judge collision).
};

struct Actor {
  double x = 0.0, y = 0.0, vx_mps = 0.0, vy_mps = 0.0, radius_m = 0.4;
  double height_m = 1.5;  // vertical extent for sensor raycast (upright
                          // cylinder z in [0, height]); physics stays planar
                          // (actors are kinematic, no engine bodies).
  // Motion policy, evaluated every step before integration. Default flies
  // the authored velocity (yesterday's straight lines, unchanged).
  ActorBehavior behavior;
};

// Authored named place: semantic task target resolved by the ROBOT map
// provider, never a raw coordinate crossing from SIM.
struct NamedPlace {
  std::string place_id;
  double x_m = 0.0, y_m = 0.0, yaw_rad = 0.0;
};

// Authored operator/dock prior for the ROBOT map frame. This is deployment
// configuration, not runtime physics truth: ScenarioSession may publish it
// through the same initial-pose API used on a real robot. The physical plant
// is never queried to fill or correct this value.
struct InitialPosePrior {
 double x_m = 0.0;
 double y_m = 0.0;
 double yaw_rad = 0.0;
 double position_sigma_m = 0.10;
 double heading_sigma_rad = 0.05;
};

// Ground material library entry (SIM-side physics truth, never ROBOT
// input). One entry resolves all contact + sensing numbers for every zone
// carrying its id: zones store the id only, values resolve at load, so a
// retune touches one record. Presentation (textures, effects) is out:
// physics contact + lidar reflectivity only.
struct MaterialEntry {
  std::string material_id;
  double friction_mu = 0.8;  // tire-road sliding friction (engine pair)
  double rolling_resist = 0.0;  // combined tire+ground Coulomb coefficient
  double contact_time_constant_s = 0.006;  // solver softness (main default)
  double contact_damping_ratio = 1.0;
  double lidar_reflectivity = 0.30;  // return strength (provider-side only)
};

// Rectangular ground-material override. Later entries win on overlap.
// Values live in the library; the zone only names the material.
struct MaterialZone {
  std::string zone_id;
  double x0 = 0.0, y0 = 0.0, x1 = 0.0, y1 = 0.0;
  std::string material_id;
};

// Authored terrain height patch (SIM-side truth). Plane + bilinear
// cross-slope + deterministic relief, sampled by the single canonical
// function in terrain.hpp (engine geometry and survey elevation share it,
// never drift apart). Nonlinear patches share one engine height field;
// planar patches use exact analytic slabs.
struct TerrainPatch {
  std::string patch_id;
  double x0 = 0.0, y0 = 0.0, x1 = 0.0, y1 = 0.0;
  double base_height_m = 0.0;
  double grade_x = 0.0;
  double grade_y = 0.0;
  double grade_xy_per_m = 0.0;  // bilinear cross-slope transition
  double roughness = 0.0;       // T2: relief waveform amplitude
};

// Bounded oriented support slab (bridge/deck): later revisions overlay
// authored patches wherever the footprint exists. Static authored
// geometry only — no construction gameplay (out of scope).
struct SupportSlab {
  std::string slab_id;
  double center_x_m = 0.0, center_y_m = 0.0;
  double half_length_m = 0.0, half_width_m = 0.0;
  double yaw_rad = 0.0;
  double center_height_m = 0.0;
  double grade_along_local_x = 0.0;
};

// Environment keyframe: simulation-time-interpolated grip multiplier per
// material (live wetness, SIM-side only — never a ROBOT field). Missing
// materials stay 1.0; ends clamp.
struct EnvironmentKeyframe {
  double time_s = 0.0;
  std::vector<std::pair<std::string, double>> material_grip;
};

struct MotorParams {
  double free_speed_mps = 3.0;  // no-load linear speed at duty 1
  double stall_force_N = 54.0;  // peak motor force at zero speed (duty 1)
  // Per-wheel equivalent translational mass (wheel + reflected rotor).
  // Same numbers the ROBOT feedforward closes against (sim-native parity).
  double wheel_equiv_mass_kg = 3.0;
  // Rolling contact (S2): Coulomb+viscous tire/ground model shared with the
  // ROBOT feedforward form. Load derives from total mass (gravity is
  // physics, applied below); rr folds tire+ground (no surface model yet).
  double vehicle_mass_kg = 24.0;
  double rolling_resist = 0.01;
  double viscous_drag_N_per_mps = 0.6;
  double coulomb_smoothing_mps = 0.05;
  double wheel_radius_m = 0.1;
  double track_width_m = 0.5;
  double wheel_base_m = 0.56;
  double max_steer_rad = 0.55;
  double max_steer_rate_radps = 2.0;
  double steer_time_constant_s = 0.08;  // servo lag to rate-implied angle
  // Service-brake capacity; 0 disables (default). The lag plants still step
  // but offer no torque, so untouched behavior is coast-only.
  double brake_max_torque_Nm = 0.0;
  double brake_apply_time_s = 0.12;
  double brake_release_time_s = 0.08;
  // Steering servo stall torque (velocity-servo model, same form as main's
  // Ackermann steering_torque). DriveProfile default mirrors main.
  double steer_stall_torque_Nm = 8.0;
  // Tire-road sliding friction for the engine contact pair and the slip
  // flag threshold (mu * load per wheel). Mirrors main's GroundContact.
  double tire_friction_mu = 0.7;
  // Skid-steer yaw scrub torque (tracked): Coulomb + viscous, opposing yaw.
  // Zero for wheeled chassis (differential pivots freely) — today's
  // behavior bit-identical. From the DriveProfile track contract.
  double yaw_scrub_coulomb_Nm = 0.0;
  double yaw_scrub_viscous_Nm_per_radps = 0.0;
  // Chassis hard geometry (from the DriveProfile chassis contract; the
  // engine build reads these, never hardcoded per-layout constants).
  // Defaults reproduce the legacy plant bit-identically.
  double body_half_height_m = 0.05;  // deck ballast half height
  double body_half_width_m = 0.12;   // deck ballast half extent
  double drive_axle_x_m = 0.0;       // drive axle line offset (base frame)
  double tire_half_width_m = 0.04;   // cylinder tread half width
  // Static mass budget (deck conserves the total, floor 0.5).
  double wheel_mass_kg = 0.2;        // per-drive-wheel static budget
  double knuckle_mass_kg = 0.1;      // steering kingpin (steered layouts)
  double tire_density_kgpm3 = 400.0;  // tire rubber density
  // Effective COM height above ground (0 = follow the axle at convert).
  double com_height_m = 0.0;
  // Static payload (declared loadout; folded master-style at the build).
  double payload_mass_kg = 0.0;
  double payload_height_m = 0.0;  // 0 = chassis COM (pure ballast)
  // Ackermann idler trail (wheel center behind the kingpin).
  double idler_trail_m = 0.02;
  // Unpowered supports (base frame, explicit). Empty = legacy fore/aft
  // ball pair, resolved at conversion from wheel_base_m.
  struct SupportParams {
    std::string kind = "ball";
    double x_m = 0.0;
    double radius_m = 0.03;
    double mass_kg = 0.1;
    double friction_mu = 0.05;
    double torsional = 0.005;
    double rolling = 0.0005;
  };
  static constexpr int kMaxSupports = 4;
  SupportParams supports[kMaxSupports];
  int support_count = 0;  // 0 = legacy default pair (conversion fills it)
  // Encoder resolution for the joint measurement model (counts per
  // revolution). The provider quantizes angles and differentiates; the
  // ROBOT loop closes on the quantized joints, like native hardware.
  int encoder_counts_per_revolution = 2048;
};

class World {
 public:
  World();
  void step(double dt_s);
  double sim_time_s() const { return sim_time_s_; }
  void set_motor_params(const MotorParams& params);
  const MotorParams& motor_params() const { return motor_; }
  // SIM-side scenario/test hook (not a ROBOT channel): normalized parking /
  // e-stop style brake request per drive wheel, clamped to [0, 1].
  // Out-of-range wheels are ignored. Reconfiguring the motor params resets
  // the brake lag plants.
  void set_brake_request(int wheel, double normalized);
  // SIM-side scenario/test hook (not a ROBOT channel): longitudinal grade
  // (rise/run along body-forward, + means uphill forward). Gravity pulls
  // through the contact patch: the engine holds gentle grades bounded,
  // steep grades break away. Default 0 (flat).
  void set_incline(double forward_grade);
  // Actuator-level plant input: normalized duties per the command mode.
  // Resolved in the engine at 1 ms substeps; the body follows from contact.
  void apply(const robot::iface::ActuatorCommand& cmd);
  const WorldTruth& truth() const { return truth_; }
  // Full wheel table for presentation (backend layout + live joints, same
  // sources as physics/encoders). Returns filled count.
  int wheels(WheelViz* out, int cap) const {
    return mujoco_.wheels(out, cap);
  }
 // Passive caster state for the SIM viewer. This never enters the ROBOT
 // observation or actuator path.
 int casters(CasterViz* out, int cap) const {
 return mujoco_.casters(out, cap);
 }
  // Sensor raycast against the compiled static scene (floor + boxes) for
  // provider simulation. False when the engine is invalid or nothing is
  // hit within max_m. Actors are supplemented analytically by the caller.
  bool raycast(double ox, double oy, double oz, double dx, double dy,
               double dz, double max_m, double& dist_m) const {
    return mujoco_.raycast(ox, oy, oz, dx, dy, dz, max_m, dist_m);
  }

  void set_static_boxes(std::vector<StaticBox> boxes) {
    static_boxes_ = std::move(boxes);
    ++geometry_version_;
  }
  const std::vector<StaticBox>& static_boxes() const { return static_boxes_; }
  std::uint64_t geometry_version() const { return geometry_version_; }
  // Authored terrain + materials (SIM-side truth, same revision discipline:
  // every setter bumps the geometry version, rebuilding engine + survey).
  void set_materials(std::vector<MaterialEntry> materials) {
    materials_ = std::move(materials);
    ++geometry_version_;
  }
  const std::vector<MaterialEntry>& materials() const { return materials_; }
  void set_material_zones(std::vector<MaterialZone> zones) {
    material_zones_ = std::move(zones);
    ++geometry_version_;
  }
  const std::vector<MaterialZone>& material_zones() const {
    return material_zones_;
  }
  void set_terrain_patches(std::vector<TerrainPatch> patches) {
    terrain_patches_ = std::move(patches);
    ++geometry_version_;
  }
  const std::vector<TerrainPatch>& terrain_patches() const {
    return terrain_patches_;
  }
  void set_support_slabs(std::vector<SupportSlab> slabs) {
    support_slabs_ = std::move(slabs);
    ++geometry_version_;
  }
  const std::vector<SupportSlab>& support_slabs() const {
    return support_slabs_;
  }
  void set_environment(std::vector<EnvironmentKeyframe> environment) {
    environment_ = std::move(environment);
    ++geometry_version_;
  }
  const std::vector<EnvironmentKeyframe>& environment() const {
    return environment_;
  }
  void add_actor(const Actor& actor) { actors_.push_back(actor); }
  const std::vector<Actor>& actors() const { return actors_; }
  void add_place(const NamedPlace& place) {
    places_.push_back(place);
    ++geometry_version_;
  }
  // Editor/scene bulk replace (SIM-side only). Mirrors set_static_boxes.
  void set_places(std::vector<NamedPlace> places) {
    places_ = std::move(places);
    ++geometry_version_;
  }
  const std::vector<NamedPlace>& places() const { return places_; }
 void set_initial_pose_prior(std::optional<InitialPosePrior> prior) {
 initial_pose_prior_ = std::move(prior);
 }
 const std::optional<InitialPosePrior>& initial_pose_prior() const {
 return initial_pose_prior_;
 }

 private:
  WorldTruth truth_{};
  MotorParams motor_{};
  MujocoBackend mujoco_;
  double sim_time_s_ = 0.0;  // accumulated plant dt: environment grip clock
  // The engine owns integration: syncs the model, steps, reads truth back.
  // Unsupported mode, broken model, or MuJoCo disabled at configure time
  // leaves truth frozen; dt <= 0 holds state. Actors stay kinematic
  // (no engine bodies for them); the judge reads the same truth.
  void step_engine(double dt_s);
  std::array<double, 4> brake_request_ = {};
  double incline_grade_ = 0.0;  // longitudinal rise/run, SIM-side only
  std::vector<StaticBox> static_boxes_;
  std::vector<MaterialEntry> materials_;
  std::vector<MaterialZone> material_zones_;
  std::vector<TerrainPatch> terrain_patches_;
  std::vector<SupportSlab> support_slabs_;
  std::vector<EnvironmentKeyframe> environment_;
  std::vector<Actor> actors_;
  std::vector<NamedPlace> places_;
 std::optional<InitialPosePrior> initial_pose_prior_;
  std::uint64_t geometry_version_ = 0;
};

}  // namespace lost_signal::sim::world
