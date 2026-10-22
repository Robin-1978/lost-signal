#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "robot/iface/motion.hpp"
#include "sim/world/actor_behavior.hpp"
#include "sim/world/contact.hpp"
#include "sim/world/mujoco_backend.hpp"
#include "sim/world/service_brake.hpp"

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
  double body_z_m = 0.0;      // engine height (analytic plant: ground)
  double body_roll_rad = 0.0;   // engine roll (analytic: level)
  double body_pitch_rad = 0.0;  // engine pitch (analytic: level)
  std::uint8_t drive_count = 0;
  std::uint8_t steer_count = 0;
  std::array<double, 4> drive_radps = {};  // mechanical rad/s
  std::array<double, 4> drive_angle_rad = {};  // continuous joint angle
  std::array<double, 4> drive_duty = {};
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
struct StaticBox {
  double x0 = 0.0, y0 = 0.0, x1 = 0.0, y1 = 0.0;
  double height_m = 0.0;  // surveyed deck height; ground is 0
};

struct Actor {
  double x = 0.0, y = 0.0, vx_mps = 0.0, vy_mps = 0.0, radius_m = 0.4;
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
  // Encoder resolution for the joint measurement model (counts per
  // revolution). The provider quantizes angles and differentiates; the
  // ROBOT loop closes on the quantized joints, like native hardware.
  int encoder_counts_per_revolution = 2048;
};

class World {
 public:
  explicit World(PhysicsBackend backend = PhysicsBackend::kAnalytic);
  void step(double dt_s);
  void set_motor_params(const MotorParams& params);
  const MotorParams& motor_params() const { return motor_; }
  PhysicsBackend backend() const { return backend_; }
  // SIM-side scenario/test hook (not a ROBOT channel): normalized parking /
  // e-stop style brake request per drive wheel, clamped to [0, 1].
  // Out-of-range wheels are ignored. Reconfiguring the motor params resets
  // the brake lag plants.
  void set_brake_request(int wheel, double normalized);
  // SIM-side scenario/test hook (not a ROBOT channel): longitudinal grade
  // (rise/run along body-forward, + means uphill forward). Gravity pulls
  // through the contact patch: stiction holds gentle grades exactly,
  // steep grades break away. Default 0 (flat). Lateral grade needs S3's
  // lateral tire state and is rejected here (fail closed, never silently
  // dropped into the longitudinal patch).
  void set_incline(double forward_grade);
  // Actuator-level plant input: normalized duties per the command mode.
  // Wheel speeds first-order track duty-implied targets; steers servo
  // toward duty-implied angles; the body follows by chassis kinematics.
  void apply(const robot::iface::ActuatorCommand& cmd);
  const WorldTruth& truth() const { return truth_; }

  void set_static_boxes(std::vector<StaticBox> boxes) {
    static_boxes_ = std::move(boxes);
    ++geometry_version_;
  }
  const std::vector<StaticBox>& static_boxes() const { return static_boxes_; }
  std::uint64_t geometry_version() const { return geometry_version_; }
  void add_actor(const Actor& actor) { actors_.push_back(actor); }
  const std::vector<Actor>& actors() const { return actors_; }
  void add_place(const NamedPlace& place) {
    places_.push_back(place);
    ++geometry_version_;
  }
  const std::vector<NamedPlace>& places() const { return places_; }

 private:
  WorldTruth truth_{};
  MotorParams motor_{};
  PhysicsBackend backend_ = PhysicsBackend::kAnalytic;
  MujocoBackend mujoco_;
  // MuJoCo branch (M1 diff only): syncs the engine model, integrates, and
  // reads truth back. Invalid backend (unsupported mode, broken model, or
  // MuJoCo disabled at configure time) leaves truth frozen — never silent
  // analytic substitution. dt <= 0 holds state (no analytic-style snap).
  void step_mujoco(double dt_s);
  std::array<ServiceBrakePlant, 4> brakes_;
  std::array<double, 4> brake_request_ = {};
  double incline_grade_ = 0.0;  // longitudinal rise/run, SIM-side only
  robot::iface::ActuatorMode last_mode_ = robot::iface::ActuatorMode::kDiffDuty;
  std::vector<StaticBox> static_boxes_;
  std::vector<Actor> actors_;
  std::vector<NamedPlace> places_;
  std::uint64_t geometry_version_ = 0;
};

}  // namespace lost_signal::sim::world
