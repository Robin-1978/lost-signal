#pragma once

// MuJoCo physics backend (SIM internals, never ROBOT input). The backend
// owns rigid-body integration: duties become joint efforts through the
// main-stack electric-drive + service-brake math, contacts resolve in the
// engine, and truth is read back from engine state for the providers to
// translate.
//
// Determinism contract: fixed 1 ms substeps, single-threaded, no noise.
// Same duties + same steps = bit-identical truth, verified by the M1
// parity tests. Anything nondeterministic fails the backend, not the test.
#ifdef LOST_SIGNAL_WITH_MUJOCO
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#endif

namespace lost_signal::sim::world {

struct MotorParams;
struct StaticBox;
struct Actor;
struct MaterialEntry;
struct MaterialZone;
struct TerrainPatch;
struct SupportSlab;
struct EnvironmentKeyframe;

#ifdef LOST_SIGNAL_WITH_MUJOCO

class MujocoBackend {
 public:
  MujocoBackend();
  ~MujocoBackend();
  MujocoBackend(const MujocoBackend&) = delete;
  MujocoBackend& operator=(const MujocoBackend&) = delete;
  MujocoBackend(MujocoBackend&&);
  MujocoBackend& operator=(MujocoBackend&&);

  bool valid() const { return ready_; }
  // Rebuilds the model when geometry or plant numbers change; no-op
  // otherwise. M1: differential bases only (2 drives, 0 steers); anything
  // else leaves valid() false and the base frozen (M2), never silently
  // approximated.
  // Terrain/material import (SIM-side truth only): authored patches become
  // oriented contact slabs, material zones become per-zone floor slabs with
  // the library friction/contact numbers. Mesh patches (roughness or
  // bilinear cross-slope) build as tile grids sampling the same canonical
  // height the survey rasterizes — authored, never smoothed. Environment
  // keyframes are NOT build inputs — time-varying grip applies per step.
  void sync(const MotorParams& motor, const std::vector<StaticBox>& boxes,
            const std::vector<MaterialEntry>& materials,
            const std::vector<MaterialZone>& zones,
            const std::vector<TerrainPatch>& patches,
            const std::vector<SupportSlab>& slabs,
            std::uint64_t geometry_version, int drive_count, int steer_count);
  // One plant tick: efforts from duties (duty-implied motor force + brake
  // capacity against shaft velocity, dissipative only), gravity tilted by
  // the longitudinal grade around the current heading, 1 ms substeps.
  // Per-wheel tire friction follows the local material zone (main's
  // tire-x-ground resolution, engine-native: the pair takes the max, so
  // the tire side carries min(global, local) and the ground side the zone
  // value) times the environment grip at sim_time_s. Slip flags confirm
  // over two consecutive steps (hysteresis against single-substep spikes).
  void step(double dt_s, const MotorParams& motor,
            const std::array<double, 4>& drive_duty,
            const std::array<double, 4>& steer_duty,
            const std::array<double, 4>& brake_request, double incline_grade,
            const std::vector<MaterialZone>& zones,
            const std::vector<MaterialEntry>& materials,
            const std::vector<EnvironmentKeyframe>& environment,
            double sim_time_s);

  // Engine state read-back (world frame, SI). Called after step().
  double body_x() const { return body_x_; }
  double body_y() const { return body_y_; }
  double body_yaw() const { return body_yaw_; }
  double forward_mps() const { return forward_mps_; }
  double lateral_mps() const { return lateral_mps_; }
  double yaw_rate() const { return yaw_rate_; }
  double drive_angle(int i) const { return drive_angle_[i]; }
  double drive_radps(int i) const { return drive_radps_[i]; }
  bool drive_slipping(int i) const { return drive_slipping_[i]; }
  double steer_angle(int i) const { return steer_angle_[i]; }
  double steer_vel(int i) const { return steer_vel_[i]; }
  double body_z() const { return body_z_; }
  double body_roll() const { return body_roll_; }
  double body_pitch() const { return body_pitch_; }
  // True 3D raycast against the compiled static scene (floor + boxes),
  // for sensor simulation. Robot geoms live in group 1 and are excluded,
  // so a sensor mounted on the body never self-hits; actors have no engine
  // bodies and are supplemented analytically by the caller (same split as
  // main's game lidar). False when the model is invalid or nothing is hit
  // within max_m (caller falls back to max range). Deterministic for the
  // same compiled state; call between steps like everything else.
  bool raycast(double ox, double oy, double oz, double dx, double dy,
               double dz, double max_m, double& dist_m) const;

 private:
  void read_back();

  struct Impl;
  std::unique_ptr<Impl> impl_;
  // Cached build inputs; mismatch triggers a rebuild on next sync().
  std::string built_signature_;
  std::uint64_t built_geometry_version_ = 0;
  bool ever_built_ = false;
  // Geometry needed by the per-step roller wrench.
  double track_m_ = 0.5;
  double base_len_m_ = 0.56;
  // Drive-wheel module positions in the base frame (per-step zone lookup).
  double drive_mod_x_[4] = {0.0, 0.0, 0.0, 0.0};
  double drive_mod_y_[4] = {0.0, 0.0, 0.0, 0.0};
  // Tire geom ids per drive wheel (per-step friction writes, -1 unset).
  int tire_geom_[4] = {-1, -1, -1, -1};
  // Zone floor geom ids in zone order (per-step environment writes).
  std::vector<int> zone_geom_;
  // Tire-side sliding baseline (min(global, local) anchor for the step).
  double mu_tire_base_ = 0.7;
  // Slip confirmation hysteresis: consecutive over-traction steps.
  int slip_count_[4] = {0, 0, 0, 0};
  // Fail-closed refuse reporting: loud once per bad authoring, quiet after.
  bool refuse_reported_ = false;
  // Drive joint DOF addresses (drive_count <= 4 in M2).
  int drive_dof_[4] = {-1, -1, -1, -1};
  int drive_count_ = 0;
  // Steer joint DOF addresses (swerve/ackermann knuckles, M2).
  int steer_dof_[4] = {-1, -1, -1, -1};
  int steer_count_ = 0;
  bool ready_ = false;
  // Latest read-back.
  double body_x_ = 0.0, body_y_ = 0.0, body_yaw_ = 0.0;
  double forward_mps_ = 0.0, lateral_mps_ = 0.0, yaw_rate_ = 0.0;
  double body_z_ = 0.0, body_roll_ = 0.0, body_pitch_ = 0.0;
  std::array<double, 4> drive_angle_ = {};
  std::array<double, 4> drive_radps_ = {};
  std::array<bool, 4> drive_slipping_ = {};
  std::array<double, 4> steer_angle_ = {};
  std::array<double, 4> steer_vel_ = {};
};

#else  // !LOST_SIGNAL_WITH_MUJOCO

// MuJoCo disabled at configure time: the backend exists as a typed
// placeholder that reports invalid, so World code compiles either way.
// The engine is the only plant; there is no fallback integration.
class MujocoBackend {
 public:
  bool valid() const { return false; }
  void sync(const MotorParams&, const std::vector<StaticBox>&,
            const std::vector<MaterialEntry>&, const std::vector<MaterialZone>&,
            const std::vector<TerrainPatch>&, const std::vector<SupportSlab>&,
            std::uint64_t, int, int) {}
  void step(double, const MotorParams&, const std::array<double, 4>&,
            const std::array<double, 4>&, const std::array<double, 4>&, double,
            const std::vector<MaterialZone>&, const std::vector<MaterialEntry>&,
            const std::vector<EnvironmentKeyframe>&, double) {}
  bool raycast(double, double, double, double, double, double, double,
               double&) const {
    return false;
  }
  double body_x() const { return 0.0; }
  double body_y() const { return 0.0; }
  double body_yaw() const { return 0.0; }
  double forward_mps() const { return 0.0; }
  double lateral_mps() const { return 0.0; }
  double yaw_rate() const { return 0.0; }
  double drive_angle(int) const { return 0.0; }
  double drive_radps(int) const { return 0.0; }
  bool drive_slipping(int) const { return false; }
  double steer_angle(int) const { return 0.0; }
  double steer_vel(int) const { return 0.0; }
  double body_z() const { return 0.0; }
  double body_roll() const { return 0.0; }
  double body_pitch() const { return 0.0; }
};

#endif  // LOST_SIGNAL_WITH_MUJOCO

}  // namespace lost_signal::sim::world
