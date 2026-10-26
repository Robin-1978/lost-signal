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
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#ifdef LOST_SIGNAL_WITH_MUJOCO
#include <memory>
#include <string>
#include <vector>
#endif

namespace lost_signal::sim::world {

// Contact materials are deliberately coarse: they describe visible impact
// behavior, while friction and compliance remain separate physical fields.
// The enum keeps the 1 kHz contact path allocation-free and typo-proof.
enum class ImpactMaterial {
 kUnknown,
 kRubber,
 kMetal,
 kConcrete,
 kGlass,
 kWood,
 kPolymer,
};

struct MotorParams;
struct StaticBox;
struct Actor;
struct MaterialEntry;
struct MaterialZone;
struct TerrainPatch;
struct SupportSlab;
struct EnvironmentKeyframe;

// Typed generation for inputs that can change the compiled MuJoCo model.
// World advances it only from model-input setters; dynamic environment,
// actors, tasks, controls and truth never participate. Keeping it distinct
// from the survey geometry revision prevents map-only edits from entering
// the 1 kHz model-build path.
struct MujocoModelRevision {
  std::uint64_t value = 0;

  friend bool operator==(const MujocoModelRevision&,
                         const MujocoModelRevision&) = default;
};

// SIM-side performance diagnostics. Tests can enforce the constant-time
// generation gate without depending on allocator or wall-clock noise.
struct MujocoBuildStats {
  std::uint64_t sync_requests = 0;
  std::uint64_t fast_path_hits = 0;
  std::uint64_t signature_serializations = 0;
  std::uint64_t rebuild_attempts = 0;
  std::uint64_t successful_rebuilds = 0;
};

// Rigid link pose expressed in the MuJoCo base-body frame. This is SIM-only
// presentation state read from mjData::xpos/xquat after stepping; renderers
// must consume it directly instead of reconstructing joints or lifting parts
// against a separately sampled terrain surface.
struct LinkPoseViz {
 bool valid = false;
 double x_m = 0.0, y_m = 0.0, z_m = 0.0;
 double qw = 1.0, qx = 0.0, qy = 0.0, qz = 0.0;
};

// Articulated joint frame read directly from MuJoCo. Origin and axis are
// expressed in the base-link frame for an exact engineering overlay.
struct JointAxisViz {
 bool valid = false;
 LinkPoseViz link_pose;
 double axis_x = 0.0, axis_y = 0.0, axis_z = 1.0;
 double angle_rad = 0.0, rate_radps = 0.0;
};

// One wheel module for presentation (base frame): position from the shared
// layout table, spin/steer from live joint read-back. Idlers (ackermann
// fronts) report driven=false. Rollers are not state (mecanum is an
// analytic wrench) and stay out — the viewer decorates those statically.
struct WheelViz {
  double x_m = 0.0, y_m = 0.0;
 double spin_rad = 0.0, spin_rate_radps = 0.0, steer_rad = 0.0;
 double steer_rate_radps = 0.0;
  bool slipping = false, driven = false;
 bool contact_valid = false;
 double contact_x_m = 0.0, contact_y_m = 0.0, contact_z_m = 0.0;
 double contact_slip_speed_mps = 0.0;
 double contact_longitudinal_slip_mps = 0.0;
 double contact_lateral_slip_mps = 0.0;
 // World-space vectors resolved on the physical contact tangent plane.
 // motion is the wheel module translation relative to the static surface;
 // slip is the tire contact-point relative velocity after removing its
 // normal component. They are SIM presentation evidence only.
 double contact_motion_x_mps = 0.0;
 double contact_motion_y_mps = 0.0;
 double contact_motion_z_mps = 0.0;
 double contact_slip_x_mps = 0.0;
 double contact_slip_y_mps = 0.0;
 double contact_slip_z_mps = 0.0;
 double contact_normal_force_N = 0.0;
 // Tangential contact force divided by the active MuJoCo friction-cone
 // capacity. One means the contact is saturated. SIM truth only.
 double contact_friction_utilization = 0.0;
 LinkPoseViz pose;
 JointAxisViz steer_joint;
 JointAxisViz drive_joint;
};

// One passive caster assembly for presentation.  The pivot and wheel
// positions are in the base frame; swivel/roll are read directly from the
// MuJoCo joints.  This is SIM-only state and is deliberately absent from the
// ROBOT observation/actuator contract.
struct CasterViz {
 double pivot_x_m = 0.0, pivot_y_m = 0.0;
 double x_m = 0.0, y_m = 0.0;
 double radius_m = 0.03;
 double swivel_rad = 0.0, roll_rad = 0.0;
 LinkPoseViz fork_pose;
 LinkPoseViz wheel_pose;
 JointAxisViz swivel_joint;
 JointAxisViz roll_joint;
};

// Aggregate non-ground impact over one public plant step. MuJoCo contact
// forces are integrated across every fixed substep, yielding a true normal
// impulse rather than a collision boolean. Presentation may consume this;
// ROBOT interfaces never expose it.
struct ImpactViz {
 bool valid = false;
 double x_m = 0.0, y_m = 0.0, z_m = 0.0;
 double normal_impulse_Ns = 0.0;
 double relative_speed_mps = 0.0;
 // A separate aggregate prevents an ineligible rubber/concrete contact from
 // borrowing the position or impulse of a simultaneous metal/metal contact.
 bool spark_eligible = false;
 double spark_x_m = 0.0, spark_y_m = 0.0, spark_z_m = 0.0;
 double spark_normal_impulse_Ns = 0.0;
 double spark_relative_speed_mps = 0.0;
 ImpactMaterial spark_robot_material = ImpactMaterial::kUnknown;
 ImpactMaterial spark_static_material = ImpactMaterial::kUnknown;
};

// Every active MuJoCo contact for the current plant state. World-space data
// is presentation evidence only; the ROBOT boundary receives ordinary sensor
// observations and never this truth-side record.
struct ContactViz {
 bool valid = false;
 double x_m = 0.0, y_m = 0.0, z_m = 0.0;
 double normal_x = 0.0, normal_y = 0.0, normal_z = 1.0;
 double normal_force_N = 0.0;
};

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
 // otherwise. Supported layouts are differential (2,0), Ackermann (2,2),
 // swerve (4,4), mecanum (4,0), plus passive caster assemblies. Unsupported
 // layouts leave valid() false and the plant frozen; they are never silently
 // approximated.
 // Terrain/material import (SIM-side truth only): exact planar patches use
 // analytic slabs; nonlinear relief, cross-slope, and depressions use one
 // continuous height field sampled from the same canonical function as the
 // survey. Material zones are contact properties resolved at each wheel,
 // never coplanar collision slabs. Environment keyframes are not build
 // inputs; time-varying grip applies per step.
  void sync(const MotorParams& motor, const std::vector<StaticBox>& boxes,
            const std::vector<MaterialEntry>& materials,
            const std::vector<MaterialZone>& zones,
            const std::vector<TerrainPatch>& patches,
            const std::vector<SupportSlab>& slabs,
            MujocoModelRevision model_revision, int drive_count,
            int steer_count);
  // One plant tick: efforts from duties (duty-implied motor force + brake
  // capacity against shaft velocity, dissipative only), gravity tilted by
  // the longitudinal grade around the current heading, 1 ms substeps.
  // Per-wheel tire friction follows the local material zone (main's
  // tire-x-ground resolution, engine-native: the pair takes the max, so
  // the tire side carries min(global, local) and the ground side the zone
 // value) times the environment grip at sim_time_s. Slip flags require
 // saturation for the contact response duration, filtering transients.
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
  // Full wheel table for presentation (drives by drive_idx, then steering
  // idlers): positions from the shared layout table, spin/steer from live
  // joint read-back, slip from the traction flags. Ball supports are
  // SIM-internal (normal load only) and stay out of this robot-facing
  // table — the ROBOT sees driven/idler wheels, never support state.
  // One source for viz — the same joints the encoder provider and the
  // physics read, so the three cannot disagree. Returns filled count
  // (cap 6: 4 drives + 2 idlers, the largest layout). Idlers report
  // driven=false, slipping=false (unpowered, unflagged — honest, never
  // invented).
  int wheels(WheelViz* out, int cap) const {
    if (out == nullptr || cap <= 0) return 0;
    int n = 0;
    for (int d = 0; d < drive_count_ && d < 4 && n < cap; ++d) {
      WheelViz w;
      w.x_m = drive_mod_x_[d];
      w.y_m = drive_mod_y_[d];
      w.spin_rad = drive_angle_[d];
 w.spin_rate_radps = drive_radps_[d];
      const int s = (d < 4) ? drive_steer_[d] : -1;
      w.steer_rad = (s >= 0 && s < 4) ? steer_angle_[s] : 0.0;
 w.steer_rate_radps = (s >= 0 && s < 4) ? steer_vel_[s] : 0.0;
      w.slipping = drive_slipping_[static_cast<std::size_t>(d)];
      w.driven = true;
 w.contact_valid = drive_contact_valid_[d];
 w.contact_x_m = drive_contact_x_[d];
 w.contact_y_m = drive_contact_y_[d];
 w.contact_z_m = drive_contact_z_[d];
 w.contact_slip_speed_mps = drive_contact_slip_mps_[d];
 w.contact_longitudinal_slip_mps = drive_contact_longitudinal_slip_mps_[d];
 w.contact_lateral_slip_mps = drive_contact_lateral_slip_mps_[d];
 w.contact_motion_x_mps = drive_contact_motion_x_mps_[d];
 w.contact_motion_y_mps = drive_contact_motion_y_mps_[d];
 w.contact_motion_z_mps = drive_contact_motion_z_mps_[d];
 w.contact_slip_x_mps = drive_contact_slip_x_mps_[d];
 w.contact_slip_y_mps = drive_contact_slip_y_mps_[d];
 w.contact_slip_z_mps = drive_contact_slip_z_mps_[d];
 w.contact_normal_force_N = drive_contact_normal_N_[d];
 w.contact_friction_utilization = drive_contact_friction_utilization_[d];
 w.pose = drive_pose_[d];
 w.drive_joint = drive_joint_[d];
 if (s >= 0 && s < 4) w.steer_joint = steer_joint_[s];
      out[n++] = w;
    }
    for (int j = 0; j < idler_count_ && j < 2 && n < cap; ++j) {
      WheelViz w;
      w.x_m = idler_mod_x_[j];
      w.y_m = idler_mod_y_[j];
      w.spin_rad = idler_angle_[j];
 w.spin_rate_radps = idler_radps_[j];
      w.steer_rad = (j < 4) ? steer_angle_[j] : 0.0;
 w.steer_rate_radps = (j < 4) ? steer_vel_[j] : 0.0;
 w.contact_valid = idler_contact_valid_[j];
 w.contact_x_m = idler_contact_x_[j];
 w.contact_y_m = idler_contact_y_[j];
 w.contact_z_m = idler_contact_z_[j];
 w.contact_slip_speed_mps = idler_contact_slip_mps_[j];
 w.contact_longitudinal_slip_mps = idler_contact_longitudinal_slip_mps_[j];
 w.contact_lateral_slip_mps = idler_contact_lateral_slip_mps_[j];
 w.contact_motion_x_mps = idler_contact_motion_x_mps_[j];
 w.contact_motion_y_mps = idler_contact_motion_y_mps_[j];
 w.contact_motion_z_mps = idler_contact_motion_z_mps_[j];
 w.contact_slip_x_mps = idler_contact_slip_x_mps_[j];
 w.contact_slip_y_mps = idler_contact_slip_y_mps_[j];
 w.contact_slip_z_mps = idler_contact_slip_z_mps_[j];
 w.contact_normal_force_N = idler_contact_normal_N_[j];
 w.contact_friction_utilization = idler_contact_friction_utilization_[j];
 w.pose = idler_pose_[j];
 if (j < 4) w.steer_joint = steer_joint_[j];
 w.drive_joint = idler_joint_[j];
      out[n++] = w;
    }
    return n;
  }
 int casters(CasterViz* out, int cap) const {
 if (out == nullptr || cap <= 0) return 0;
 int n = 0;
 for (int i = 0; i < support_count_ && i < 4 && n < cap; ++i) {
 CasterViz c;
 c.pivot_x_m = support_anchor_x_[i];
 c.pivot_y_m = support_anchor_y_[i];
 c.swivel_rad = support_swivel_[i];
 c.roll_rad = support_roll_[i];
 c.radius_m = support_radius_[i];
 const double ct = std::cos(c.swivel_rad);
 const double st = std::sin(c.swivel_rad);
 c.x_m = c.pivot_x_m - support_trail_[i] * ct;
 c.y_m = c.pivot_y_m - support_trail_[i] * st;
 c.fork_pose = support_fork_pose_[i];
 c.wheel_pose = support_wheel_pose_[i];
 c.swivel_joint = support_swivel_joint_[i];
 c.roll_joint = support_roll_joint_[i];
 out[n++] = c;
 }
 return n;
 }
 int joint_axes(JointAxisViz* out, int cap) const {
 if (out == nullptr || cap <= 0) return 0;
 int n = 0;
 const auto append = [&](const JointAxisViz& axis) {
 if (n >= cap || !axis.valid) return;
 out[n++] = axis;
 };
 for (int i = 0; i < drive_count_ && i < 4; ++i) append(drive_joint_[i]);
 for (int i = 0; i < steer_count_ && i < 4; ++i) append(steer_joint_[i]);
 for (int i = 0; i < idler_count_ && i < 2; ++i) append(idler_joint_[i]);
 for (int i = 0; i < support_count_ && i < 4; ++i) {
 append(support_swivel_joint_[i]);
 append(support_roll_joint_[i]);
 }
 return n;
 }
 int contacts(ContactViz* out, int cap) const {
 if (out == nullptr || cap <= 0) return 0;
 const int n = std::min(cap, contact_viz_count_);
 for (int i = 0; i < n; ++i) out[i] = contact_viz_[i];
 return n;
 }
 const ImpactViz& impact() const { return impact_; }
  double body_z() const { return body_z_; }
  double body_roll() const { return body_roll_; }
  double body_pitch() const { return body_pitch_; }
 double total_mass_kg() const { return total_mass_kg_; }
 double system_com_x_m() const { return system_com_x_m_; }
 double system_com_y_m() const { return system_com_y_m_; }
 double system_com_z_m() const { return system_com_z_m_; }
 // COM expressed in the live base-link frame. The legacy system_com_*()
 // values remain world-frame telemetry for compatibility; presentation uses
 // this frame-local triplet so a tilted/rotating chassis keeps the marker
 // attached to the physical mass centre.
 double system_com_relative_x_m() const { return system_com_relative_x_m_; }
 double system_com_relative_y_m() const { return system_com_relative_y_m_; }
 double system_com_relative_z_m() const { return system_com_relative_z_m_; }
  const MujocoBuildStats& build_stats() const { return build_stats_; }
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
 void read_contacts();

  struct Impl;
  std::unique_ptr<Impl> impl_;
  // Check the generation/layout key before inspecting model inputs. A changed
  // key permits one signature serialization; the signature then prevents a
  // redundant rebuild for equivalent compiled content.
  bool has_sync_key_ = false;
  MujocoModelRevision synced_model_revision_{};
  int synced_drive_count_ = 0;
  int synced_steer_count_ = 0;
  std::string built_signature_;
  bool ever_built_ = false;
  MujocoBuildStats build_stats_{};
  // Geometry needed by the coupled Mecanum roller constraint.
  int base_qpos_ = -1;
  int base_dof_ = -1;
  double track_m_ = 0.5;
  double base_len_m_ = 0.56;
  // Drive-wheel module positions in the base frame (per-step zone lookup).
  double drive_mod_x_[4] = {0.0, 0.0, 0.0, 0.0};
  double drive_mod_y_[4] = {0.0, 0.0, 0.0, 0.0};
  // Steer joint per drive module (-1 = fixed wheel), from the shared
  // layout table (swerve aligns by construction; ackermann rears read -1).
  int drive_steer_[4] = {-1, -1, -1, -1};
  // Steering-idler modules (ackermann fronts): knuckle positions for viz
  // + free-spin read-back (idlerj hinges, same order as the steer joints).
  double idler_mod_x_[2] = {0.0, 0.0};
  double idler_mod_y_[2] = {0.0, 0.0};
  double idler_angle_[2] = {0.0, 0.0};
 double idler_radps_[2] = {0.0, 0.0};
 bool idler_contact_valid_[2] = {false, false};
 double idler_contact_x_[2] = {0.0, 0.0};
 double idler_contact_y_[2] = {0.0, 0.0};
 double idler_contact_z_[2] = {0.0, 0.0};
 double idler_contact_slip_mps_[2] = {0.0, 0.0};
 double idler_contact_longitudinal_slip_mps_[2] = {0.0, 0.0};
 double idler_contact_lateral_slip_mps_[2] = {0.0, 0.0};
 double idler_contact_motion_x_mps_[2] = {0.0, 0.0};
 double idler_contact_motion_y_mps_[2] = {0.0, 0.0};
 double idler_contact_motion_z_mps_[2] = {0.0, 0.0};
 double idler_contact_slip_x_mps_[2] = {0.0, 0.0};
 double idler_contact_slip_y_mps_[2] = {0.0, 0.0};
 double idler_contact_slip_z_mps_[2] = {0.0, 0.0};
 double idler_contact_normal_N_[2] = {0.0, 0.0};
 double idler_contact_friction_utilization_[2] = {0.0, 0.0};
  int idler_dof_[2] = {-1, -1};
 int idler_body_[2] = {-1, -1};
 LinkPoseViz idler_pose_[2];
 JointAxisViz idler_joint_[2];
  int idler_count_ = 0;
 // Passive caster assemblies.  The fork swivel and wheel roll are real
 // MuJoCo hinges; these values are read back for SIM presentation only.
 double support_anchor_x_[4] = {0.0, 0.0, 0.0, 0.0};
 double support_anchor_y_[4] = {0.0, 0.0, 0.0, 0.0};
 double support_radius_[4] = {0.03, 0.03, 0.03, 0.03};
 double support_trail_[4] = {0.02, 0.02, 0.02, 0.02};
 double support_swivel_[4] = {0.0, 0.0, 0.0, 0.0};
 double support_roll_[4] = {0.0, 0.0, 0.0, 0.0};
 int support_swivel_dof_[4] = {-1, -1, -1, -1};
 int support_roll_dof_[4] = {-1, -1, -1, -1};
 int support_fork_body_[4] = {-1, -1, -1, -1};
 int support_wheel_body_[4] = {-1, -1, -1, -1};
 LinkPoseViz support_fork_pose_[4];
 LinkPoseViz support_wheel_pose_[4];
 JointAxisViz support_swivel_joint_[4];
 JointAxisViz support_roll_joint_[4];
 int support_geom_[4] = {-1, -1, -1, -1};
 int support_shoulder_geom_[4][2] = {{-1, -1}, {-1, -1}, {-1, -1},
 {-1, -1}};
 double support_mu_base_[4] = {0.05, 0.05, 0.05, 0.05};
  double support_rolling_base_m_[4] = {0.0005, 0.0005, 0.0005, 0.0005};
 int support_count_ = 0;
  // Tire geom ids per drive wheel (per-step friction writes, -1 unset).
  int tire_geom_[4] = {-1, -1, -1, -1};
  // Idler tire geom ids (per-step zone follow, same doctrine as drives —
  // a frozen baseline would anchor the plant on slick ground).
  int idler_geom_[2] = {-1, -1};
  // Zone floor geom ids in zone order (per-step environment writes).
  std::vector<int> zone_geom_;
 std::vector<int> static_geom_;
 std::vector<ImpactMaterial> static_impact_material_;
  // Tire-side sliding baseline (min(global, local) anchor for the step).
  double mu_tire_base_ = 0.7;
 // Mecanum support cylinders resolve normal load only. The authored tire
 // coefficient separately bounds the coupled 45-degree roller impulses.
  double mecanum_traction_mu_base_ = 0.7;
  double mecanum_traction_mu_[4] = {0.7, 0.7, 0.7, 0.7};
 // Current pre-solve support load used to bound roller impulses. The
 // acceleration-only contact prepass refreshes it every fixed quantum.
  double mecanum_normal_load_N_[4] = {0.0, 0.0, 0.0, 0.0};
 // Slip confirmation is a physical duration, independent of how callers
 // group canonical engine ticks around sensor deadlines.
 double slip_saturated_time_s_[4] = {0.0, 0.0, 0.0, 0.0};
 double slip_confirmation_time_s_[4] = {0.0, 0.0, 0.0, 0.0};
 double slip_unobserved_time_s_[4] = {0.0, 0.0, 0.0, 0.0};
  // Fail-closed refuse reporting: loud once per bad authoring, quiet after.
  bool refuse_reported_ = false;
  // Drive joint DOF addresses (drive_count <= 4 in M2).
  int drive_dof_[4] = {-1, -1, -1, -1};
 int drive_body_[4] = {-1, -1, -1, -1};
 LinkPoseViz drive_pose_[4];
 JointAxisViz drive_joint_[4];
 JointAxisViz steer_joint_[4];
  int drive_count_ = 0;
  // Steer joint DOF addresses (swerve/ackermann knuckles, M2).
  int steer_dof_[4] = {-1, -1, -1, -1};
 int steer_body_[4] = {-1, -1, -1, -1};
  int steer_count_ = 0;
  bool ready_ = false;
  // Latest read-back.
  double body_x_ = 0.0, body_y_ = 0.0, body_yaw_ = 0.0;
  double forward_mps_ = 0.0, lateral_mps_ = 0.0, yaw_rate_ = 0.0;
  double body_z_ = 0.0, body_roll_ = 0.0, body_pitch_ = 0.0;
 double total_mass_kg_ = 0.0;
 double system_com_x_m_ = 0.0, system_com_y_m_ = 0.0;
 double system_com_z_m_ = 0.0;
 double system_com_relative_x_m_ = 0.0, system_com_relative_y_m_ = 0.0;
 double system_com_relative_z_m_ = 0.0;
  std::array<double, 4> drive_angle_ = {};
  std::array<double, 4> drive_radps_ = {};
  std::array<bool, 4> drive_slipping_ = {};
 bool drive_contact_valid_[4] = {false, false, false, false};
 double drive_contact_x_[4] = {0.0, 0.0, 0.0, 0.0};
 double drive_contact_y_[4] = {0.0, 0.0, 0.0, 0.0};
 double drive_contact_z_[4] = {0.0, 0.0, 0.0, 0.0};
 double drive_contact_slip_mps_[4] = {0.0, 0.0, 0.0, 0.0};
 double drive_contact_longitudinal_slip_mps_[4] = {0.0, 0.0, 0.0, 0.0};
 double drive_contact_lateral_slip_mps_[4] = {0.0, 0.0, 0.0, 0.0};
 double drive_contact_motion_x_mps_[4] = {0.0, 0.0, 0.0, 0.0};
 double drive_contact_motion_y_mps_[4] = {0.0, 0.0, 0.0, 0.0};
 double drive_contact_motion_z_mps_[4] = {0.0, 0.0, 0.0, 0.0};
 double drive_contact_slip_x_mps_[4] = {0.0, 0.0, 0.0, 0.0};
 double drive_contact_slip_y_mps_[4] = {0.0, 0.0, 0.0, 0.0};
 double drive_contact_slip_z_mps_[4] = {0.0, 0.0, 0.0, 0.0};
 double drive_contact_normal_N_[4] = {0.0, 0.0, 0.0, 0.0};
 double drive_contact_friction_utilization_[4] = {0.0, 0.0, 0.0, 0.0};
  std::array<double, 4> steer_angle_ = {};
  std::array<double, 4> steer_vel_ = {};
 ImpactViz impact_;
 std::array<ContactViz, 64> contact_viz_{};
 int contact_viz_count_ = 0;
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
            MujocoModelRevision, int, int) {}
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
  int wheels(WheelViz*, int) const { return 0; }
  double body_z() const { return 0.0; }
  double body_roll() const { return 0.0; }
  double body_pitch() const { return 0.0; }
 double total_mass_kg() const { return 0.0; }
 double system_com_x_m() const { return 0.0; }
 double system_com_y_m() const { return 0.0; }
 double system_com_z_m() const { return 0.0; }
 double system_com_relative_x_m() const { return 0.0; }
 double system_com_relative_y_m() const { return 0.0; }
 double system_com_relative_z_m() const { return 0.0; }
  const MujocoBuildStats& build_stats() const { return build_stats_; }
 int casters(CasterViz*, int) const { return 0; }
 int joint_axes(JointAxisViz*, int) const { return 0; }
 int contacts(ContactViz*, int) const { return 0; }
 const ImpactViz& impact() const { return impact_; }

 private:
 ImpactViz impact_;
  MujocoBuildStats build_stats_{};
};

#endif  // LOST_SIGNAL_WITH_MUJOCO

}  // namespace lost_signal::sim::world
