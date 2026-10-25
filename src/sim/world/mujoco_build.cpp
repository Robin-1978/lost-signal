// MuJoCo plant assembly: spec helpers (bodies/joints/geoms, layout
// table) plus MujocoBackend::sync (model build from MotorParams). The
// per-step loop lives in mujoco_step.cpp.
#include "sim/world/mujoco_backend.hpp"
#include "sim/world/mujoco_backend_detail.hpp"

#ifdef LOST_SIGNAL_WITH_MUJOCO

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <mujoco/mujoco.h>

#include "sim/world/motor.hpp"
#include "sim/world/service_brake.hpp"
#include "sim/world/terrain.hpp"
#include "sim/world/world.hpp"

namespace lost_signal::sim::world {

namespace {


// Engine-native contact constants. Sliding grip comes from the drive
// profile (tire_friction_mu, main's GroundContact default 0.7); torsional
// and rolling lengths stay constant (main derives them from content the
// clean tree does not carry yet — M3).
constexpr double kWheelTorsionalFriction = 0.01;
constexpr double kWheelRollingFriction = 0.002;
constexpr double kFloorSlidingFriction = 1.0;
constexpr double kQuatIdentity[4] = {1.0, 0.0, 0.0, 0.0};
// 90 degrees about X (w,x,y,z): lays a z-axis cylinder onto the y axle.
constexpr double kQuatX90[4] = {0.7071067811865476, 0.7071067811865476, 0.0,
                                0.0};

struct JointStateSnapshot {
  std::string name;
  int type = -1;
  std::vector<double> qpos;
  std::vector<double> qvel;
};

int joint_qpos_width(int type) {
  switch (type) {
    case mjJNT_FREE: return 7;
    case mjJNT_BALL: return 4;
    case mjJNT_SLIDE:
    case mjJNT_HINGE: return 1;
    default: return 0;
  }
}

int joint_qvel_width(int type) {
  switch (type) {
    case mjJNT_FREE: return 6;
    case mjJNT_BALL: return 3;
    case mjJNT_SLIDE:
    case mjJNT_HINGE: return 1;
    default: return 0;
  }
}

std::vector<JointStateSnapshot> capture_joint_state(
    const mjModel* model, const mjData* data) {
  std::vector<JointStateSnapshot> out;
  if (model == nullptr || data == nullptr) return out;
  out.reserve(static_cast<std::size_t>(model->njnt));
  for (int joint = 0; joint < model->njnt; ++joint) {
    const char* name = mj_id2name(model, mjOBJ_JOINT, joint);
    if (name == nullptr || name[0] == '\0') continue;
    const int type = model->jnt_type[joint];
    const int nq = joint_qpos_width(type);
    const int nv = joint_qvel_width(type);
    if (nq <= 0 || nv <= 0) continue;
    const int qpos = model->jnt_qposadr[joint];
    const int qvel = model->jnt_dofadr[joint];
    JointStateSnapshot state;
    state.name = name;
    state.type = type;
    state.qpos.assign(data->qpos + qpos, data->qpos + qpos + nq);
    state.qvel.assign(data->qvel + qvel, data->qvel + qvel + nv);
    out.push_back(std::move(state));
  }
  return out;
}

void restore_joint_state(const std::vector<JointStateSnapshot>& snapshots,
    const mjModel* model, mjData* data) {
  for (const auto& state : snapshots) {
    const int joint = mj_name2id(model, mjOBJ_JOINT, state.name.c_str());
    if (joint < 0 || model->jnt_type[joint] != state.type) continue;
    const int nq = joint_qpos_width(state.type);
    const int nv = joint_qvel_width(state.type);
    if (static_cast<int>(state.qpos.size()) != nq ||
        static_cast<int>(state.qvel.size()) != nv) continue;
    std::copy(state.qpos.begin(), state.qpos.end(),
        data->qpos + model->jnt_qposadr[joint]);
    std::copy(state.qvel.begin(), state.qvel.end(),
        data->qvel + model->jnt_dofadr[joint]);
  }
}

void set_name(mjsElement* element, const std::string& name) {
  mjs_setName(element, name.c_str());
}

mjsBody* add_body(mjsBody* parent, const std::string& name, double x,
                  double y, double z) {
  mjsBody* body = mjs_addBody(parent, nullptr);
  set_name(body->element, name);
  body->pos[0] = x;
  body->pos[1] = y;
  body->pos[2] = z;
  return body;
}

mjsJoint* add_hinge(mjsBody* parent, const std::string& name, double ax,
                    double ay, double az, double armature = 0.0) {
  mjsJoint* joint = mjs_addJoint(parent, nullptr);
  set_name(joint->element, name);
  joint->type = mjJNT_HINGE;
  joint->axis[0] = ax;
  joint->axis[1] = ay;
  joint->axis[2] = az;
  joint->armature = armature;
  return joint;
}

mjsGeom* add_geom(mjsBody* parent, const std::string& name, mjtGeom type,
                 double sx, double sy, double sz, int group = 0,
                 const double* quat = nullptr) {
  mjsGeom* geom = mjs_addGeom(parent, nullptr);
  set_name(geom->element, name);
  geom->type = type;
  geom->group = group;
 if (group == 1) {
 // Robot links collide with world geoms (default contype 1), while links
 // inside the same articulated robot never collide with each other. This
 // permits a faithful outer body shell with wheel cut-throughs represented
 // by the articulated tires instead of shrinking the chassis to a tiny tub.
 geom->contype = 2;
 geom->conaffinity = 1;
 }
  geom->size[0] = sx;
  geom->size[1] = sy;
  geom->size[2] = sz;
  if (quat != nullptr) {
    geom->quat[0] = quat[0];
    geom->quat[1] = quat[1];
    geom->quat[2] = quat[2];
    geom->quat[3] = quat[3];
  } else {
    geom->quat[0] = kQuatIdentity[0];
    geom->quat[1] = kQuatIdentity[1];
    geom->quat[2] = kQuatIdentity[2];
    geom->quat[3] = kQuatIdentity[3];
  }
  return geom;
}

// Quaternion multiply, (w,x,y,z) convention.
void quat_mul(const double a[4], const double b[4], double out[4]) {
  out[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
  out[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
  out[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
  out[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

// Orientation leveling a slab top onto the authored grade. Sign check:
// R_y(-a) sends +X upward with slope tan(a); R_x(b) sends +Y upward with
// slope tan(b). Applied as q = qy * qx (pitch first in the local frame).
void grade_quat(double grade_x, double grade_y, double out[4]) {
  const double p = -std::atan(grade_x);
  const double r = std::atan(grade_y);
  const double qy[4] = {std::cos(p * 0.5), 0.0, std::sin(p * 0.5), 0.0};
  const double qx[4] = {std::cos(r * 0.5), std::sin(r * 0.5), 0.0, 0.0};
  quat_mul(qy, qx, out);
}

// Wheel modules per layout: (x, y, steered) in base frame, with drive and
// steer channel assignment in loop order. Shared by the build (bodies) and
// the step (wheel world positions for the zone lookup): one table, no drift.
struct PlacedModule {
  double x = 0.0, y = 0.0;
  bool steered = false;
  bool idle = false;  // free-spinning idler (ackermann fronts)
  bool is_drive = false;
  bool is_steer = false;
  int drive_idx = -1;
  int steer_idx = -1;
};

std::vector<PlacedModule> layout_modules(int drive_count, int steer_count,
                                         double track, double base_len,
                                         double axle_x) {
  std::vector<PlacedModule> modules;
 modules.reserve(4);
  if (drive_count == 2 && steer_count == 0) {
    modules.push_back({axle_x, track / 2});
    modules.push_back({axle_x, -track / 2});
  } else if (drive_count == 2 && steer_count == 2) {
    // Rear drives + front steered IDLERS (free-spinning, no torque):
    // the two drive duties are the rear axle, like the main stack.
    modules.push_back({-base_len / 2 + axle_x, track / 2});
    modules.push_back({-base_len / 2 + axle_x, -track / 2});
    modules.push_back({base_len / 2, track / 2, true, true});
    modules.push_back({base_len / 2, -track / 2, true, true});
  } else {
    const double sx[4] = {base_len / 2, base_len / 2, -base_len / 2,
                          -base_len / 2};
    const double sy[4] = {track / 2, -track / 2, track / 2, -track / 2};
    for (int i = 0; i < 4; ++i) {
      modules.push_back({sx[i], sy[i], steer_count == 4});
    }
  }
  int drive_idx = 0, steer_idx = 0;
  for (std::size_t m = 0; m < modules.size(); ++m) {
    const bool is_drive = !modules[m].idle && (drive_idx < drive_count) &&
                          ((drive_count == 2 && steer_count == 2)
                               ? (static_cast<int>(m) < 2)
                               : true);
    const bool is_steer =
        modules[m].steered && steer_idx < steer_count;
    modules[m].is_drive = is_drive;
    modules[m].is_steer = is_steer;
    if (is_drive) modules[m].drive_idx = drive_idx++;
    if (is_steer) modules[m].steer_idx = steer_idx++;
  }
  return modules;
}

}  // namespace

void MujocoBackend::sync(const MotorParams& motor,
                          const std::vector<StaticBox>& boxes,
                          const std::vector<MaterialEntry>& materials,
                          const std::vector<MaterialZone>& zones,
                          const std::vector<TerrainPatch>& patches,
                          const std::vector<SupportSlab>& slabs,
                          std::uint64_t geometry_version, int drive_count,
                          int steer_count) {
  // Supported layouts mirror the actuator modes: (2,0) diff, (2,2)
  // ackermann, (4,4) swerve, (4,0) mecanum. Anything else is not
  // approximated.
  const bool supported =
      (drive_count == 2 && steer_count == 0) ||
      (drive_count == 2 && steer_count == 2) ||
      (drive_count == 4 && steer_count == 4) ||
      (drive_count == 4 && steer_count == 0);
  if (!supported) {
    drive_count_ = 0;
    steer_count_ = 0;
    ready_ = false;
    return;
  }
  // Mesh patches (roughness > 0 or bilinear cross-slope) build as tile
  // grids below (piecewise-planar, same top-coincidence construction per
  // tile): the surface is authored, never silently smoothed. Only unknown
  // zone materials refuse the build fail-closed.
  for (const auto& z : zones) {
    if (find_material(materials, z.material_id) == nullptr) {
      if (!refuse_reported_) {
        std::fprintf(stderr,
                     "mujoco_backend: zone '%s' references unknown material "
                     "'%s'; build refused\n",
                     z.zone_id.c_str(), z.material_id.c_str());
        refuse_reported_ = true;
      }
      drive_count_ = 0;
      steer_count_ = 0;
      ready_ = false;
      return;
    }
  }
  std::ostringstream sig;
  sig << motor.free_speed_mps << "|" << motor.stall_force_N << "|"
      << motor.vehicle_mass_kg << "|" << motor.wheel_radius_m << "|"
      << motor.track_width_m << "|" << motor.wheel_base_m << "|"
      << motor.brake_max_torque_Nm << "|" << motor.brake_apply_time_s << "|"
      << motor.brake_release_time_s << "|" << motor.steer_stall_torque_Nm
      << "|" << motor.max_steer_rad << "|" << motor.max_steer_rate_radps
      << "|" << motor.tire_friction_mu << "|" << motor.viscous_drag_N_per_mps
      << "|" << motor.wheel_equiv_mass_kg << "|" << motor.rolling_resist
      << "|" << motor.coulomb_smoothing_mps << "|"
      << motor.encoder_counts_per_revolution << "|"
      << motor.yaw_scrub_coulomb_Nm << "|" << motor.yaw_scrub_viscous_Nm_per_radps
      // Chassis hard geometry: every build input participates, or a
      // config switch keeps the stale model (found live: axle/support
      // changes rebuilt nothing). Supports itemized per entry.
 << "|" << motor.body_half_height_m << "|" << motor.body_half_length_m
 << "|" << motor.body_half_width_m
      << "|" << motor.drive_axle_x_m << "|" << motor.tire_half_width_m
 << "|" << motor.wheel_mass_kg << "|" << motor.knuckle_mass_kg
 << "|" << motor.com_height_m
      << "|" << motor.payload_mass_kg << "|" << motor.payload_height_m
      << "|" << motor.idler_trail_m << "|" << motor.support_count << "|";
  for (int i = 0; i < motor.support_count && i < MotorParams::kMaxSupports;
       ++i) {
    const auto& s = motor.supports[i];
    sig << s.kind << "," << s.x_m << "," << s.y_m << "," << s.trail_m
        << "," << s.radius_m << "," << s.mass_kg << "," << s.friction_mu
        << "," << s.swivel_damping_Nms << "," << s.swivel_friction_Nm
        << "," << s.rolling_friction_m
        << ";";
  }
  sig << "|" << drive_count << "|" << steer_count << "|" << boxes.size()
      << "|";
  for (const auto& b : boxes) {
    sig << b.x0 << "," << b.y0 << "," << b.x1 << "," << b.y1 << "," << b.height_m
        << ";";
  }
  sig << "M|";
  for (const auto& m : materials) {
    sig << m.material_id << "," << m.friction_mu << "," << m.rolling_resist
        << "," << m.contact_time_constant_s << "," << m.contact_damping_ratio
        << ";";
  }
  sig << "Z|";
  for (const auto& z : zones) {
    sig << z.zone_id << "," << z.x0 << "," << z.y0 << "," << z.x1 << ","
        << z.y1 << "," << z.material_id << ";";
  }
  sig << "T|";
  for (const auto& p : patches) {
    sig << p.patch_id << "," << p.x0 << "," << p.y0 << "," << p.x1 << ","
        << p.y1 << "," << p.base_height_m << "," << p.grade_x << ","
        << p.grade_y << "," << p.grade_xy_per_m << "," << p.roughness << ";";
  }
  sig << "S|";
  for (const auto& s : slabs) {
    sig << s.slab_id << "," << s.center_x_m << "," << s.center_y_m << ","
        << s.half_length_m << "," << s.half_width_m << "," << s.yaw_rad << ","
        << s.center_height_m << "," << s.grade_along_local_x << ";";
  }
  const std::string signature = sig.str();
  if (ever_built_ && signature == built_signature_ &&
      geometry_version == built_geometry_version_ && valid()) {
    return;
  }
  // Preserve every compatible named joint independently. Topology changes
  // may add or remove caster and future arm joints without teleporting the
  // base or resetting wheel/steering state.
  std::vector<JointStateSnapshot> saved_joint_state;
  if (valid()) {
    saved_joint_state = capture_joint_state(impl_->model, impl_->data);
  }
  impl_.reset(new Impl());
  ready_ = false;
  drive_count_ = 0;
  steer_count_ = 0;
  base_qpos_ = -1;
  base_dof_ = -1;
 for (int i = 0; i < 4; ++i) {
 drive_dof_[i] = steer_dof_[i] = -1;
 drive_body_[i] = -1;
 drive_pose_[i] = LinkPoseViz{};
 }
  for (int i = 0; i < 4; ++i) drive_steer_[i] = -1;
  for (int i = 0; i < 2; ++i) {
    idler_dof_[i] = -1;
 idler_body_[i] = -1;
    idler_angle_[i] = 0.0;
 idler_pose_[i] = LinkPoseViz{};
  }
  idler_count_ = 0;
 support_count_ = 0;
 for (int i = 0; i < 4; ++i) {
 support_swivel_dof_[i] = -1;
 support_roll_dof_[i] = -1;
 support_fork_body_[i] = -1;
 support_wheel_body_[i] = -1;
 support_fork_pose_[i] = LinkPoseViz{};
 support_wheel_pose_[i] = LinkPoseViz{};
 support_geom_[i] = -1;
 support_shoulder_geom_[i][0] = -1;
 support_shoulder_geom_[i][1] = -1;
 support_swivel_[i] = 0.0;
 support_roll_[i] = 0.0;
 }

  const double r = std::max(1e-3, motor.wheel_radius_m);
  const double track = std::max(0.1, motor.track_width_m);
  const double base_len = std::max(0.1, motor.wheel_base_m);
  // Chassis hard geometry (config, never per-layout constants). Every guard
  // below refuses the build fail-closed (frozen truth, loud once) — a bad
  // chassis must never silently build a different plant.
  const double deck_half_h = motor.body_half_height_m;
 const double body_half_length = motor.body_half_length_m;
 const double body_half_width = motor.body_half_width_m;
  const double tire_hw = motor.tire_half_width_m;
  const double axle_x = motor.drive_axle_x_m;
  auto refuse_chassis = [&](const char* what) {
    if (!refuse_reported_) {
      std::fprintf(stderr, "mujoco_backend: bad chassis %s; build refused\n",
                   what);
      refuse_reported_ = true;
    }
    drive_count_ = 0;
    steer_count_ = 0;
    ready_ = false;
  };
  if (!(deck_half_h > 0.0) || !(deck_half_h < r)) {
    refuse_chassis("'body_half_height_m' must clear the floor (0, r)");
    return;
  }
 if (!(body_half_length > 0.0) || !(body_half_width > 0.0)) {
 refuse_chassis("body half-length and half-width must be positive");
    return;
  }
  if (!(tire_hw > 0.0) || !(tire_hw < r)) {
    refuse_chassis("'tire_half_width_m' must be in (0, r)");
    return;
  }
  // Axle offset stays on the chassis (4-wheel layouts ignore it entirely).
  if (!std::isfinite(axle_x) || std::abs(axle_x) > base_len / 2) {
    refuse_chassis("'drive_axle_x_m' must sit within +-base/2");
    return;
  }
  if (!(motor.wheel_mass_kg > 0.0)) {
    refuse_chassis("'wheel_mass_kg' must be positive");
    return;
  }
  if (!(motor.knuckle_mass_kg > 0.0)) {
    refuse_chassis("'knuckle_mass_kg' must be positive");
    return;
  }
  // Effective COM (0 = follow the axle — same single rule as the loader).
  const double com_eff =
      motor.com_height_m > 0.0 ? motor.com_height_m : r;
 if (!std::isfinite(com_eff) || !(com_eff > 0.0)) {
 refuse_chassis("COM height must be finite positive");
    return;
  }
  if (!(motor.idler_trail_m >= 0.0) ||
      !std::isfinite(motor.idler_trail_m)) {
    refuse_chassis("'idler_trail_m' must be finite non-negative");
    return;
  }
  if (!(motor.payload_mass_kg >= 0.0) ||
      !std::isfinite(motor.payload_mass_kg) ||
      !(motor.payload_height_m >= 0.0) ||
      !std::isfinite(motor.payload_height_m)) {
    refuse_chassis("payload mass/height must be finite non-negative");
    return;
  }
  // (traction load uses chassis + payload at step time — see load_N.)
 // Unpowered supports resolved up front (pure data, no model calls):
 // support_count == 0 = legacy fore/aft pair — the conversion fills it;
 // a negative count is an explicit no-support tracked chassis. The
 // Every authored support is an explicit caster assembly. Its fork swivel,
 // trail and wheel roll are engine state rather than viewer animation.
  MotorParams::SupportParams legs[MotorParams::kMaxSupports];
  int leg_count = 0;
  if (motor.support_count < 0) {
 // Tracked chassis carry their load on the two drive tracks; no caster
 // assembly is present unless the machine config explicitly declares one.
 leg_count = 0;
 } else if (motor.support_count == 0) {
    const double dx = base_len / 2 * 0.9;
    for (int i = 0; i < 2; ++i) {
      legs[leg_count].kind = "caster";
      legs[leg_count].x_m = (i == 0 ? dx : -dx);
      legs[leg_count].radius_m = 0.03;
      legs[leg_count].mass_kg = 0.1;
      legs[leg_count].friction_mu = 0.05;
      legs[leg_count].trail_m = 0.02;
      legs[leg_count].swivel_damping_Nms = 0.005;
      legs[leg_count].swivel_friction_Nm = 0.0;
      legs[leg_count].rolling_friction_m = 0.0005;
      ++leg_count;
    }
  } else {
    for (int i = 0; i < motor.support_count && i < MotorParams::kMaxSupports;
         ++i) {
      legs[leg_count++] = motor.supports[i];
    }
  }
  double support_mass = 0.0;
  for (int i = 0; i < leg_count; ++i) support_mass += legs[i].mass_kg;
 const std::vector<PlacedModule> modules =
 layout_modules(drive_count, steer_count, track, base_len, axle_x);
 int wheel_count = 0;
 int knuckle_count = 0;
 for (const auto& module : modules) {
 if (module.is_drive || module.idle) ++wheel_count;
 if (module.is_steer) ++knuckle_count;
 }
 const double component_mass =
 motor.wheel_mass_kg * wheel_count +
 motor.knuckle_mass_kg * knuckle_count + support_mass;
 const double base_mass = motor.vehicle_mass_kg - component_mass;
 if (!(base_mass > 0.1) || !std::isfinite(base_mass)) {
 refuse_chassis("vehicle mass must exceed wheel, knuckle and support mass");
 return;
 }
 double component_moment_kgm =
 (motor.wheel_mass_kg * wheel_count +
 motor.knuckle_mass_kg * knuckle_count) * r;
 for (int i = 0; i < leg_count; ++i) {
 component_moment_kgm += legs[i].mass_kg * legs[i].radius_m;
 }
 const double ballast_height =
 (motor.vehicle_mass_kg * com_eff - component_moment_kgm) / base_mass;
 if (!(ballast_height > 0.0) || !std::isfinite(ballast_height)) {
 refuse_chassis("declared COM is below the component mass floor");
 return;
 }
  const double mu = std::max(0.05, motor.tire_friction_mu);
  // Mecanum rollers slide freely sideways in reality; the smooth-cylinder
  // stand-in would otherwise grip laterally and pin the body against the
  // virtual-work wrench. Keep drive grip via the wrench, drop the scrub.
  const double mu_tire =
      (drive_count == 4 && steer_count == 0) ? 0.05 : mu;
  mu_tire_base_ = mu_tire;
  for (int i = 0; i < 4; ++i) {
    slip_count_[i] = 0;  // fresh model: reconfirm traction, never inherit
    tire_geom_[i] = -1;
  }
  for (int i = 0; i < 2; ++i) {
    idler_geom_[i] = -1;
  }
  zone_geom_.clear();

 // Wheel modules use the same table for build and per-step zone lookup.
  idler_count_ = 0;
  for (const auto& mod : modules) {
    if (mod.is_drive && mod.drive_idx >= 0 && mod.drive_idx < 4) {
      drive_mod_x_[mod.drive_idx] = mod.x;
      drive_mod_y_[mod.drive_idx] = mod.y;
      drive_steer_[mod.drive_idx] =
          (mod.is_steer && mod.steer_idx >= 0 && mod.steer_idx < 4)
              ? mod.steer_idx
              : -1;
    }
    // Steering idlers (ackermann fronts): knuckle position for viz (the
    // 2 cm mechanical trail shifts the contact, not the drawn module).
    if (mod.idle && mod.is_steer && mod.steer_idx >= 0 &&
        mod.steer_idx < 2) {
      idler_mod_x_[mod.steer_idx] = mod.x;
      idler_mod_y_[mod.steer_idx] = mod.y;
      ++idler_count_;
    }
  }
  // The free-joint base frame is the drive axle, so terrain revalidation
 // must use actual wheel contact footprints. Sampling the body centre lifts
 // the whole chassis over a narrow centre ridge and hides a genuine belly
 // scrape; sampling support wheels preserves that physical distinction.
 auto support_ground_height = [&](double bx, double by, double yaw) {
 double h = 0.0;
 const double c = std::cos(yaw), sn = std::sin(yaw);
 auto sample_at = [&](double lx, double ly) {
 const double wx = bx + lx * c - ly * sn;
 const double wy = by + lx * sn + ly * c;
 h = std::max(h, sample_terrain(patches, slabs, wx, wy).height_m);
 };
 for (const auto& mod : modules) sample_at(mod.x, mod.y);
    for (int i = 0; i < leg_count; ++i) sample_at(legs[i].x_m, legs[i].y_m);
 return h;
 };
 // Drive joint i follows module i for the first drive_count modules;
  // steer joint i follows the i-th steered module. Ackermann orders its
  // arrays [rear, rear, front, front]: drives are modules 0,1 and steers
  // are modules 2,3.
  // Model from data, no XML anywhere: the mjSpec is built programmatically
  // from the live MotorParams + geometry (the JSON-file equivalent), then
  // compiled. Same structure main's XML describes, minus the string layer.
  mjSpec* spec = mj_makeSpec();
  if (spec == nullptr) {
    return;
  }
  spec->option.timestep = 0.001;
  mjsBody* world = mjs_findBody(spec, "world");
  const bool is_mecanum_layout = (drive_count == 4 && steer_count == 0);
 const bool use_heightfield = std::any_of(
 patches.begin(), patches.end(), [](const TerrainPatch& p) {
 if (p.roughness > 0.0 || std::abs(p.grade_xy_per_m) > 1e-12) {
 return true;
 }
 const double width = p.x1 - p.x0;
 const double height = p.y1 - p.y0;
 const double h00 = p.base_height_m;
 const double h10 = h00 + p.grade_x * width;
 const double h01 = h00 + p.grade_y * height;
 const double h11 = h10 + p.grade_y * height;
 return std::min({h00, h10, h01, h11}) < -1e-6;
 });
 // Planar terrain can sit on MuJoCo's infinite plane and uses exact analytic
 // slabs below. Nonlinear relief, cross terms, and depressions replace the
 // complete scenario ground with one continuous height field.
 if (!use_heightfield) {
 mjsGeom* floor = mjs_addGeom(world, nullptr);
 set_name(floor->element, "floor");
 floor->type = mjGEOM_PLANE;
 floor->size[0] = 20.0;
 floor->size[1] = 20.0;
 floor->size[2] = 0.1;
 // Mecanum contacts carry weight only (rollers are not meshed); planar
 // motion comes from the analytic roller wrench below. The floor must
 // slide as freely as the roller stand-ins or the pair friction pins the
 // body against the wrench. Other layouts keep full grip.
 floor->friction[0] =
 is_mecanum_layout ? mu_tire : kFloorSlidingFriction;
 floor->friction[1] = 0.01;
 floor->friction[2] = 0.001;
 }
  mjsBody* base = add_body(world, "base", 0.0, 0.0, r);
  mjsJoint* base_free = mjs_addFreeJoint(base);
  set_name(base_free->element, "base_free");
 // Collision geometry and inertia are separate. COM tuning therefore moves
 // only the contactless ballast, never the visible/physical belly shell.
 mjsGeom* shell = add_geom(base, "body_shell", mjGEOM_BOX,
 body_half_length, body_half_width, deck_half_h, 1);
 shell->mass = 0.0;
 shell->density = 0.0;
 mjsGeom* ballast = add_geom(base, "ballast", mjGEOM_BOX,
 std::max(0.01, body_half_length * 0.45),
 std::max(0.01, body_half_width * 0.45),
 std::max(0.01, deck_half_h * 0.45), 1);
 ballast->mass = base_mass;
 ballast->pos[2] = ballast_height - r;
 ballast->contype = 0;
 ballast->conaffinity = 0;
  // Declared payload, master-style fold: a contactless point-mass sphere
  // at the payload height (group 1 like all robot geoms: no ray hit, no
  // contact). Exact for total mass and COM height; intrinsic inertia of
  // real cargo is neglected (parallel-axis term dominates), documented.
  if (motor.payload_mass_kg > 0.0) {
    const double pcom =
        motor.payload_height_m > 0.0 ? motor.payload_height_m : com_eff;
    mjsGeom* cargo = add_geom(base, "payload", mjGEOM_SPHERE, 0.05, 0.0, 0.0,
                              1);
    cargo->pos[2] = pcom - r;
    cargo->mass = motor.payload_mass_kg;
    cargo->contype = 0;
    cargo->conaffinity = 0;
  }
  int idle_idx = 0;
  for (std::size_t m = 0; m < modules.size(); ++m) {
    const bool is_drive = modules[m].is_drive;
    const bool is_steer = modules[m].is_steer;
    const int drive_idx = modules[m].drive_idx;
    const int steer_idx = modules[m].steer_idx;
    mjsBody* knuckle = nullptr;
    if (is_steer) {
      knuckle = add_body(base, "knuckle" + std::to_string(steer_idx),
                         modules[m].x, modules[m].y, 0.0);
      mjsJoint* steer =
          add_hinge(knuckle, "steer" + std::to_string(steer_idx), 0.0, 0.0, 1.0);
      if (!(drive_count == 4 && steer_count == 4)) {
        steer->limited = mjLIMITED_TRUE;
        steer->range[0] = -motor.max_steer_rad;
        steer->range[1] = motor.max_steer_rad;
      }
      mjsGeom* kingpin = add_geom(knuckle,
                                  "kingpin" + std::to_string(steer_idx),
                                  mjGEOM_SPHERE, 0.02, 0.0, 0.0, 1);
      kingpin->mass = motor.knuckle_mass_kg;
    }
    mjsBody* parent = is_steer ? knuckle : base;
    const double px = is_steer ? 0.0 : modules[m].x;
    const double py = is_steer ? 0.0 : modules[m].y;
    if (is_drive) {
      mjsBody* wheel =
          add_body(parent, "wheel" + std::to_string(drive_idx), px, py, 0.0);
      mjsJoint* drive = add_hinge(wheel, "drive" + std::to_string(drive_idx),
                                  0.0, 1.0, 0.0,
 std::max(1e-6,
 std::max(0.0, motor.wheel_equiv_mass_kg -
 0.5 * motor.wheel_mass_kg) * r * r));
      // S2 viscous drag, native mechanism: patch force c*v becomes joint
      // damping torque c*r^2*w. Coulomb/rolling stay in the contact pair.
      drive->damping[0] =
          std::max(0.0, motor.viscous_drag_N_per_mps) * r * r;
      mjsGeom* tire = add_geom(wheel, "tire" + std::to_string(drive_idx),
                               mjGEOM_CYLINDER, r, tire_hw, 0.0, 1);
      tire->quat[0] = kQuatX90[0];
      tire->quat[1] = kQuatX90[1];
      tire->quat[2] = kQuatX90[2];
      tire->quat[3] = kQuatX90[3];
 tire->mass = motor.wheel_mass_kg;
 tire->priority = 2;
      tire->friction[0] = mu_tire;
      tire->friction[1] = kWheelTorsionalFriction;
      tire->friction[2] = kWheelRollingFriction;
    } else if (modules[m].idle) {
      // Ackermann front idlers: free-spinning wheels inside the steering
      // knuckles (no torque — the two drive duties are the rear axle).
      // Mechanical trail from config (wheel center behind the kingpin,
      // like a real caster): zero trail is neutrally stable and solver
      // noise walks it into a mirrored wedge; trail makes straight-ahead
      // the stable equilibrium the servo then works against, like a car.
      mjsBody* idler = add_body(knuckle, "idler" + std::to_string(idle_idx),
                                -motor.idler_trail_m, 0.0, 0.0);
      add_hinge(idler, "idlerj" + std::to_string(idle_idx), 0.0, 1.0, 0.0);
      mjsGeom* itire = add_geom(idler, "idlertire" + std::to_string(idle_idx),
                                mjGEOM_CYLINDER, r, tire_hw, 0.0, 1);
      itire->quat[0] = kQuatX90[0];
      itire->quat[1] = kQuatX90[1];
      itire->quat[2] = kQuatX90[2];
      itire->quat[3] = kQuatX90[3];
 itire->mass = motor.wheel_mass_kg;
 itire->priority = 2;
      itire->friction[0] = mu;
      itire->friction[1] = kWheelTorsionalFriction;
      itire->friction[2] = kWheelRollingFriction;
      ++idle_idx;
    }
  }
 // Passive caster assemblies: unactuated fork swivel and wheel roll are
 // engine joints. Their authored low friction carries normal load without
 // becoming a hidden drive channel.
  //
 // Each caster owns its trail, contact friction, rolling friction, swivel
 // damping and swivel friction. These quantities have separate units and
 // are never silently capped or reused for a different mechanism.
  // Build loop over the legs resolved up front (validation first — a bad
  // support refuses before any body exists).
  for (int i = 0; i < leg_count; ++i) {
    const auto& leg = legs[i];
    if (leg.kind != "caster") {
      refuse_chassis("support kind must be 'caster'");
      mj_deleteSpec(spec);
      return;
    }
    if (!(leg.radius_m > 0.0) || !(leg.radius_m <= r) ||
        !(leg.mass_kg > 0.0) || !(leg.friction_mu >= 0.0) ||
        !(leg.trail_m >= 0.0) || !(leg.swivel_damping_Nms >= 0.0) ||
        !(leg.swivel_friction_Nm >= 0.0) || !(leg.rolling_friction_m >= 0.0)) {
      refuse_chassis("support needs radius in (0, r], mass > 0");
      mj_deleteSpec(spec);
      return;
    }
    const double cz = -(r - leg.radius_m);  // feet level with tire feet
    const double trail = leg.trail_m;
 support_anchor_x_[i] = leg.x_m;
    support_anchor_y_[i] = leg.y_m;
 support_radius_[i] = leg.radius_m;
 support_trail_[i] = trail;
 // Passive casters should carry normal load without becoming a second
 // powered axle. Their lateral slip is deliberately low; the wheel-roll
 // hinge and trail provide the physical caster response while keeping the
 // legacy drive force budget independent of support tire authoring.
    support_mu_base_[i] = leg.friction_mu;
    support_rolling_base_m_[i] = leg.rolling_friction_m;
 mjsBody* fork = add_body(base, "caster" + std::to_string(i), leg.x_m,
        leg.y_m, cz);
 mjsJoint* swivel = add_hinge(fork, "casterswivel" + std::to_string(i),
 0.0, 0.0, 1.0);
    swivel->damping[0] = leg.swivel_damping_Nms;
    swivel->frictionloss = leg.swivel_friction_Nm;
 mjsGeom* bearing = add_geom(fork,
 "casterbearing" + std::to_string(i), mjGEOM_SPHERE,
 std::max(0.008, leg.radius_m * 0.35), 0.0, 0.0, 1);
 bearing->mass = leg.mass_kg * 0.25;
 bearing->contype = 0;
 bearing->conaffinity = 0;
 mjsBody* wheel = add_body(fork, "casterwheel" + std::to_string(i),
 -trail, 0.0, 0.0);
 mjsJoint* roll = add_hinge(wheel, "casterroll" + std::to_string(i),
 0.0, 1.0, 0.0,
 std::max(1e-7, leg.mass_kg * leg.radius_m * leg.radius_m * 0.25));
 roll->damping[0] = 0.0;
 roll->frictionloss = 0.0;
 const double caster_hw = std::max(1e-4,
 std::min(tire_hw, leg.radius_m * 0.75));
 // Crowned tread: the centre rib has the nominal radius and carries the
 // normal load on straight travel; the two shoulders are a few millimetres
 // smaller and only engage under lean/side load. This is a compact physical
 // approximation of a real passive caster tire and avoids making a flat,
 // full-width cylinder scrub laterally across the floor.
 const double crown_halfwidth = caster_hw * 0.5;
 const double shoulder_halfwidth = caster_hw * 0.25;
 const double shoulder_offset = crown_halfwidth + shoulder_halfwidth;
 const double shoulder_drop = std::max(1.0e-4, leg.radius_m * 0.10);
 const double shoulder_radius =
 std::max(1.0e-4, leg.radius_m - shoulder_drop);
 const auto configure_tire = [&](mjsGeom* geom, double mass) {
 geom->quat[0] = kQuatX90[0];
 geom->quat[1] = kQuatX90[1];
 geom->quat[2] = kQuatX90[2];
 geom->quat[3] = kQuatX90[3];
 geom->mass = mass;
 // MuJoCo combines equal-priority contact friction with a max rule. Give
 // the passive tire a higher contact priority so its authored low scrub
 // coefficient wins against the high-grip floor while still carrying the
 // normal load. Drive tires remain on the regular floor priority.
 geom->priority = 2;
      geom->friction[0] = leg.friction_mu;
      // Swivel bearing losses live on the swivel joint. Contact rolling loss
      // uses its separately authored MuJoCo length; torsional contact stays zero.
 geom->friction[1] = 0.0;
      geom->friction[2] = leg.rolling_friction_m;
 };
 mjsGeom* tire = add_geom(wheel, "castertire" + std::to_string(i),
 mjGEOM_CYLINDER, leg.radius_m, crown_halfwidth, 0.0, 1);
 configure_tire(tire, leg.mass_kg * 0.50);
 mjsGeom* shoulder_l = add_geom(
 wheel, "castertire" + std::to_string(i) + "_shoulder_l",
 mjGEOM_CYLINDER, shoulder_radius, shoulder_halfwidth, 0.0, 1);
 shoulder_l->pos[1] = shoulder_offset;
 configure_tire(shoulder_l, leg.mass_kg * 0.125);
 mjsGeom* shoulder_r = add_geom(
 wheel, "castertire" + std::to_string(i) + "_shoulder_r",
 mjGEOM_CYLINDER, shoulder_radius, shoulder_halfwidth, 0.0, 1);
 shoulder_r->pos[1] = -shoulder_offset;
 configure_tire(shoulder_r, leg.mass_kg * 0.125);
 ++support_count_;
  }
  // Surveyed static geometry: fixed worldbody boxes (no joints).
  // Height <= 0 means flat paint (floor marking, survey plate): no engine
  // body, no contact, no ray hit. Never silently substitute a default
  // height — that turned paint into walls.
  int bi = 0;
  for (const auto& b : boxes) {
    if (!(b.height_m > 0.0)) continue;  // flat paint: skip the engine body
    const double hx = std::max(0.01, (b.x1 - b.x0) / 2);
    const double hy = std::max(0.01, (b.y1 - b.y0) / 2);
    const double hh = std::max(0.05, b.height_m / 2);
    mjsGeom* wall = add_geom(world, "static" + std::to_string(bi),
                             mjGEOM_BOX, hx, hy, hh);
    wall->pos[0] = (b.x0 + b.x1) / 2;
    wall->pos[1] = (b.y0 + b.y1) / 2;
    wall->pos[2] = hh;
    wall->friction[0] = 1.0;
    wall->friction[1] = 0.01;
    wall->friction[2] = 0.001;
    ++bi;
  }
 // Material zones: thin floor slabs (top 1 mm above the plane, avoiding
  // coplanar contact ambiguity) carrying the library friction + contact
  // numbers. The pair takes the max, so the tire side (per-step, per-wheel)
  // governs at/below its rating and grippier ground wins upward — the same
  // relationship the legacy floor (1.0) has with the tire rating.
  // Material properties are applied directly to live contacts in step().
 // Nonlinear or below-grade authored terrain is one height field over the
 // complete map footprint. Adjacent patches share vertices and contact
 // normals; there are no internal tile seams, and negative relief is not
 // masked by an infinite floor at z=0.
 if (use_heightfield) {
 double x0 = 0.0, y0 = 0.0, x1 = 0.0, y1 = 0.0;
 auto include = [&](double ax0, double ay0, double ax1, double ay1) {
 x0 = std::min(x0, ax0);
 y0 = std::min(y0, ay0);
 x1 = std::max(x1, ax1);
 y1 = std::max(y1, ay1);
 };
 for (const auto& p : patches) include(p.x0, p.y0, p.x1, p.y1);
 for (const auto& b : boxes) include(b.x0, b.y0, b.x1, b.y1);
 for (const auto& z : zones) include(z.x0, z.y0, z.x1, z.y1);
 for (const auto& s : slabs) {
 const double extent = std::hypot(s.half_length_m, s.half_width_m);
 include(s.center_x_m - extent, s.center_y_m - extent,
 s.center_x_m + extent, s.center_y_m + extent);
 }
 constexpr double kGroundMarginM = 2.0;
 x0 -= kGroundMarginM;
 y0 -= kGroundMarginM;
 x1 += kGroundMarginM;
 y1 += kGroundMarginM;
 const double width = std::max(0.1, x1 - x0);
 const double height = std::max(0.1, y1 - y0);
 // Keep nonlinear contact geometry at the authored spatial scale even when
 // a scenario spans hundreds of metres. One globally capped height field
 // made its cell width grow with map extent, while the survey and renderer
 // continued sampling the canonical surface at full fidelity. Fixed-world
 // tiles share boundary vertices and allocate a fine grid only where an
 // authored terrain patch is present; flat tiles need four corners.
 constexpr double kTargetCellM = 0.10;
 constexpr int kMaximumFineCellsPerTile = 255;
 constexpr double kMaximumTileSpanM =
 kTargetCellM * kMaximumFineCellsPerTile;
 const int tile_columns = std::max(
 1, static_cast<int>(std::ceil(width / kMaximumTileSpanM)));
 const int tile_rows = std::max(
 1, static_cast<int>(std::ceil(height / kMaximumTileSpanM)));
 for (int tile_y = 0; tile_y < tile_rows; ++tile_y) {
 const double tile_y0 = y0 + tile_y * kMaximumTileSpanM;
 const double tile_y1 = std::min(y1, tile_y0 + kMaximumTileSpanM);
 for (int tile_x = 0; tile_x < tile_columns; ++tile_x) {
 const double tile_x0 = x0 + tile_x * kMaximumTileSpanM;
 const double tile_x1 = std::min(x1, tile_x0 + kMaximumTileSpanM);
 const bool contains_terrain = std::any_of(
 patches.begin(), patches.end(), [&](const TerrainPatch& patch) {
 return patch.x1 > tile_x0 && patch.x0 < tile_x1 &&
 patch.y1 > tile_y0 && patch.y0 < tile_y1;
 });
 const double tile_width = tile_x1 - tile_x0;
 const double tile_height = tile_y1 - tile_y0;
 const int nx = contains_terrain
 ? std::clamp(static_cast<int>(std::ceil(
 tile_width / kTargetCellM)), 1, kMaximumFineCellsPerTile)
 : 1;
 const int ny = contains_terrain
 ? std::clamp(static_cast<int>(std::ceil(
 tile_height / kTargetCellM)), 1, kMaximumFineCellsPerTile)
 : 1;
 std::vector<float> elevation;
 elevation.reserve(static_cast<std::size_t>(nx + 1) *
 static_cast<std::size_t>(ny + 1));
 double min_h = std::numeric_limits<double>::infinity();
 double max_h = -std::numeric_limits<double>::infinity();
 // mjsHField userdata is row-major from minimum Y to maximum Y. Only
 // MJCF's inline text reader performs an image-row flip.
 for (int iy = 0; iy <= ny; ++iy) {
 const double y = tile_y0 +
 tile_height * static_cast<double>(iy) / ny;
 for (int ix = 0; ix <= nx; ++ix) {
 const double x = tile_x0 +
 tile_width * static_cast<double>(ix) / nx;
 const double h = sample_terrain(patches, {}, x, y).height_m;
 elevation.push_back(static_cast<float>(h));
 min_h = std::min(min_h, h);
 max_h = std::max(max_h, h);
 }
 }
 const std::string suffix = std::to_string(tile_y) + "_" +
 std::to_string(tile_x);
 const std::string field_name = "terrain_surface_" + suffix;
 mjsHField* field = mjs_addHField(spec);
 set_name(field->element, field_name);
 field->nrow = ny + 1;
 field->ncol = nx + 1;
 field->size[0] = tile_width * 0.5;
 field->size[1] = tile_height * 0.5;
 field->size[2] = std::max(1e-4, max_h - min_h);
 field->size[3] = std::max(0.10, std::abs(min_h) + 0.10);
 mjs_setFloat(field->userdata, elevation.data(),
 static_cast<int>(elevation.size()));
 mjsGeom* ground = add_geom(
 world, "terrain_ground_" + suffix, mjGEOM_HFIELD,
 0.0, 0.0, 0.0);
 mjs_setString(ground->hfieldname, field_name.c_str());
 ground->pos[0] = (tile_x0 + tile_x1) * 0.5;
 ground->pos[1] = (tile_y0 + tile_y1) * 0.5;
 ground->pos[2] = min_h;
 ground->friction[0] =
 is_mecanum_layout ? mu_tire : kFloorSlidingFriction;
 ground->friction[1] = 0.01;
 ground->friction[2] = 0.001;
 }
 }

 }

 if (!use_heightfield) {
 // Planar patches use exact oriented slabs whose top face coincides with the
  // canonical height plane (same grade through the center point the survey
  // rasterizes, so engine contact and map elevation agree exactly on the
  // footprint). Patch borders are retaining walls, never ramps.
 // Nonlinear patches never enter this path; the single height field above
 // owns those surfaces.
  for (std::size_t pi = 0; pi < patches.size(); ++pi) {
    const TerrainPatch& p = patches[pi];
    if (!(p.x1 > p.x0 && p.y1 > p.y0)) continue;
      const double hx = (p.x1 - p.x0) / 2;
      const double hy = (p.y1 - p.y0) / 2;
      const double cx = (p.x0 + p.x1) / 2;
      const double cy = (p.y0 + p.y1) / 2;
      TerrainSample center = sample_terrain(patches, slabs, cx, cy);
      const double half_t = (center.height_m + 0.5) / 2;
      double q[4];
      grade_quat(p.grade_x, p.grade_y, q);
      mjsGeom* pg = add_geom(world, "terrain" + std::to_string(pi), mjGEOM_BOX,
                             hx, hy, std::max(0.05, half_t), 0, q);
      pg->pos[0] = cx;
      pg->pos[1] = cy;
      // Tilt-corrected so the top face passes through the surveyed center.
      pg->pos[2] = center.height_m -
                   std::max(0.05, half_t) * std::cos(std::atan(p.grade_x)) *
                       std::cos(std::atan(p.grade_y));
      pg->friction[0] = kFloorSlidingFriction;
      pg->friction[1] = 0.01;
      pg->friction[2] = 0.001;
  }
  // Continuous patch seams receive a small beveled contact cap. The
 // authored elevation remains piecewise (and the lidar/map still report
 // it), while the engine gets a wheelable transition instead of two
 // coplanar box faces whose solver edge behaves like a curb. A height step
 // above the tolerance is intentionally left as a retaining wall.
 auto seam_plane_height = [](const TerrainPatch& t, double x, double y) {
 const double dx = x - t.x0, dy = y - t.y0;
 return t.base_height_m + t.grade_x * dx + t.grade_y * dy;
 };
 int seam_index = 0;
 for (std::size_t ai = 0; ai < patches.size(); ++ai) {
 const TerrainPatch& a = patches[ai];
 if (a.roughness > 0.0 || std::abs(a.grade_xy_per_m) > 1e-12) continue;
 for (std::size_t bi = ai + 1; bi < patches.size(); ++bi) {
 const TerrainPatch& b = patches[bi];
 if (b.roughness > 0.0 || std::abs(b.grade_xy_per_m) > 1e-12) continue;
 bool x_seam = false;
 double edge = 0.0;
 if (std::abs(a.x1 - b.x0) < 1e-9) { x_seam = true; edge = a.x1; }
 else if (std::abs(b.x1 - a.x0) < 1e-9) { x_seam = true; edge = b.x1; }
 else if (std::abs(a.y1 - b.y0) < 1e-9) { edge = a.y1; }
 else if (std::abs(b.y1 - a.y0) < 1e-9) { edge = b.y1; }
 else continue;
 const double lo = x_seam ? std::max(a.y0, b.y0)
 : std::max(a.x0, b.x0);
 const double hi = x_seam ? std::min(a.y1, b.y1)
 : std::min(a.x1, b.x1);
 if (!(hi - lo > 0.04)) continue;
 const double mid = 0.5 * (lo + hi);
 const double sx = x_seam ? edge : mid;
 const double sy = x_seam ? mid : edge;
 const double ha = seam_plane_height(a, sx, sy);
 const double hb = seam_plane_height(b, sx, sy);
 if (std::abs(ha - hb) > 0.01) continue;
 // Equal seam heights define one continuous surface even when its slope
 // changes abruptly (a ridge or valley). The vertical side faces of the
 // two finite slabs are construction artifacts, not authored retaining
 // walls, so cover them with the local tangent transition below. True
 // height discontinuities were rejected by the height check above.
 const double h = 0.5 * (ha + hb);
 const double gx = 0.5 * (a.grade_x + b.grade_x);
 const double gy = 0.5 * (a.grade_y + b.grade_y);
 const double half_len = 0.03;
 const double half_width = 0.5 * (hi - lo);
 const double half_t = std::max(0.05, (h + 0.5) / 2);
 double q[4]; grade_quat(gx, gy, q);
 mjsGeom* cap = add_geom(world, "terrain_seam" + std::to_string(seam_index++),
 mjGEOM_BOX, x_seam ? half_len : half_width,
 x_seam ? half_width : half_len, half_t, 0, q);
 cap->pos[0] = x_seam ? edge : mid;
 cap->pos[1] = x_seam ? mid : edge;
 cap->pos[2] = h - half_t * std::cos(std::atan(gx)) *
 std::cos(std::atan(gy));
 cap->friction[0] = kFloorSlidingFriction;
 cap->friction[1] = 0.01;
 cap->friction[2] = 0.001;
 }
 }
 }
 // Support slabs: thin oriented decks (same top-coincidence construction).
  for (std::size_t si = 0; si < slabs.size(); ++si) {
    const SupportSlab& s = slabs[si];
    if (!(s.half_length_m > 0.0 && s.half_width_m > 0.0)) continue;
    TerrainSample center = sample_terrain(patches, slabs, s.center_x_m,
                                          s.center_y_m);
    double qg[4];
    grade_quat(s.grade_along_local_x, 0.0, qg);
    const double yaw_q[4] = {std::cos(s.yaw_rad * 0.5), 0.0, 0.0,
                             std::sin(s.yaw_rad * 0.5)};
    double q[4];
    quat_mul(yaw_q, qg, q);
    constexpr double kSlabHalfT = 0.03;
    mjsGeom* sg = add_geom(world, "slab" + std::to_string(si), mjGEOM_BOX,
                           s.half_length_m, s.half_width_m, kSlabHalfT, 0, q);
    sg->pos[0] = s.center_x_m;
    sg->pos[1] = s.center_y_m;
    sg->pos[2] = center.height_m -
                 kSlabHalfT * std::cos(std::atan(s.grade_along_local_x));
    sg->friction[0] = kFloorSlidingFriction;
    sg->friction[1] = 0.01;
    sg->friction[2] = 0.001;
  }

  mjModel* model = mj_compile(spec, nullptr);
  if (model == nullptr) {
    std::fprintf(stderr, "mujoco_backend: mj_compile failed: %s\n",
                 mjs_getError(spec));
    mj_deleteSpec(spec);
    return;  // valid() stays false: never step a broken model
  }
  mj_deleteSpec(spec);
  mjData* data = mj_makeData(model);
  if (data == nullptr) {
    mj_deleteModel(model);
    return;
  }
 const int base_joint = mj_name2id(model, mjOBJ_JOINT, "base_free");
 if (base_joint < 0 || model->jnt_type[base_joint] != mjJNT_FREE) {
  mj_deleteData(data);
  mj_deleteModel(model);
  return;
 }
 const int base_qpos = model->jnt_qposadr[base_joint];
  impl_->model = model;
  impl_->data = data;
 // A free joint defaults to the world origin, which puts the axle and every
 // tire below the floor until the first solver step lifts the chassis.  That
 // transient is observable in the viewer (and can leave a wheel apparently
 // embedded when the first command changes the joint layout and rebuilds the
 // model).  The base frame is the drive axle frame, so a level spawn is
 // exactly one configured drive radius above the canonical ground plane.
 // Seed the engine state before restoring a compatible rebuild snapshot. The
 // snapshot remains authoritative for x/y, attitude, velocities, and joint
 // angles, while root z is revalidated against the new wheel radius and the
 // canonical authored terrain below.
 data->qpos[base_qpos + 0] = 0.0;
 data->qpos[base_qpos + 1] = 0.0;
 data->qpos[base_qpos + 2] = support_ground_height(0.0, 0.0, 0.0) + r;
 data->qpos[base_qpos + 3] = 1.0;
 data->qpos[base_qpos + 4] = 0.0;
 data->qpos[base_qpos + 5] = 0.0;
 data->qpos[base_qpos + 6] = 0.0;
  // Brake lag shapes from the motor contract (main-stack mapping).
  const ServiceBrakeParams bp = brake_params_from_motor(motor);
  for (auto& b : impl_->brakes) b = ServiceBrakePlant{bp};
  int found_drives = 0, found_steers = 0;
  for (int i = 0; i < 4; ++i) {
 char name[64];
    std::snprintf(name, sizeof(name), "drive%d", i);
    const int jid = mj_name2id(model, mjOBJ_JOINT, name);
    drive_dof_[i] = (jid >= 0) ? model->jnt_dofadr[jid] : -1;
    if (drive_dof_[i] >= 0) ++found_drives;
    std::snprintf(name, sizeof(name), "steer%d", i);
    const int sid = mj_name2id(model, mjOBJ_JOINT, name);
    steer_dof_[i] = (sid >= 0) ? model->jnt_dofadr[sid] : -1;
    if (steer_dof_[i] >= 0) ++found_steers;
    std::snprintf(name, sizeof(name), "tire%d", i);
    tire_geom_[i] = mj_name2id(model, mjOBJ_GEOM, name);
 std::snprintf(name, sizeof(name), "wheel%d", i);
 drive_body_[i] = mj_name2id(model, mjOBJ_BODY, name);
    if (i < 2) {
      std::snprintf(name, sizeof(name), "idlerj%d", i);
      const int iid = mj_name2id(model, mjOBJ_JOINT, name);
      idler_dof_[i] = (iid >= 0) ? model->jnt_dofadr[iid] : -1;
 std::snprintf(name, sizeof(name), "idler%d", i);
 idler_body_[i] = mj_name2id(model, mjOBJ_BODY, name);
      // Idler friction target (per-step zone follow, same as drives).
      std::snprintf(name, sizeof(name), "idlertire%d", i);
      idler_geom_[i] = mj_name2id(model, mjOBJ_GEOM, name);
    }
 if (i < support_count_) {
 std::snprintf(name, sizeof(name), "casterswivel%d", i);
 const int sid = mj_name2id(model, mjOBJ_JOINT, name);
 support_swivel_dof_[i] = (sid >= 0) ? model->jnt_dofadr[sid] : -1;
 std::snprintf(name, sizeof(name), "casterroll%d", i);
 const int rid = mj_name2id(model, mjOBJ_JOINT, name);
 support_roll_dof_[i] = (rid >= 0) ? model->jnt_dofadr[rid] : -1;
 std::snprintf(name, sizeof(name), "caster%d", i);
 support_fork_body_[i] = mj_name2id(model, mjOBJ_BODY, name);
 std::snprintf(name, sizeof(name), "casterwheel%d", i);
 support_wheel_body_[i] = mj_name2id(model, mjOBJ_BODY, name);
 std::snprintf(name, sizeof(name), "castertire%d", i);
 support_geom_[i] = mj_name2id(model, mjOBJ_GEOM, name);
 std::snprintf(name, sizeof(name), "castertire%d_shoulder_l", i);
 support_shoulder_geom_[i][0] = mj_name2id(model, mjOBJ_GEOM, name);
 std::snprintf(name, sizeof(name), "castertire%d_shoulder_r", i);
 support_shoulder_geom_[i][1] = mj_name2id(model, mjOBJ_GEOM, name);
 }
  }
  zone_geom_.clear();
  for (std::size_t zi = 0; zi < zones.size(); ++zi) {
    const std::string name = "zone" + std::to_string(zi);
    zone_geom_.push_back(mj_name2id(model, mjOBJ_GEOM, name.c_str()));
  }
  if (found_drives != drive_count || found_steers != steer_count) {
    // The compiled model is still local until the joint census succeeds.
    // Release both allocations on this fail-closed path; otherwise every
    // refused/incomplete rebuild leaks a complete model and its data arena.
    // Null the installed handles: ~Impl() frees non-null members, and a
    // dangling install would double-free on the next sync or at teardown.
    mj_deleteData(data);
    mj_deleteModel(model);
    impl_->model = nullptr;
    impl_->data = nullptr;
    return;  // incomplete model: frozen, never silent substitution
  }
  drive_count_ = drive_count;
  steer_count_ = steer_count;
  ready_ = true;
  refuse_reported_ = false;
  track_m_ = track;
  base_len_m_ = base_len;
  base_qpos_ = base_qpos;
  base_dof_ = model->jnt_dofadr[base_joint];
  built_signature_ = signature;
  built_geometry_version_ = geometry_version;
  ever_built_ = true;
 restore_joint_state(saved_joint_state, model, data);
 {
 // Preserve a live vehicle's elevation when it is above the new contact
 // plane (for example while riding a box edge), but never restore a root
 // pose below the lowest wheel contact. Starting at the exact boundary lets
 // MuJoCo's contact solver settle without carrying an obsolete penetration.
  const double qw = data->qpos[base_qpos + 3];
  const double qx = data->qpos[base_qpos + 4];
  const double qy = data->qpos[base_qpos + 5];
  const double qz = data->qpos[base_qpos + 6];
 const double yaw = std::atan2(2.0 * (qw * qz + qx * qy),
 1.0 - 2.0 * (qy * qy + qz * qz));
  const double ground = support_ground_height(
      data->qpos[base_qpos + 0], data->qpos[base_qpos + 1], yaw);
 const double minimum_z = ground + r;
  if (!std::isfinite(data->qpos[base_qpos + 2]) ||
      data->qpos[base_qpos + 2] < minimum_z) {
    data->qpos[base_qpos + 2] = minimum_z;
 }
 }
 // Refresh kinematics for both a fresh spawn and a restored snapshot before
 // the first integration step, so read-back and raycasts see the same state.
 mj_forward(model, data);
  read_back();
}

}  // namespace lost_signal::sim::world

#endif  // LOST_SIGNAL_WITH_MUJOCO
