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
#include <sstream>
#include <string>
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
      << "|" << motor.body_half_height_m << "|" << motor.body_half_width_m
      << "|" << motor.drive_axle_x_m << "|" << motor.tire_half_width_m
      << "|" << motor.wheel_mass_kg << "|" << motor.knuckle_mass_kg
      << "|" << motor.tire_density_kgpm3 << "|" << motor.com_height_m
      << "|" << motor.payload_mass_kg << "|" << motor.payload_height_m
      << "|" << motor.idler_trail_m << "|" << motor.support_count << "|";
  for (int i = 0; i < motor.support_count && i < MotorParams::kMaxSupports;
       ++i) {
    const auto& s = motor.supports[i];
    sig << s.kind << "," << s.x_m << "," << s.radius_m << "," << s.mass_kg
        << "," << s.friction_mu << "," << s.torsional << "," << s.rolling
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
  // Preserve the dynamic state across a motor-only rebuild (same bodies,
  // same DOFs: boxes are static, layout is fixed by the mode).
  std::vector<double> saved_qpos, saved_qvel;
  if (valid()) {
    mjModel* old_model = impl_->model;
    mjData* old_data = impl_->data;
    if (old_model->nq == 0) {
      (void)old_data;
    } else {
      saved_qpos.assign(old_data->qpos, old_data->qpos + old_model->nq);
      saved_qvel.assign(old_data->qvel, old_data->qvel + old_model->nv);
    }
  }
  impl_.reset(new Impl());
  ready_ = false;
  drive_count_ = 0;
  steer_count_ = 0;
  for (int i = 0; i < 4; ++i) drive_dof_[i] = steer_dof_[i] = -1;
  for (int i = 0; i < 4; ++i) drive_steer_[i] = -1;
  for (int i = 0; i < 2; ++i) {
    idler_dof_[i] = -1;
    idler_angle_[i] = 0.0;
  }
  idler_count_ = 0;

  const double r = std::max(1e-3, motor.wheel_radius_m);
  const double track = std::max(0.1, motor.track_width_m);
  const double base_len = std::max(0.1, motor.wheel_base_m);
  // Chassis hard geometry (config, never per-layout constants). Every guard
  // below refuses the build fail-closed (frozen truth, loud once) — a bad
  // chassis must never silently build a different plant.
  const double deck_half_h = motor.body_half_height_m;
  const double tub_half_xy = motor.body_half_width_m;
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
  // Tub stays clear of every wheel swept volume (same constraint the
  // legacy 0.12 tub satisfied: |y| >= track/2 - width, |x| >= base/2 - r).
  // A full-deck overlap resolved explosively and masked as a steering
  // instability — never again.
  const double tub_limit =
      std::min(track / 2 - tire_hw, base_len / 2 - r);
  if (!(tub_half_xy > 0.0) || !(tub_half_xy < tub_limit)) {
    refuse_chassis("'body_half_width_m' must clear the wheel swept volume");
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
  if (!(motor.tire_density_kgpm3 > 0.0)) {
    refuse_chassis("'tire_density_kgpm3' must be positive");
    return;
  }
  // Effective COM (0 = follow the axle — same single rule as the loader).
  const double com_eff =
      motor.com_height_m > 0.0 ? motor.com_height_m : r;
  if (!std::isfinite(com_eff) || com_eff - deck_half_h <= 0.0) {
    refuse_chassis("COM must clear the deck (belly = com - half_h > 0)");
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
  // support_count == 0 = legacy fore/aft ball pair — the conversion fills
  // it, direct-built params use this same rule, so exactly one default.
  // Only "ball" exists: free spheres carrying normal load only
  // (near-zero tangential by authoring). Anything else refuses below.
  MotorParams::SupportParams legs[MotorParams::kMaxSupports];
  int leg_count = 0;
  if (motor.support_count <= 0) {
    const double dx = base_len / 2 * 0.9;
    for (int i = 0; i < 2; ++i) {
      legs[leg_count].kind = "ball";
      legs[leg_count].x_m = (i == 0 ? dx : -dx);
      legs[leg_count].radius_m = 0.03;
      legs[leg_count].mass_kg = 0.1;
      legs[leg_count].friction_mu = 0.05;
      legs[leg_count].torsional = 0.005;
      legs[leg_count].rolling = 0.0005;
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
  // Deck conserves the vehicle total (wheels + supports itemized, floor
  // guards a degenerate all-wheel plant). Previously the 0.2/wheel split
  // was hardcoded and supports were never subtracted.
  const double base_mass = std::max(
      0.5, motor.vehicle_mass_kg - motor.wheel_mass_kg * drive_count -
               support_mass);
  // Central mass tub (ballast only; rendering shells belong to M3).
  // Size comes from config; the swept-volume guard above is what keeps a
  // full-deck overlap (initial penetration resolving explosively, masking
  // as steering instability) from ever rebuilding.
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

  // Wheel modules per layout: one shared table for the build (bodies)
  // and the step (wheel world positions for the zone lookup).
  const std::vector<PlacedModule> modules =
      layout_modules(drive_count, steer_count, track, base_len, axle_x);
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
  mjsBody* base = add_body(world, "base", 0.0, 0.0, r);
  mjs_addFreeJoint(base);
  mjsGeom* deck =
      add_geom(base, "deck", mjGEOM_BOX, tub_half_xy, tub_half_xy, deck_half_h, 1);
  deck->mass = base_mass;
  // Ballast center rides the effective COM (body frame: base origin sits
  // at axle height r, so the offset is com - r; 0 by default, legacy-exact).
  deck->pos[2] = com_eff - r;
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
                                  std::max(1e-6, motor.wheel_equiv_mass_kg * r * r));
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
      tire->density = motor.tire_density_kgpm3;
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
      itire->density = motor.tire_density_kgpm3;
      itire->friction[0] = mu;
      itire->friction[1] = kWheelTorsionalFriction;
      itire->friction[2] = kWheelRollingFriction;
      ++idle_idx;
    }
  }
  // Ball casters fore/aft, like the real differential base: unactuated
  // balls that roll instead of gripping, so motor reaction torque settles
  // into weight transfer instead of slamming the deck onto the floor.
  //
  // Deliberately NOT forked swivel wheels (measured, kept honest): a
  // trailed fork transmits tangential force into the deck — on the thin-
  // margin icy climb the undriven forks steal normal force AND brake the
  // crest pivot (stuck at x=6.0 with both drives spinning), and on 2 cm
  // relief the small fork wheels plow and beach the plant. Free balls
  // carry normal load only (near-zero tangential by authoring below), so
  // both pinned missions pass; the fork+wheel doctrine from master stays
  // where master uses it — animation mounts, never contact.
  // Build loop over the legs resolved up front (validation first — a bad
  // support refuses before any body exists).
  for (int i = 0; i < leg_count; ++i) {
    const auto& leg = legs[i];
    if (leg.kind != "ball") {
      refuse_chassis("support kind must be 'ball'");
      mj_deleteSpec(spec);
      return;
    }
    if (!(leg.radius_m > 0.0) || !(leg.radius_m <= r) ||
        !(leg.mass_kg > 0.0) || !(leg.friction_mu >= 0.0) ||
        !(leg.torsional >= 0.0) || !(leg.rolling >= 0.0)) {
      refuse_chassis("support needs radius in (0, r], mass > 0");
      mj_deleteSpec(spec);
      return;
    }
    const double cz = -(r - leg.radius_m);  // feet level with tire feet
    mjsBody* caster = add_body(base, "caster" + std::to_string(i), leg.x_m,
                               0.0, cz);
    mjsJoint* ball = mjs_addJoint(caster, nullptr);
    set_name(ball->element, ("casterj" + std::to_string(i)).c_str());
    ball->type = mjJNT_BALL;
    mjsGeom* ball_g = add_geom(caster, "ball" + std::to_string(i),
                               mjGEOM_SPHERE, leg.radius_m, 0.0, 0.0, 1);
    ball_g->mass = leg.mass_kg;
    ball_g->friction[0] = leg.friction_mu;
    ball_g->friction[1] = leg.torsional;
    ball_g->friction[2] = leg.rolling;
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
  for (std::size_t zi = 0; zi < zones.size(); ++zi) {
    const MaterialZone& z = zones[zi];
    if (!(z.x1 > z.x0 && z.y1 > z.y0)) continue;  // zero area covers nothing
    const MaterialEntry* mat = find_material(materials, z.material_id);
    if (mat == nullptr) continue;  // refused at sync head; defensive
    const double hx = (z.x1 - z.x0) / 2;
    const double hy = (z.y1 - z.y0) / 2;
    mjsGeom* zg = add_geom(world, "zone" + std::to_string(zi), mjGEOM_BOX,
                           hx, hy, 0.025);
    zg->pos[0] = (z.x0 + z.x1) / 2;
    zg->pos[1] = (z.y0 + z.y1) / 2;
    zg->pos[2] = 0.001 - 0.025;
    zg->friction[0] = std::max(0.0, mat->friction_mu);
    zg->friction[1] = 0.01;
    // Engine-native rolling parameter fed from the material: Coulomb rr is
    // a force ratio, the solver wants a torque arm — scale by the wheel
    // radius (rr 0.015 at r 0.1 lands at the legacy 0.002 neighborhood).
    zg->friction[2] =
        std::clamp(mat->rolling_resist * r, 0.0, 0.05);
    zg->solref[0] = std::max(1e-4, mat->contact_time_constant_s);
    zg->solref[1] = std::max(0.0, mat->contact_damping_ratio);
  }
  // Terrain patches: oriented slabs whose top face coincides with the
  // canonical height plane (same grade through the center point the survey
  // rasterizes, so engine contact and map elevation agree exactly on the
  // footprint). Patch borders are retaining walls, never ramps.
  // Mesh patches (roughness or bilinear cross-slope: no single plane fits)
  // build as tile grids instead — each tile samples the canonical height +
  // gradient at its center through the full patch/slab stack (later wins),
  // so overlaps resolve identically in engine and survey. Tile pitch is
  // capped at 1/4 of the shortest relief wavelength (0.95 m); larger
  // patches coarsen with a one-time notice, never silently.
  for (std::size_t pi = 0; pi < patches.size(); ++pi) {
    const TerrainPatch& p = patches[pi];
    if (!(p.x1 > p.x0 && p.y1 > p.y0)) continue;
    const bool mesh = p.roughness > 0.0 ||
                      std::abs(p.grade_xy_per_m) > 1e-12;
    if (!mesh) {
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
      continue;
    }
    const double w = p.x1 - p.x0, h = p.y1 - p.y0;
    int nx = std::max(1, static_cast<int>(std::ceil(w / 0.25)));
    int ny = std::max(1, static_cast<int>(std::ceil(h / 0.25)));
    if (nx > 48 || ny > 48) {
      if (!refuse_reported_) {
        std::fprintf(stderr,
                     "mujoco_backend: terrain patch '%s' tiles coarsened "
                     "(%dx%d capped at 48/side)\n",
                     p.patch_id.c_str(), nx, ny);
        refuse_reported_ = true;
      }
      nx = std::min(nx, 48);
      ny = std::min(ny, 48);
    }
    const double dx = w / nx, dy = h / ny;
    for (int ix = 0; ix < nx; ++ix) {
      for (int iy = 0; iy < ny; ++iy) {
        const double cx = p.x0 + (ix + 0.5) * dx;
        const double cy = p.y0 + (iy + 0.5) * dy;
        TerrainSample center = sample_terrain(patches, slabs, cx, cy);
        const double half_t = (center.height_m + 0.5) / 2;
        double q[4];
        grade_quat(center.grade_x, center.grade_y, q);
        mjsGeom* tg =
            add_geom(world,
                     "terrain" + std::to_string(pi) + "t" + std::to_string(ix) +
                         "_" + std::to_string(iy),
                     mjGEOM_BOX, dx / 2, dy / 2, std::max(0.05, half_t), 0,
                     q);
        tg->pos[0] = cx;
        tg->pos[1] = cy;
        tg->pos[2] = center.height_m -
                     std::max(0.05, half_t) *
                         std::cos(std::atan(center.grade_x)) *
                         std::cos(std::atan(center.grade_y));
        tg->friction[0] = kFloorSlidingFriction;
        tg->friction[1] = 0.01;
        tg->friction[2] = 0.001;
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
  impl_->model = model;
  impl_->data = data;
  // Brake lag shapes from the motor contract (main-stack mapping).
  const ServiceBrakeParams bp = brake_params_from_motor(motor);
  for (auto& b : impl_->brakes) b = ServiceBrakePlant{bp};
  int found_drives = 0, found_steers = 0;
  for (int i = 0; i < 4; ++i) {
    char name[16];
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
    if (i < 2) {
      std::snprintf(name, sizeof(name), "idlerj%d", i);
      const int iid = mj_name2id(model, mjOBJ_JOINT, name);
      idler_dof_[i] = (iid >= 0) ? model->jnt_dofadr[iid] : -1;
      // Idler friction target (per-step zone follow, same as drives).
      std::snprintf(name, sizeof(name), "idlertire%d", i);
      idler_geom_[i] = mj_name2id(model, mjOBJ_GEOM, name);
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
  built_signature_ = signature;
  built_geometry_version_ = geometry_version;
  ever_built_ = true;
  if (!saved_qpos.empty() && !saved_qvel.empty() &&
      static_cast<int>(saved_qpos.size()) == model->nq &&
      static_cast<int>(saved_qvel.size()) == model->nv) {
    std::copy(saved_qpos.begin(), saved_qpos.end(), data->qpos);
    std::copy(saved_qvel.begin(), saved_qvel.end(), data->qvel);
    mj_forward(model, data);
  }
  read_back();
}

}  // namespace lost_signal::sim::world

#endif  // LOST_SIGNAL_WITH_MUJOCO
