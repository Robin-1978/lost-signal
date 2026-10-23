#include "sim/world/mujoco_backend.hpp"

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
                                         double track, double base_len) {
  std::vector<PlacedModule> modules;
  if (drive_count == 2 && steer_count == 0) {
    modules.push_back({0.0, track / 2});
    modules.push_back({0.0, -track / 2});
  } else if (drive_count == 2 && steer_count == 2) {
    // Rear drives + front steered IDLERS (free-spinning, no torque):
    // the two drive duties are the rear axle, like the main stack.
    modules.push_back({-base_len / 2, track / 2});
    modules.push_back({-base_len / 2, -track / 2});
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

// Tire-side target + ground-side value for one wheel (engine max() rule
// makes the pair take the larger). Tire carries min(global, local x env)
// so zones at/below the tire rating resolve exactly; grippier ground wins
// upward through the ground side, like the legacy floor.
void resolve_wheel_mu(double mu_tire_base,
                      const std::vector<MaterialZone>& zones,
                      const std::vector<MaterialEntry>& materials,
                      const std::vector<EnvironmentKeyframe>& environment,
                      double sim_time_s, double wx, double wy,
                      double& tire_target, double& ground_mu) {
  tire_target = mu_tire_base;
  ground_mu = mu_tire_base;
  const MaterialZone* zone = material_zone_at(zones, wx, wy);
  if (zone == nullptr) return;
  const MaterialEntry* mat = find_material(materials, zone->material_id);
  if (mat == nullptr) return;
  const double grip =
      environment_grip_at(environment, mat->material_id, sim_time_s);
  const double local = std::max(0.0, mat->friction_mu) * std::max(0.0, grip);
  ground_mu = local;
  tire_target = std::min(mu_tire_base, local);
}

}  // namespace

struct MujocoBackend::Impl {
  mjModel* model = nullptr;
  mjData* data = nullptr;
  std::array<ServiceBrakePlant, 4> brakes;
  ~Impl() {
    if (data != nullptr) mj_deleteData(data);
    if (model != nullptr) mj_deleteModel(model);
  }
};

MujocoBackend::MujocoBackend() : impl_(std::make_unique<Impl>()) {}

MujocoBackend::~MujocoBackend() = default;

MujocoBackend::MujocoBackend(MujocoBackend&&) = default;

MujocoBackend& MujocoBackend::operator=(MujocoBackend&&) = default;

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
  // Mesh-only features refuse the build fail-closed: a silently smoothed
  // surface would lie to both the contact solver and the survey.
  for (const auto& p : patches) {
    if (p.roughness > 0.0 || std::abs(p.grade_xy_per_m) > 1e-12) {
      if (!refuse_reported_) {
        std::fprintf(stderr,
                     "mujoco_backend: terrain patch '%s' needs mesh support "
                     "(roughness/bilinear); build refused\n",
                     p.patch_id.c_str());
        refuse_reported_ = true;
      }
      drive_count_ = 0;
      steer_count_ = 0;
      ready_ = false;
      return;
    }
  }
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
      << "|" << drive_count << "|" << steer_count << "|" << boxes.size()
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
        << p.grade_y << ";";
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

  const double r = std::max(1e-3, motor.wheel_radius_m);
  const double track = std::max(0.1, motor.track_width_m);
  const double base_len = std::max(0.1, motor.wheel_base_m);
  const double base_mass =
      std::max(0.5, motor.vehicle_mass_kg - 0.2 * drive_count);
  const double deck_half_h = 0.05;
  // Central mass tub, clear of every wheel swept volume: wheels reach
  // |y| >= track/2 - width and |x| >= base_len/2 - radius, so the tub
  // stays well inside both. (A full-deck box overlapped the tire corners
  // by millimetres; the initial penetration resolved explosively and
  // masked as a steering instability. Rendering shells belong to M3;
  // this box is ballast only.)
  const double tub_half_xy = 0.12;
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
  zone_geom_.clear();

  // Wheel modules per layout: one shared table for the build (bodies)
  // and the step (wheel world positions for the zone lookup).
  const std::vector<PlacedModule> modules =
      layout_modules(drive_count, steer_count, track, base_len);
  for (const auto& mod : modules) {
    if (mod.is_drive && mod.drive_idx >= 0 && mod.drive_idx < 4) {
      drive_mod_x_[mod.drive_idx] = mod.x;
      drive_mod_y_[mod.drive_idx] = mod.y;
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
      kingpin->mass = 0.1;
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
                               mjGEOM_CYLINDER, r, 0.04, 0.0, 1);
      tire->quat[0] = kQuatX90[0];
      tire->quat[1] = kQuatX90[1];
      tire->quat[2] = kQuatX90[2];
      tire->quat[3] = kQuatX90[3];
      tire->density = 400.0;
      tire->friction[0] = mu_tire;
      tire->friction[1] = kWheelTorsionalFriction;
      tire->friction[2] = kWheelRollingFriction;
    } else if (modules[m].idle) {
      // Ackermann front idlers: free-spinning wheels inside the steering
      // knuckles (no torque — the two drive duties are the rear axle).
      // Mechanical trail (wheel center behind the kingpin, like a real
      // caster): zero trail is neutrally stable and solver noise walks it
      // into a mirrored wedge; trail makes straight-ahead the stable
      // equilibrium the servo then works against, like a real car.
      constexpr double kMechanicalTrailM = 0.02;
      mjsBody* idler = add_body(knuckle, "idler" + std::to_string(idle_idx),
                                -kMechanicalTrailM, 0.0, 0.0);
      add_hinge(idler, "idlerj" + std::to_string(idle_idx), 0.0, 1.0, 0.0);
      mjsGeom* itire = add_geom(idler, "idlertire" + std::to_string(idle_idx),
                                mjGEOM_CYLINDER, r, 0.04, 0.0, 1);
      itire->quat[0] = kQuatX90[0];
      itire->quat[1] = kQuatX90[1];
      itire->quat[2] = kQuatX90[2];
      itire->quat[3] = kQuatX90[3];
      itire->density = 400.0;
      itire->friction[0] = mu;
      itire->friction[1] = kWheelTorsionalFriction;
      itire->friction[2] = kWheelRollingFriction;
      ++idle_idx;
    }
  }
  // Ball casters fore/aft, like the real differential base: unactuated
  // balls that roll instead of gripping, so motor reaction torque settles
  // into weight transfer instead of slamming the deck onto the floor.
  const double caster_r = 0.03;
  const double caster_x = base_len / 2 * 0.9;
  const double caster_z = -(r - caster_r);
  for (int i = 0; i < 2; ++i) {
    const double cx = (i == 0 ? caster_x : -caster_x);
    mjsBody* caster = add_body(base, i == 0 ? "caster_f" : "caster_r", cx, 0.0,
                               caster_z);
    mjsJoint* ball = mjs_addJoint(caster, nullptr);
    set_name(ball->element, i == 0 ? "casterj0" : "casterj1");
    ball->type = mjJNT_BALL;
    mjsGeom* ball_g = add_geom(caster, i == 0 ? "ball0" : "ball1",
                               mjGEOM_SPHERE, caster_r, 0.0, 0.0, 1);
    ball_g->mass = 0.1;
    ball_g->friction[0] = 0.05;
    ball_g->friction[1] = 0.005;
    ball_g->friction[2] = 0.0005;
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
  }
  zone_geom_.clear();
  for (std::size_t zi = 0; zi < zones.size(); ++zi) {
    const std::string name = "zone" + std::to_string(zi);
    zone_geom_.push_back(mj_name2id(model, mjOBJ_GEOM, name.c_str()));
  }
  if (found_drives != drive_count || found_steers != steer_count) {
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

void MujocoBackend::step(double dt_s, const MotorParams& motor,
                         const std::array<double, 4>& drive_duty,
                         const std::array<double, 4>& steer_duty,
                         const std::array<double, 4>& brake_request,
                         double incline_grade,
                         const std::vector<MaterialZone>& zones,
                         const std::vector<MaterialEntry>& materials,
                         const std::vector<EnvironmentKeyframe>& environment,
                         double sim_time_s) {
  if (!valid() || drive_count_ <= 0 || !(dt_s > 0.0)) return;
  mjModel* model = impl_->model;
  mjData* data = impl_->data;
  // Per-step ground truth: zone friction follows the environment grip
  // (live model-array writes, no rebuild), tire sides follow the local
  // zone at each wheel. Stale (20 ms) body pose is fine — zones are meters.
  const double cy = std::cos(body_yaw_), sy = std::sin(body_yaw_);
  double tire_target[4] = {mu_tire_base_, mu_tire_base_, mu_tire_base_,
                           mu_tire_base_};
  double ground_mu[4] = {mu_tire_base_, mu_tire_base_, mu_tire_base_,
                         mu_tire_base_};
  for (int d = 0; d < drive_count_ && d < 4; ++d) {
    const double wx =
        body_x_ + drive_mod_x_[d] * cy - drive_mod_y_[d] * sy;
    const double wy =
        body_y_ + drive_mod_x_[d] * sy + drive_mod_y_[d] * cy;
    resolve_wheel_mu(mu_tire_base_, zones, materials, environment, sim_time_s,
                     wx, wy, tire_target[d], ground_mu[d]);
    if (tire_geom_[d] >= 0) {
      model->geom_friction[tire_geom_[d] * 3 + 0] = tire_target[d];
    }
  }
  for (std::size_t zi = 0; zi < zones.size() && zi < zone_geom_.size();
       ++zi) {
    if (zone_geom_[zi] < 0) continue;
    const MaterialEntry* mat =
        find_material(materials, zones[zi].material_id);
    if (mat == nullptr) continue;
    model->geom_friction[zone_geom_[zi] * 3 + 0] =
        std::max(0.0, mat->friction_mu) *
        std::max(0.0,
                 environment_grip_at(environment, mat->material_id,
                                     sim_time_s));
  }
  const double r = std::max(1e-6, motor.wheel_radius_m);
  // Grade as gravity tilt around the current heading (main-stack
  // body-forward semantics: uphill forward pulls backward).
  const double g = 9.81;
  const double grade = std::isfinite(incline_grade) ? incline_grade : 0.0;
  model->opt.gravity[0] = -g * grade * std::cos(body_yaw_);
  model->opt.gravity[1] = -g * grade * std::sin(body_yaw_);
  model->opt.gravity[2] = -g;
  // Main-stack duty path, verbatim: duty-implied motor force only (the
  // resistance/traction-limit subtraction lives on main's PID path, not
  // the duty our actuator commands take). Rolling losses come from the
  // engine contact parameters. Slip is reported, never clamped here.
  //
  // Split-step contract, verbatim from main's physics step: mj_step1
  // computes position/velocity-dependent quantities first, then the
  // freshly resolved controls and brake forces are accepted before the
  // acceleration/constraint solve in mj_step2. qfrc_applied is recomputed
  // every substep and zeroed at the end, exactly like main.
  const double n_sub = std::round(dt_s / 0.001);
  const int n = std::max(1, std::min(static_cast<int>(n_sub), 1000));
  const double step_dt = dt_s / n;
  const double load_N =
      std::max(0.0, motor.vehicle_mass_kg) * g / std::max(1, drive_count_);
  // Traction evidence, main's motor_drive flag: the applied force
  // past the local pair capacity reports slipping. Reported only — the
  // duty path never clamps (verbatim main). Capacity is the engine max()
  // rule over the tire/ground sides resolved above.
  //
  // Brake lag state advances at the outer 20 ms cadence (same as main's
  // per-tick plant step); capacity resolves per substep below.
  double brake_capacity_Nm[4] = {0.0, 0.0, 0.0, 0.0};
  for (int i = 0; i < drive_count_; ++i) {
    brake_capacity_Nm[i] =
        impl_->brakes[static_cast<std::size_t>(i)].step(brake_request[i],
                                                        dt_s);
  }
  // Low-speed smoothing for the brake fade (matches the ServiceBrakePlant
  // default; M2 feeds it from the drive profile like main's content data).
  constexpr double kBrakeLowSpeedSmoothingRadps = 0.6;
  // Mecanum roller stand-in (documented simplification): plain cylinders
  // cannot push sideways, so the 45-degree rollers' lateral effect is
  // applied as a body wrench with the virtual-work dual of the mecanum
  // forward map (same coefficients as the kinematics, power-consistent).
  // Fore-aft stays in the engine (no double count: the spin pattern sums
  // ~0 fore-aft). Explicit roller bodies are M3 work.
  constexpr double kRollerCos = 0.7071067811865476;
  const bool is_mecanum = (drive_count_ == 4 && steer_count_ == 0);
  double motor_N_last[4] = {0.0, 0.0, 0.0, 0.0};
  for (int s = 0; s < n; ++s) {
    model->opt.timestep = step_dt;
    mj_step1(model, data);
    double roller_fx = 0.0, roller_fy = 0.0, roller_t = 0.0;
    for (int d = 0; d < drive_count_; ++d) {
      const int dof = drive_dof_[d];
      if (dof < 0) continue;
      data->qfrc_applied[dof] = 0.0;
      const double v = data->qvel[dof] * r;
      const double motor_N = dc_motor_force_N(drive_duty[d], v,
                                              motor.stall_force_N,
                                              motor.free_speed_mps);
      data->qfrc_applied[dof] = motor_N * r;
      motor_N_last[d] = motor_N;
      if (brake_capacity_Nm[d] > 0.0) {
        const double direction = std::clamp(
            data->qvel[dof] / kBrakeLowSpeedSmoothingRadps, -1.0, 1.0);
        data->qfrc_applied[dof] -= direction * brake_capacity_Nm[d];
      }
      if (is_mecanum) {
        // Roller signs follow the mecanum map rows (fore, left, yaw):
        // Fx = F0+F1+F2+F3, Fy = -F0+F1+F2-F3,
        // T = radius*(-F0+F1-F2+F3), roller-cosine scaled.
        roller_fx += motor_N * kRollerCos;
        const double sign =
            (d == 0 || d == 3) ? -1.0 : 1.0;
        const double tsign =
            (d == 0 || d == 2) ? -1.0 : 1.0;
        roller_fy += sign * motor_N * kRollerCos;
        roller_t += tsign * motor_N * kRollerCos * (track_m_ / 2 + base_len_m_ / 2) *
                    0.5;
      }
    }
    for (int t = 0; t < steer_count_; ++t) {
      const int dof = steer_dof_[t];
      if (dof < 0) continue;
      data->qfrc_applied[dof] = 0.0;
      const double vel = data->qvel[dof];
      // Steering servo torque, main's Ackermann form: duty-implied rate
      // against the velocity response curve, full authority opposing.
      const bool motoring = steer_duty[t] * vel > 0.0;
      const double curve =
          motoring ? std::max(0.0, 1.0 - std::abs(vel) /
                                             std::max(0.05, motor.max_steer_rate_radps))
                   : 1.0;
      data->qfrc_applied[dof] =
          steer_duty[t] * motor.steer_stall_torque_Nm * curve;
    }
    if (is_mecanum) {
      // Body-frame wrench -> world frame via the current yaw. The free
      // joint owns dofs 0..5 (x,y,z,roll,pitch,yaw); nothing else writes
      // them, so assign (not accumulate) each substep.
      const double qw = data->qpos[3], qx = data->qpos[4],
                   qy = data->qpos[5], qz = data->qpos[6];
      const double yaw = std::atan2(2.0 * (qw * qz + qx * qy),
                                    1.0 - 2.0 * (qy * qy + qz * qz));
      const double c = std::cos(yaw), s = std::sin(yaw);
      data->qfrc_applied[0] = roller_fx * c - roller_fy * s;
      data->qfrc_applied[1] = roller_fx * s + roller_fy * c;
      data->qfrc_applied[5] = roller_t;
    }
    mj_step2(model, data);
  }
  // Traction evidence, main's motor_drive flag: the settled (last-substep)
  // applied force past the local pair capacity reports slipping. Reported
  // only — the duty path never clamps (verbatim main). Confirmed over two
  // consecutive steps so one transient step cannot latch the label.
  for (int d = 0; d < drive_count_ && d < 4; ++d) {
    const double traction_N =
        std::max(tire_target[d], ground_mu[d]) * load_N;
    if (std::abs(motor_N_last[d]) > traction_N) {
      slip_count_[d] = std::min(slip_count_[d] + 1, 3);
    } else {
      slip_count_[d] = 0;
    }
    drive_slipping_[static_cast<std::size_t>(d)] = slip_count_[d] >= 2;
  }
  model->opt.timestep = 0.001;
  for (int i = 0; i < model->nv; ++i) data->qfrc_applied[i] = 0.0;
  mj_forward(model, data);
  read_back();
}

bool MujocoBackend::raycast(double ox, double oy, double oz, double dx,
                           double dy, double dz, double max_m,
                           double& dist_m) const {
  if (!valid() || !(max_m > 0.0)) return false;
  const double len =
      std::sqrt(dx * dx + dy * dy + dz * dz);
  if (!(len > 1e-12) || !std::isfinite(len)) return false;
  mjModel* model = impl_->model;
  mjData* data = impl_->data;
  if (model == nullptr || data == nullptr) return false;
  const mjtNum pnt[3] = {ox, oy, oz};
  const mjtNum vec[3] = {dx / len, dy / len, dz / len};
  // Group 0 only: floor + static boxes. Group 1 (deck, wheels, casters)
  // is the querying robot itself and stays out of its own sensor.
  const mjtByte groups[mjNGROUP] = {1, 0, 0, 0, 0, 0};
  int geomid = -1;
  mjtNum dist[3] = {0, 0, 0};
  const mjtNum d =
      mj_ray(model, data, pnt, vec, groups, 1, -1, &geomid, dist);
  if (d < 0.0 || geomid < 0 || d > max_m) return false;
  dist_m = static_cast<double>(d);
  return true;
}

void MujocoBackend::read_back() {
  if (!valid()) return;
  mjModel* model = impl_->model;
  mjData* data = impl_->data;
  body_x_ = data->qpos[0];
  body_y_ = data->qpos[1];
  body_z_ = data->qpos[2];
  const double qw = data->qpos[3], qx = data->qpos[4], qy = data->qpos[5],
               qz = data->qpos[6];
  body_yaw_ = std::atan2(2.0 * (qw * qz + qx * qy),
                         1.0 - 2.0 * (qy * qy + qz * qz));
  body_roll_ = std::atan2(2.0 * (qw * qx + qy * qz),
                          1.0 - 2.0 * (qx * qx + qy * qy));
  const double sinp = std::clamp(2.0 * (qw * qy - qz * qx), -1.0, 1.0);
  body_pitch_ = std::asin(sinp);
  const double vx = data->qvel[0], vy = data->qvel[1];
  forward_mps_ = vx * std::cos(body_yaw_) + vy * std::sin(body_yaw_);
  lateral_mps_ = -vx * std::sin(body_yaw_) + vy * std::cos(body_yaw_);
  yaw_rate_ = data->qvel[5];
  for (int d = 0; d < drive_count_; ++d) {
    const int dof = drive_dof_[d];
    if (dof < 0) continue;
    // Hinge DOF address == joint qpos address for these joints.
    const int jid = mj_name2id(model, mjOBJ_JOINT,
                               d == 0 ? "drive0"
                               : d == 1 ? "drive1"
                               : d == 2 ? "drive2"
                                        : "drive3");
    const int qadr = (jid >= 0) ? model->jnt_qposadr[jid] : -1;
    if (qadr >= 0) drive_angle_[d] = data->qpos[qadr];
    drive_radps_[d] = data->qvel[dof];
  }
  for (int t = 0; t < steer_count_; ++t) {
    const int dof = steer_dof_[t];
    if (dof < 0) continue;
    const int jid = mj_name2id(model, mjOBJ_JOINT,
                               t == 0 ? "steer0"
                               : t == 1 ? "steer1"
                               : t == 2 ? "steer2"
                                        : "steer3");
    const int qadr = (jid >= 0) ? model->jnt_qposadr[jid] : -1;
    if (qadr >= 0) steer_angle_[t] = data->qpos[qadr];
    steer_vel_[t] = data->qvel[dof];
  }
}

}  // namespace lost_signal::sim::world

#endif  // LOST_SIGNAL_WITH_MUJOCO
