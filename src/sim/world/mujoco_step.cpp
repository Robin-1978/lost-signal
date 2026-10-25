// MuJoCo duty loop: per-step friction resolve plus
// MujocoBackend::step (substepped duties, brake, wrench, slip evidence).
// Plant assembly lives in mujoco_build.cpp.
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
#include <vector>

#include <mujoco/mujoco.h>

#include "sim/world/motor.hpp"
#include "sim/world/service_brake.hpp"
#include "sim/world/terrain.hpp"
#include "sim/world/world.hpp"

namespace lost_signal::sim::world {

namespace {


// Tire-side target + ground observation for one wheel. Tire geoms have
// higher MuJoCo contact priority, so the resolved coefficient is the
// physical series limit min(tire, ground x environment) on every surface,
// including a height field. Material zones therefore remain properties,
// never raised coplanar collision slabs.
struct ResolvedContactMaterial {
 double tire_mu = 0.0;
 double rolling_arm_m = 0.0;
 double time_constant_s = 0.02;
 double damping_ratio = 1.0;
};

ResolvedContactMaterial resolve_wheel_contact(double mu_tire_base,
 double default_rolling_arm_m,
                      const std::vector<MaterialZone>& zones,
                      const std::vector<MaterialEntry>& materials,
                      const std::vector<EnvironmentKeyframe>& environment,
 double sim_time_s, double wx, double wy) {
 ResolvedContactMaterial out;
 out.tire_mu = mu_tire_base;
 out.rolling_arm_m = default_rolling_arm_m;
  const MaterialZone* zone = material_zone_at(zones, wx, wy);
 if (zone == nullptr) return out;
  const MaterialEntry* mat = find_material(materials, zone->material_id);
 if (mat == nullptr) return out;
  const double grip =
      environment_grip_at(environment, mat->material_id, sim_time_s);
  const double local = std::max(0.0, mat->friction_mu) * std::max(0.0, grip);
 out.tire_mu = std::min(mu_tire_base, local);
 out.rolling_arm_m = std::max(0.0, mat->rolling_resist);
 out.time_constant_s = std::max(1e-4, mat->contact_time_constant_s);
 out.damping_ratio = std::max(0.0, mat->contact_damping_ratio);
 return out;
}

}  // namespace

void MujocoBackend::step(double dt_s, const MotorParams& motor,
                         const std::array<double, 4>& drive_duty,
                         const std::array<double, 4>& steer_duty,
                         const std::array<double, 4>& brake_request,
                         double incline_grade,
                         const std::vector<MaterialZone>& zones,
                         const std::vector<MaterialEntry>& materials,
                         const std::vector<EnvironmentKeyframe>& environment,
                         double sim_time_s) {
 impact_ = ImpactViz{};
 if (!valid() || drive_count_ <= 0 || !(dt_s > 0.0)) return;
  mjModel* model = impl_->model;
  mjData* data = impl_->data;
 const double r = std::max(1e-6, motor.wheel_radius_m);
 auto apply_contact_material = [&](int geom_id,
 const ResolvedContactMaterial& contact) {
 if (geom_id < 0) return;
 model->geom_friction[geom_id * 3 + 0] = contact.tire_mu;
 model->geom_friction[geom_id * 3 + 2] =
 std::clamp(contact.rolling_arm_m * r, 0.0, 0.05);
 model->geom_solref[geom_id * mjNREF + 0] = contact.time_constant_s;
 model->geom_solref[geom_id * mjNREF + 1] = contact.damping_ratio;
 };
  // Per-step ground truth: zone friction follows the environment grip
  // (live model-array writes, no rebuild), tire sides follow the local
  // zone at each wheel. Stale (20 ms) body pose is fine — zones are meters.
  const double cy = std::cos(body_yaw_), sy = std::sin(body_yaw_);
  for (int d = 0; d < drive_count_ && d < 4; ++d) {
    const double wx =
        body_x_ + drive_mod_x_[d] * cy - drive_mod_y_[d] * sy;
    const double wy =
        body_y_ + drive_mod_x_[d] * sy + drive_mod_y_[d] * cy;
 const auto contact = resolve_wheel_contact(
 mu_tire_base_, motor.rolling_resist, zones, materials, environment,
 sim_time_s, wx, wy);
 apply_contact_material(tire_geom_[d], contact);
  }
  // Ackermann front idlers are unpowered rolling contacts: they resolve
  // the same zone lookup at their own footprints, never a frozen baseline
  // (a gripping idler would anchor the plant on slick ground while the
  // drives slide — same doctrine as the drives, see header audit note).
 // Passive caster friction stays at the authored low baseline until the
 // footprint-specific material lookup below is applied.
  {
    for (int j = 0; j < idler_count_ && j < 2; ++j) {
      if (idler_geom_[j] < 0) continue;
      const double wx = body_x_ + idler_mod_x_[j] * cy - idler_mod_y_[j] * sy;
      const double wy = body_y_ + idler_mod_x_[j] * sy + idler_mod_y_[j] * cy;
 const auto contact = resolve_wheel_contact(
 mu_tire_base_, motor.rolling_resist, zones, materials, environment,
 sim_time_s, wx, wy);
 apply_contact_material(idler_geom_[j], contact);
    }
  }
 // Passive caster tires follow the same material/environment contact lookup
 // as drive and Ackermann idler tires. Their footprint includes the live
 // fork swivel and configured trail, so engine contact and visual contact
 // remain on the same authored patch.
 for (int i = 0; i < support_count_ && i < 4; ++i) {
 if (support_geom_[i] < 0) continue;
 const double ct = std::cos(support_swivel_[i]);
 const double st = std::sin(support_swivel_[i]);
 const double bx = support_anchor_x_[i] - support_trail_[i] * ct;
 const double by = support_anchor_y_[i] - support_trail_[i] * st;
 const double wx = body_x_ + bx * cy - by * sy;
 const double wy = body_y_ + bx * sy + by * cy;
 const auto contact = resolve_wheel_contact(
    support_mu_base_[i], support_rolling_base_m_[i], zones, materials, environment,
 sim_time_s, wx, wy);
 apply_contact_material(support_geom_[i], contact);
 for (int shoulder = 0; shoulder < 2; ++shoulder) {
 const int gid = support_shoulder_geom_[i][shoulder];
 apply_contact_material(gid, contact);
 }
 }
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
  // Brake lag state advances at the outer 20 ms cadence (same as main's
  // per-tick plant step); capacity resolves per substep below.
  double brake_capacity_Nm[4] = {0.0, 0.0, 0.0, 0.0};
  for (int i = 0; i < drive_count_; ++i) {
    brake_capacity_Nm[i] =
        impl_->brakes[static_cast<std::size_t>(i)].step(brake_request[i],
                                                        dt_s);
  }
  // A service brake is a bounded passive joint-friction constraint. Let
  // MuJoCo resolve sticking at zero speed: applying an explicit signed
  // torque from the pre-step velocity can cross zero inside a substep and
  // make a strong brake alternate direction, injecting a low-speed creep.
  // frictionloss has torque units for a hinge and enforces exactly the
  // requested Coulomb capacity without ever driving the shaft.
  for (int d = 0; d < drive_count_; ++d) {
    const int dof = drive_dof_[d];
    if (dof >= 0) model->dof_frictionloss[dof] = brake_capacity_Nm[d];
  }
  // Mecanum roller stand-in (documented simplification): plain cylinders
  // cannot push sideways, so the 45-degree rollers' lateral effect is
  // applied as a body wrench with the virtual-work dual of the mecanum
  // forward map (same coefficients as the kinematics, power-consistent).
  // Fore-aft stays in the engine (no double count: the spin pattern sums
  // ~0 fore-aft). Explicit roller bodies are M3 work.
  constexpr double kRollerCos = 0.7071067811865476;
  const bool is_mecanum = (drive_count_ == 4 && steer_count_ == 0);
 const double utilization_threshold = 1.0 - std::sqrt(std::clamp(
 model->opt.tolerance, 1e-12, 1e-2));
 struct ContactWindow {
 bool valid = false;
 double peak_normal = 0.0;
 double peak_utilization = 0.0;
 double saturated_time_s = 0.0;
 double confirmation_time_s = 0.0;
 } contact_window[4];
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
 // Body-frame wrench -> world frame via the current yaw. The named base
 // free joint owns six consecutive dofs; nothing else writes them, so
 // assign (not accumulate) each substep.
    const double qw = data->qpos[base_qpos_ + 3];
    const double qx = data->qpos[base_qpos_ + 4];
    const double qy = data->qpos[base_qpos_ + 5];
    const double qz = data->qpos[base_qpos_ + 6];
      const double yaw = std::atan2(2.0 * (qw * qz + qx * qy),
                                    1.0 - 2.0 * (qy * qy + qz * qz));
      const double c = std::cos(yaw), s = std::sin(yaw);
    data->qfrc_applied[base_dof_ + 0] = roller_fx * c - roller_fy * s;
    data->qfrc_applied[base_dof_ + 1] = roller_fx * s + roller_fy * c;
    data->qfrc_applied[base_dof_ + 5] = roller_t;
    }
    // Tracked skid-steer scrub (main's TrackedDrive yaw scrub): Coulomb +
    // viscous torque opposing yaw from track shear. Zero params (wheeled
    // chassis) write zero — today's behavior bit-identical. Assigned, never
 // accumulated: the base yaw wrench is ours alone on non-mecanum layouts
    // (mecanum adds its roller torque above first).
    {
      const double scrub_c = std::max(0.0, motor.yaw_scrub_coulomb_Nm);
      const double scrub_v =
          std::max(0.0, motor.yaw_scrub_viscous_Nm_per_radps);
      if (scrub_c > 0.0 || scrub_v > 0.0) {
    const double wyaw = data->qvel[base_dof_ + 5];
        const double scrub_t =
            scrub_c * std::tanh(wyaw / 0.1) + scrub_v * wyaw;
        if (is_mecanum) {
      data->qfrc_applied[base_dof_ + 5] -= scrub_t;
        } else {
      data->qfrc_applied[base_dof_ + 5] = -scrub_t;
        }
      }
    }
    mj_step2(model, data);
 // Integrate only robot-to-authored-static contact forces. Ground,
 // terrain, material zones and support slabs are deliberately absent from
 // static_geom_, so normal wheel loading can never masquerade as a crash.
 // This is presentation truth only and has no route into ROBOT inputs.
 for (int ci = 0; ci < data->ncon; ++ci) {
 const mjContact& contact = data->contact[ci];
 const bool first_static = std::find(static_geom_.begin(), static_geom_.end(),
 contact.geom1) != static_geom_.end();
 const bool second_static = std::find(static_geom_.begin(), static_geom_.end(),
 contact.geom2) != static_geom_.end();
 if (first_static == second_static) continue;
 const int robot_geom = first_static ? contact.geom2 : contact.geom1;
 const int robot_body = model->geom_bodyid[robot_geom];
 if (robot_body <= 0) continue;
 mjtNum force[6] = {};
 mj_contactForce(model, data, ci, force);
 const double normal_N = std::max(0.0, static_cast<double>(force[0]));
 const double impulse_Ns = normal_N * step_dt;
 if (!(impulse_Ns > 0.0)) continue;

 mjtNum spatial[6] = {};
 mj_objectVelocity(model, data, mjOBJ_BODY, robot_body, spatial, 0);
 const mjtNum* center = data->xpos + 3 * robot_body;
 const double rx = contact.pos[0] - center[0];
 const double ry = contact.pos[1] - center[1];
 const double rz = contact.pos[2] - center[2];
 const double vx = spatial[3] + spatial[1] * rz - spatial[2] * ry;
 const double vy = spatial[4] + spatial[2] * rx - spatial[0] * rz;
 const double vz = spatial[5] + spatial[0] * ry - spatial[1] * rx;
 const double relative_speed = std::sqrt(vx * vx + vy * vy + vz * vz);
 const double accumulated = impact_.normal_impulse_Ns;
 const double total = accumulated + impulse_Ns;
 impact_.x_m =
 (impact_.x_m * accumulated + contact.pos[0] * impulse_Ns) / total;
 impact_.y_m =
 (impact_.y_m * accumulated + contact.pos[1] * impulse_Ns) / total;
 impact_.z_m =
 (impact_.z_m * accumulated + contact.pos[2] * impulse_Ns) / total;
 impact_.normal_impulse_Ns = total;
 impact_.relative_speed_mps =
 std::max(impact_.relative_speed_mps, relative_speed);
 impact_.valid = true;
 }
 // Contacts can make and break inside the public 20 ms plant step. Inspect
 // every resolved 1 ms force solve so a sliding wheel is not relabelled
 // merely because the final substep happens to be unloaded. This tight loop
 // reads only contact forces; full velocity/pose telemetry is sampled once
 // below, keeping accelerated headless runs cheap.
 for (int d = 0; d < drive_count_ && d < 4; ++d) {
 const int geom_id = tire_geom_[d];
 if (geom_id < 0) continue;
 bool saturated_this_substep = false;
 double response_time_s = step_dt;
 for (int ci = 0; ci < data->ncon; ++ci) {
 const mjContact& contact = data->contact[ci];
 if (contact.geom1 != geom_id && contact.geom2 != geom_id) continue;
 mjtNum force[6] = {};
 mj_contactForce(model, data, ci, force);
 const double normal = std::max(0.0, static_cast<double>(force[0]));
 if (!(normal > 0.0)) continue;
 ContactWindow& out = contact_window[d];
 out.valid = true;
 out.peak_normal = std::max(out.peak_normal, normal);
 if (contact.dim >= 3) {
 const double mu1 = std::max(1e-9, contact.friction[0]);
 const double mu2 = std::max(1e-9, contact.friction[1]);
 const double scaled1 = std::abs(static_cast<double>(force[1])) / mu1;
 const double scaled2 = std::abs(static_cast<double>(force[2])) / mu2;
 const double demand = model->opt.cone == mjCONE_ELLIPTIC
 ? std::hypot(scaled1, scaled2)
 : scaled1 + scaled2;
 const double utilization = demand / normal;
 out.peak_utilization = std::max(out.peak_utilization, utilization);
 if (utilization >= utilization_threshold) {
 saturated_this_substep = true;
 // Positive solref[0] is MuJoCo's contact time constant. A boundary touch
 // shorter than one contact response is solver settling, not sustained
 // traction loss. Direct-format (negative) solref has no time constant,
 // so the public plant step is the confirmation horizon.
 const double contact_response_s = contact.solref[0] > 0.0
 ? static_cast<double>(contact.solref[0])
 : dt_s;
 response_time_s = std::max(response_time_s, contact_response_s);
 }
 }
 if (saturated_this_substep) {
 ContactWindow& out = contact_window[d];
 out.saturated_time_s += step_dt;
 out.confirmation_time_s = std::max(
 out.confirmation_time_s, response_time_s);
 }
 }
 }
  }
  model->opt.timestep = 0.001;
  for (int i = 0; i < model->nv; ++i) data->qfrc_applied[i] = 0.0;
  mj_forward(model, data);
  read_back();
 // Traction saturation is read from the resolved MuJoCo contact force and
 // that contact's actual normal load/friction cone. This captures load
 // transfer, split-mu and loss of contact; declared total mass divided by
 // wheel count cannot. The cone boundary comes from the active solver
 // tolerance and the persistence horizon from contact solref. Accumulating
 // seconds instead of outer-call counts keeps the label invariant when a
 // sensor cadence changes how completed physics ticks are sampled.
 for (int d = 0; d < drive_count_ && d < 4; ++d) {
 const bool saturated = contact_window[d].valid &&
 contact_window[d].peak_normal > 0.0 &&
 contact_window[d].peak_utilization >= utilization_threshold;
 if (saturated) {
 slip_saturated_time_s_[d] += contact_window[d].saturated_time_s;
 slip_confirmation_time_s_[d] = std::max(
 slip_confirmation_time_s_[d], contact_window[d].confirmation_time_s);
 } else {
 slip_saturated_time_s_[d] = 0.0;
 slip_confirmation_time_s_[d] = 0.0;
 }
 drive_slipping_[static_cast<std::size_t>(d)] =
 slip_confirmation_time_s_[d] > 0.0 &&
 slip_saturated_time_s_[d] +
 std::numeric_limits<double>::epsilon() * dt_s >=
 slip_confirmation_time_s_[d];
 }
}

}  // namespace lost_signal::sim::world

#endif  // LOST_SIGNAL_WITH_MUJOCO
