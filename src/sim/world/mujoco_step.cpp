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


// Tire-side target + ground-side value for one wheel (engine max() rule
// makes the pair take the larger). Tire carries min(global, local x env)
// so zones at/below the tire rating resolve exactly; grippier ground wins
// upward through the ground side, like the legacy floor.
//
// Audit note (friction doctrine, pinned in test_mujoco_backend): given the
// engine's max() rule this assignment is optimal — (T, L) would read
// max(T, L) and mask slick zones behind the tire rating, while (min, L)
// resolves the zone exactly in every case. Residual known limit: a worn
// tire (low T) on grippier ground reads the ground value (upward mask);
// no max-rule assignment can express a tire cap upward without lying
// about the zone, so it stays documented, never papered over with a
// plant-side clamp (that would saturate twice on top of the solver;
// report-only stands). The slip label thresholds on the same max() pair
// value the solver enforces, so evidence and physics cannot disagree.
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
  // Ackermann front idlers are unpowered rolling contacts: they resolve
  // the same zone lookup at their own footprints, never a frozen baseline
  // (a gripping idler would anchor the plant on slick ground while the
  // drives slide — same doctrine as the drives, see header audit note).
  // Ball supports keep their authored near-zero friction (normal load
  // only, deliberately ungripped).
  {
    double tire_target = mu_tire_base_, ground_mu = mu_tire_base_;
    for (int j = 0; j < idler_count_ && j < 2; ++j) {
      if (idler_geom_[j] < 0) continue;
      const double wx = body_x_ + idler_mod_x_[j] * cy - idler_mod_y_[j] * sy;
      const double wy = body_y_ + idler_mod_x_[j] * sy + idler_mod_y_[j] * cy;
      resolve_wheel_mu(mu_tire_base_, zones, materials, environment,
                       sim_time_s, wx, wy, tire_target, ground_mu);
      model->geom_friction[idler_geom_[j] * 3 + 0] = tire_target;
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
  // Traction load rides the TOTAL mass (chassis + declared payload),
  // master-style: heavier plants grip more and demand more, like the
  // engine (payload 0 today: bit-identical).
  const double load_N = (std::max(0.0, motor.vehicle_mass_kg) +
                         std::max(0.0, motor.payload_mass_kg)) *
                        g / std::max(1, drive_count_);
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
    // Tracked skid-steer scrub (main's TrackedDrive yaw scrub): Coulomb +
    // viscous torque opposing yaw from track shear. Zero params (wheeled
    // chassis) write zero — today's behavior bit-identical. Assigned, never
    // accumulated: qfrc_applied[5] is ours alone on non-mecanum layouts
    // (mecanum adds its roller torque above first).
    {
      const double scrub_c = std::max(0.0, motor.yaw_scrub_coulomb_Nm);
      const double scrub_v =
          std::max(0.0, motor.yaw_scrub_viscous_Nm_per_radps);
      if (scrub_c > 0.0 || scrub_v > 0.0) {
        const double wyaw = data->qvel[5];
        const double scrub_t =
            scrub_c * std::tanh(wyaw / 0.1) + scrub_v * wyaw;
        if (is_mecanum) {
          data->qfrc_applied[5] -= scrub_t;
        } else {
          data->qfrc_applied[5] = -scrub_t;
        }
      }
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

}  // namespace lost_signal::sim::world

#endif  // LOST_SIGNAL_WITH_MUJOCO
