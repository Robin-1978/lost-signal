// MuJoCo duty loop: per-step friction resolve plus
// MujocoBackend::step (substepped duties, brake, wrench, slip evidence).
// Plant assembly lives in mujoco_build.cpp.
#include "sim/world/mujoco_backend.hpp"
#include "sim/world/mujoco_backend_detail.hpp"

#ifdef LOST_SIGNAL_WITH_MUJOCO

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iterator>
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

// Solve a dense system of at most four equations with partial pivoting.
// Mecanum's constraint matrix is positive definite while every wheel has
// finite inertia; rejecting a numerically singular active set is safer than
// turning it into an unbounded impulse.
bool solve_small_system(double matrix[4][4], double rhs[4], int size,
 double solution[4]) {
 double scale = 0.0;
 for (int row = 0; row < size; ++row) {
 for (int col = 0; col < size; ++col) {
 scale = std::max(scale, std::abs(matrix[row][col]));
 }
 }
 constexpr double kPivotSafetyUlps = 64.0;
 const double pivot_floor = std::numeric_limits<double>::epsilon() *
 std::max(1.0, scale) * kPivotSafetyUlps;
 for (int col = 0; col < size; ++col) {
 int pivot = col;
 for (int row = col + 1; row < size; ++row) {
 if (std::abs(matrix[row][col]) > std::abs(matrix[pivot][col])) {
 pivot = row;
 }
 }
 if (!(std::abs(matrix[pivot][col]) > pivot_floor)) return false;
 if (pivot != col) {
 for (int k = col; k < size; ++k) {
 std::swap(matrix[col][k], matrix[pivot][k]);
 }
 std::swap(rhs[col], rhs[pivot]);
 }
 for (int row = col + 1; row < size; ++row) {
 const double factor = matrix[row][col] / matrix[col][col];
 matrix[row][col] = 0.0;
 for (int k = col + 1; k < size; ++k) {
 matrix[row][k] -= factor * matrix[col][k];
 }
 rhs[row] -= factor * rhs[col];
 }
 }
 for (int row = size - 1; row >= 0; --row) {
 double value = rhs[row];
 for (int col = row + 1; col < size; ++col) {
 value -= matrix[row][col] * solution[col];
 }
 solution[row] = value / matrix[row][row];
 }
 return true;
}

// Exact four-variable box QP by active-face enumeration:
// min 0.5 p' W p - slip' p, |p_i| <= bound_i.
// The 3^4 search is fixed work, treats all wheels symmetrically, and avoids
// the wheel-order bias of sequential impulse clipping.
std::array<double, 4> solve_box_impulses(
 const double w[4][4], const double slip[4], const double bound[4]) {
 std::array<double, 4> best{};
 double best_cost = 0.0; // p=0 is always feasible.
 constexpr int kActiveSets = 81; // 3^4: free, lower, upper per wheel.
 constexpr double kFeasibilitySafetyUlps = 128.0;
 for (int encoded = 0; encoded < kActiveSets; ++encoded) {
 int state[4] = {};
 int free_index[4] = {};
 int free_count = 0;
 int digits = encoded;
 double candidate[4] = {};
 for (int i = 0; i < 4; ++i) {
 state[i] = digits % 3;
 digits /= 3;
 if (state[i] == 0) {
 free_index[free_count++] = i;
 } else {
 candidate[i] = state[i] == 1 ? -bound[i] : bound[i];
 }
 }
 if (free_count > 0) {
 double matrix[4][4] = {};
 double rhs[4] = {};
 double solution[4] = {};
 for (int row = 0; row < free_count; ++row) {
 const int i = free_index[row];
 rhs[row] = slip[i];
 for (int j = 0; j < 4; ++j) {
 if (state[j] != 0) rhs[row] -= w[i][j] * candidate[j];
 }
 for (int col = 0; col < free_count; ++col) {
 matrix[row][col] = w[i][free_index[col]];
 }
 }
 if (!solve_small_system(matrix, rhs, free_count, solution)) continue;
 bool feasible = true;
 for (int row = 0; row < free_count; ++row) {
 const int i = free_index[row];
 const double tolerance = std::numeric_limits<double>::epsilon() *
 (1.0 + bound[i]) * kFeasibilitySafetyUlps;
 if (solution[row] < -bound[i] - tolerance ||
 solution[row] > bound[i] + tolerance) {
 feasible = false;
 break;
 }
 candidate[i] = std::clamp(solution[row], -bound[i], bound[i]);
 }
 if (!feasible) continue;
 }
 double cost = 0.0;
 for (int i = 0; i < 4; ++i) {
 cost -= slip[i] * candidate[i];
 for (int j = 0; j < 4; ++j) {
 cost += 0.5 * candidate[i] * w[i][j] * candidate[j];
 }
 }
 if (std::isfinite(cost) && cost < best_cost) {
 for (int i = 0; i < 4; ++i) best[i] = candidate[i];
 best_cost = cost;
 }
 }
 return best;
}

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
 const std::array<double, 4>& arm_duty,
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
 const bool is_mecanum = (drive_count_ == 4 && steer_count_ == 0);
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
 auto contact = resolve_wheel_contact(
 is_mecanum ? mecanum_traction_mu_base_ : mu_tire_base_,
 motor.rolling_resist, zones, materials, environment,
 sim_time_s, wx, wy);
 if (is_mecanum) {
 mecanum_traction_mu_[d] = contact.tire_mu;
 // MuJoCo's smooth support cylinder owns only the normal solve. The
 // roller-direction impulse below owns all tangential traction.
 contact.tire_mu = 0.0;
 }
 apply_contact_material(tire_geom_[d], contact);
 if (is_mecanum && tire_geom_[d] >= 0) {
 // The smooth cylinder is only a normal-load carrier. Cylinder torsional
 // and rolling friction describe an ordinary tire and fight the diagonal
 // roller constraint, especially as support transfers between wheels.
 model->geom_friction[tire_geom_[d] * 3 + 1] = 0.0;
 model->geom_friction[tire_geom_[d] * 3 + 2] = 0.0;
 }
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
  const double grade = std::isfinite(incline_grade) ? incline_grade : 0.0;
 const InclineGravity gravity = incline_gravity(grade, body_yaw_);
 model->opt.gravity[0] = gravity.x_mps2;
 model->opt.gravity[1] = gravity.y_mps2;
 model->opt.gravity[2] = gravity.z_mps2;
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
 // World calls the backend on the canonical 1 ms grid, so brake lag,
 // contact-material changes, controls, and the constraint solve all observe
 // the same partition regardless of which public harness drives the world.
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
 // Mecanum's smooth cylinders carry normal load while a coupled pre-solve
 // constraint represents the 45-degree rollers. Motor torque remains on
 // the wheel shafts; the constraint applies equal and opposite generalized
 // impulses to the shafts and chassis, so power and slip stay closed.
 const double utilization_threshold = 1.0 - std::sqrt(std::clamp(
 model->opt.tolerance, 1e-12, 1e-2));
 struct ContactWindow {
 bool valid = false;
 double peak_normal = 0.0;
 double peak_utilization = 0.0;
 double saturated_time_s = 0.0;
 double confirmation_time_s = 0.0;
 } contact_window[4];
 auto& mecanum_g = impl_->mecanum_g;
 auto& mecanum_minv_g = impl_->mecanum_minv_g;
 auto& mecanum_motor_rhs = impl_->mecanum_motor_rhs;
 auto& mecanum_motor_dv = impl_->mecanum_motor_dv;
 auto& mecanum_jacp = impl_->mecanum_jacp;
 auto& mecanum_jacr = impl_->mecanum_jacr;
 if (is_mecanum) {
 mecanum_g.resize(static_cast<std::size_t>(4 * model->nv));
 mecanum_minv_g.resize(static_cast<std::size_t>(4 * model->nv));
 mecanum_motor_rhs.resize(static_cast<std::size_t>(model->nv));
 mecanum_motor_dv.resize(static_cast<std::size_t>(model->nv));
 mecanum_jacp.resize(static_cast<std::size_t>(3 * model->nv));
 mecanum_jacr.resize(static_cast<std::size_t>(3 * model->nv));
 }
  for (int s = 0; s < n; ++s) {
    model->opt.timestep = step_dt;
    mj_step1(model, data);
 std::fill(data->qfrc_applied, data->qfrc_applied + model->nv, 0.0);
 if (is_mecanum) {
 std::fill(mecanum_motor_rhs.begin(), mecanum_motor_rhs.end(), 0.0);
 }
    for (int d = 0; d < drive_count_; ++d) {
      const int dof = drive_dof_[d];
      if (dof < 0) continue;
      data->qfrc_applied[dof] = 0.0;
      const double v = data->qvel[dof] * r;
      const double motor_N = dc_motor_force_N(drive_duty[d], v,
                                              motor.stall_force_N,
                                              motor.free_speed_mps);
      data->qfrc_applied[dof] = motor_N * r;
 if (is_mecanum) mecanum_motor_rhs[dof] = motor_N * r;
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
 // Articulated joints use the same explicit normalized-effort contract as
 // native hardware. MuJoCo integrates the torque with gravity, damping and
 // contact constraints; no renderer-side pose or speed is synthesized.
 for (int a = 0; a < 4; ++a) {
 const int dof = arm_joint_dof_[a];
 if (dof < 0 || !motor.arm.enabled) continue;
 data->qfrc_applied[dof] = std::clamp(arm_duty[a], -1.0, 1.0) *
 motor.arm.joint_torque_limit_Nm[a];
 }
 std::array<double, 4> mecanum_impulse{};
 double mecanum_bound[4] = {};
 double mecanum_slip[4] = {};
 if (is_mecanum) {
 // Build G from each wheel's actual contact-point Jacobian. Projection
 // perpendicular to the passive roller axis gives the exact generalized
 // reaction for the wheel, chassis and current contact orientation.
 // Resolve this quantum's normal contact first. Reusing a previous load can
 // inject energy when a rigid four-wheel chassis transfers support between
 // diagonals; this acceleration-only prepass does not advance simulation.
 mj_forwardSkip(model, data, mjSTAGE_VEL, 1);
 constexpr double kMinimumSupportingNormalZ = 0.5;
 double current_load[4] = {};
 double contact_point[4][3] = {};
 double contact_normal[4][3] = {};
 for (int ci = 0; ci < data->ncon; ++ci) {
 const mjContact& contact = data->contact[ci];
 if (std::abs(static_cast<double>(contact.frame[2])) <
 kMinimumSupportingNormalZ) {
 continue;
 }
 for (int d = 0; d < 4; ++d) {
 if (contact.geom1 != tire_geom_[d] &&
 contact.geom2 != tire_geom_[d]) {
 continue;
 }
 mjtNum force[6] = {};
 mj_contactForce(model, data, ci, force);
 const double normal_force =
 std::max(0.0, static_cast<double>(force[0]));
 if (!(normal_force > 0.0)) continue;
 current_load[d] += normal_force;
 double nx = contact.frame[0];
 double ny = contact.frame[1];
 double nz = contact.frame[2];
 if (nz < 0.0) {
 nx = -nx;
 ny = -ny;
 nz = -nz;
 }
 for (int axis = 0; axis < 3; ++axis) {
 contact_point[d][axis] += normal_force * contact.pos[axis];
 }
 contact_normal[d][0] += normal_force * nx;
 contact_normal[d][1] += normal_force * ny;
 contact_normal[d][2] += normal_force * nz;
 }
 }
 std::fill(mecanum_g.begin(), mecanum_g.end(), 0.0);
 double resolved_load_sum = 0.0;
 for (double load : current_load) resolved_load_sum += load;
 const double gravity_magnitude = std::sqrt(
 model->opt.gravity[0] * model->opt.gravity[0] +
 model->opt.gravity[1] * model->opt.gravity[1] +
 model->opt.gravity[2] * model->opt.gravity[2]);
 const double supported_weight = mj_getTotalmass(model) * gravity_magnitude;
 // A compliant contact can report a large one-quantum impact peak while a
 // rigid four-wheel chassis settles. It is valid impact evidence, but using
 // it as sustained drive traction breaks the Coulomb fixed point. Preserve
 // the measured per-wheel load transfer and cap only the aggregate tractive
 // budget at the vehicle's supported weight.
 const double traction_load_scale = resolved_load_sum > supported_weight
 ? supported_weight / resolved_load_sum : 1.0;
 for (int i = 0; i < 4; ++i) {
 mecanum_normal_load_N_[i] = current_load[i] * traction_load_scale;
 if (!(current_load[i] > 0.0)) continue;
 for (int axis = 0; axis < 3; ++axis) {
 contact_point[i][axis] /= current_load[i];
 contact_normal[i][axis] /= current_load[i];
 }
 const double normal_length = std::sqrt(
 contact_normal[i][0] * contact_normal[i][0] +
 contact_normal[i][1] * contact_normal[i][1] +
 contact_normal[i][2] * contact_normal[i][2]);
 if (!(normal_length > std::numeric_limits<double>::epsilon())) continue;
 for (double& component : contact_normal[i]) component /= normal_length;
 const mjtNum* wheel_matrix = data->xmat + 9 * drive_body_[i];
 double axle[3] = {wheel_matrix[1], wheel_matrix[4], wheel_matrix[7]};
 const double axle_normal = axle[0] * contact_normal[i][0] +
 axle[1] * contact_normal[i][1] +
 axle[2] * contact_normal[i][2];
 for (int axis = 0; axis < 3; ++axis) {
 axle[axis] -= axle_normal * contact_normal[i][axis];
 }
 const double axle_length =
 std::sqrt(axle[0] * axle[0] + axle[1] * axle[1] + axle[2] * axle[2]);
 if (!(axle_length > std::numeric_limits<double>::epsilon())) continue;
 for (double& component : axle) component /= axle_length;
 const double forward[3] = {
 axle[1] * contact_normal[i][2] - axle[2] * contact_normal[i][1],
 axle[2] * contact_normal[i][0] - axle[0] * contact_normal[i][2],
 axle[0] * contact_normal[i][1] - axle[1] * contact_normal[i][0]};
 const double handedness = (i == 0 || i == 3) ? 1.0 : -1.0;
 double roller_axis[3] = {
 axle[0] + handedness * forward[0],
 axle[1] + handedness * forward[1],
 axle[2] + handedness * forward[2]};
 const double roller_length = std::sqrt(
 roller_axis[0] * roller_axis[0] +
 roller_axis[1] * roller_axis[1] +
 roller_axis[2] * roller_axis[2]);
 for (double& component : roller_axis) component /= roller_length;
 const double traction[3] = {
 contact_normal[i][1] * roller_axis[2] -
 contact_normal[i][2] * roller_axis[1],
 contact_normal[i][2] * roller_axis[0] -
 contact_normal[i][0] * roller_axis[2],
 contact_normal[i][0] * roller_axis[1] -
 contact_normal[i][1] * roller_axis[0]};
 mj_jac(model, data, mecanum_jacp.data(), mecanum_jacr.data(),
 contact_point[i], drive_body_[i]);
 mjtNum* row = mecanum_g.data() + i * model->nv;
 for (int dof = 0; dof < model->nv; ++dof) {
 row[dof] = traction[0] * mecanum_jacp[dof] +
 traction[1] * mecanum_jacp[model->nv + dof] +
 traction[2] * mecanum_jacp[2 * model->nv + dof];
 }
 }
 mj_solveM(model, data, mecanum_minv_g.data(), mecanum_g.data(), 4);
 mj_solveM(model, data, mecanum_motor_dv.data(),
 mecanum_motor_rhs.data(), 1);
 double w[4][4] = {};
 double slip[4] = {};
 for (int i = 0; i < 4; ++i) {
 const mjtNum* gi = mecanum_g.data() + i * model->nv;
 for (int dof = 0; dof < model->nv; ++dof) {
 slip[i] += gi[dof] *
 (data->qvel[dof] + mecanum_motor_dv[dof] * step_dt);
 }
 for (int j = 0; j < 4; ++j) {
 const mjtNum* minv_gj =
 mecanum_minv_g.data() + j * model->nv;
 for (int dof = 0; dof < model->nv; ++dof) {
 w[i][j] += gi[dof] * minv_gj[dof];
 }
 }
 mecanum_bound[i] = std::max(0.0, mecanum_traction_mu_[i]) *
 std::max(0.0, mecanum_normal_load_N_[i]) * step_dt;
 mecanum_slip[i] = slip[i];
 }
 // Roundoff in M^-1 products can make W microscopically asymmetric.
 for (int i = 0; i < 4; ++i) {
 for (int j = i + 1; j < 4; ++j) {
 const double symmetric = 0.5 * (w[i][j] + w[j][i]);
 w[i][j] = w[j][i] = symmetric;
 }
 }
 mecanum_impulse = solve_box_impulses(w, slip, mecanum_bound);
 for (int i = 0; i < 4; ++i) {
 const mjtNum* gi = mecanum_g.data() + i * model->nv;
 const double force_scale = -mecanum_impulse[i] / step_dt;
 for (int dof = 0; dof < model->nv; ++dof) {
 data->qfrc_applied[dof] += gi[dof] * force_scale;
 }
 }
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
 if (is_mecanum) {
 // Feed the next fixed quantum with the newly resolved support loads.
 // A supporting normal lies within 60 degrees of world up, excluding wall
 // strikes from the roller traction budget without assuming a flat floor.
 constexpr double kMinimumSupportingNormalZ = 0.5;
 double next_load[4] = {};
 for (int ci = 0; ci < data->ncon; ++ci) {
 const mjContact& contact = data->contact[ci];
 if (std::abs(static_cast<double>(contact.frame[2])) <
 kMinimumSupportingNormalZ) {
 continue;
 }
 for (int d = 0; d < 4; ++d) {
 if (contact.geom1 != tire_geom_[d] &&
 contact.geom2 != tire_geom_[d]) {
 continue;
 }
 mjtNum force[6] = {};
 mj_contactForce(model, data, ci, force);
 next_load[d] += std::max(0.0, static_cast<double>(force[0]));
 }
 }
 for (int d = 0; d < 4; ++d) mecanum_normal_load_N_[d] = next_load[d];
 }
 // Integrate only robot-to-authored-static contact forces. Ground,
 // terrain, material zones and support slabs are deliberately absent from
 // static_geom_, so normal wheel loading can never masquerade as a crash.
 // This is presentation truth only and has no route into ROBOT inputs.
 const auto rubber_geom = [&](int geom_id) {
 for (int d = 0; d < 4; ++d) {
 if (tire_geom_[d] == geom_id) return true;
 }
 for (int j = 0; j < 2; ++j) {
 if (idler_geom_[j] == geom_id) return true;
 }
 for (int i = 0; i < support_count_ && i < 4; ++i) {
 if (support_geom_[i] == geom_id ||
 support_shoulder_geom_[i][0] == geom_id ||
 support_shoulder_geom_[i][1] == geom_id) {
 return true;
 }
 }
 return false;
 };
 for (int ci = 0; ci < data->ncon; ++ci) {
 const mjContact& contact = data->contact[ci];
 const auto first_it = std::find(
 static_geom_.begin(), static_geom_.end(), contact.geom1);
 const auto second_it = std::find(
 static_geom_.begin(), static_geom_.end(), contact.geom2);
 const bool first_static = first_it != static_geom_.end();
 const bool second_static = second_it != static_geom_.end();
 if (first_static == second_static) continue;
 const int robot_geom = first_static ? contact.geom2 : contact.geom1;
 const int robot_body = model->geom_bodyid[robot_geom];
 if (robot_body <= 0) continue;
 const std::size_t static_index = static_cast<std::size_t>(std::distance(
 static_geom_.begin(), first_static ? first_it : second_it));
 const ImpactMaterial static_material =
 static_index < static_impact_material_.size()
 ? static_impact_material_[static_index] : ImpactMaterial::kUnknown;
 const ImpactMaterial robot_material = rubber_geom(robot_geom)
 ? ImpactMaterial::kRubber : ImpactMaterial::kMetal;
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
 if (robot_material == ImpactMaterial::kMetal &&
 static_material == ImpactMaterial::kMetal) {
 const double spark_accumulated = impact_.spark_normal_impulse_Ns;
 const double spark_total = spark_accumulated + impulse_Ns;
 impact_.spark_x_m =
 (impact_.spark_x_m * spark_accumulated +
 contact.pos[0] * impulse_Ns) / spark_total;
 impact_.spark_y_m =
 (impact_.spark_y_m * spark_accumulated +
 contact.pos[1] * impulse_Ns) / spark_total;
 impact_.spark_z_m =
 (impact_.spark_z_m * spark_accumulated +
 contact.pos[2] * impulse_Ns) / spark_total;
 impact_.spark_normal_impulse_Ns = spark_total;
 impact_.spark_relative_speed_mps = std::max(
 impact_.spark_relative_speed_mps, relative_speed);
 impact_.spark_robot_material = robot_material;
 impact_.spark_static_material = static_material;
 impact_.spark_eligible = true;
 }
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
 if (is_mecanum && contact.solref[0] > 0.0) {
 response_time_s = std::max(
 response_time_s, static_cast<double>(contact.solref[0]));
 }
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
 }
 if (is_mecanum) {
 ContactWindow& out = contact_window[d];
 const double bound = mecanum_bound[d];
 if (bound > std::numeric_limits<double>::min()) {
 const double utilization = std::abs(mecanum_impulse[d]) / bound;
 out.peak_utilization = std::max(out.peak_utilization, utilization);
 saturated_this_substep = utilization >= utilization_threshold;
 } else if (std::abs(mecanum_slip[d]) > model->opt.tolerance) {
 // A loaded zero-grip wheel cannot transmit any requested impulse.
 out.peak_utilization = std::numeric_limits<double>::infinity();
 saturated_this_substep = out.valid;
 }
 }
 if (saturated_this_substep) {
 ContactWindow& out = contact_window[d];
 // A tire can resolve through several contact points in one quantum. Count
 // physical time once per wheel, never once per point.
 out.saturated_time_s += step_dt;
 out.confirmation_time_s = std::max(
 out.confirmation_time_s, response_time_s);
 }
 }
  }
  model->opt.timestep = 0.001;
  for (int i = 0; i < model->nv; ++i) data->qfrc_applied[i] = 0.0;
  mj_forward(model, data);
  read_back();
 // Traction saturation is read from the resolved MuJoCo contact force, or
 // from the coupled roller impulse for Mecanum, and the actual normal load.
 // This captures load
 // transfer, split-mu and loss of contact; declared total mass divided by
 // wheel count cannot. The cone boundary comes from the active solver
 // tolerance and the persistence horizon from contact solref. Accumulating
 // seconds instead of outer-call counts keeps the label invariant when a
 // sensor cadence changes how completed physics ticks are sampled.
 for (int d = 0; d < drive_count_ && d < 4; ++d) {
 if (is_mecanum) {
 drive_contact_friction_utilization_[d] =
 contact_window[d].peak_utilization;
 }
 const double saturated_time_s = std::clamp(
 contact_window[d].saturated_time_s, 0.0, dt_s);
 if (saturated_time_s > 0.0) {
 slip_confirmation_time_s_[d] = std::max(
 slip_confirmation_time_s_[d], contact_window[d].confirmation_time_s);
 }
 const double horizon_s = slip_confirmation_time_s_[d];
 if (horizon_s > 0.0) {
 // Contact points naturally appear and disappear while a tire rolls. A
 // gap shorter than the contact response carries no evidence that traction
 // recovered, while a resolved unsaturated contact does. A wheel that stays
 // airborne longer than the same physical horizon also drains the label.
 double quiet_time_s = 0.0;
 if (saturated_time_s > 0.0) {
 slip_unobserved_time_s_[d] = 0.0;
 } else if (contact_window[d].valid) {
 slip_unobserved_time_s_[d] = 0.0;
 quiet_time_s = dt_s;
 } else {
 slip_unobserved_time_s_[d] += dt_s;
 if (slip_unobserved_time_s_[d] >= horizon_s) quiet_time_s = dt_s;
 }
 slip_saturated_time_s_[d] = std::clamp(
 slip_saturated_time_s_[d] + saturated_time_s - quiet_time_s,
 0.0, horizon_s);
 const double tolerance =
 std::numeric_limits<double>::epsilon() * dt_s;
 if (!drive_slipping_[static_cast<std::size_t>(d)] &&
 slip_saturated_time_s_[d] + tolerance >= horizon_s) {
 drive_slipping_[static_cast<std::size_t>(d)] = true;
 } else if (drive_slipping_[static_cast<std::size_t>(d)] &&
 slip_saturated_time_s_[d] <= tolerance) {
 drive_slipping_[static_cast<std::size_t>(d)] = false;
 }
 if (slip_saturated_time_s_[d] <= tolerance) {
 slip_saturated_time_s_[d] = 0.0;
 slip_confirmation_time_s_[d] = 0.0;
 slip_unobserved_time_s_[d] = 0.0;
 }
 } else {
 drive_slipping_[static_cast<std::size_t>(d)] = false;
 slip_unobserved_time_s_[d] = 0.0;
 }
 }
}

}  // namespace lost_signal::sim::world

#endif  // LOST_SIGNAL_WITH_MUJOCO
