#include "sim/world/mujoco_backend.hpp"

#ifdef LOST_SIGNAL_WITH_MUJOCO

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>

#include <mujoco/mujoco.h>

#include "sim/world/motor.hpp"
#include "sim/world/service_brake.hpp"
#include "sim/world/world.hpp"

namespace lost_signal::sim::world {

namespace {

// M1 contact constants (engine-native). Sliding grip ~0.9 with light
// rolling loss; DriveProfile-fed friction arrives in M2. Rolling resistance
// beyond this stays in the analytic R(v) form only if parity demands it.
constexpr double kWheelSlidingFriction = 0.9;
constexpr double kWheelTorsionalFriction = 0.01;
constexpr double kWheelRollingFriction = 0.002;
constexpr double kFloorSlidingFriction = 1.0;

std::string fmt_double(double v) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.17g", v);
  return buf;
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

void MujocoBackend::sync(const MotorParams& motor,
                         const std::vector<StaticBox>& boxes,
                         std::uint64_t geometry_version, int drive_count,
                         int steer_count) {
  // M1: differential only. Anything else is not approximated.
  if (drive_count != 2 || steer_count != 0) {
    drive_count_ = 0;
    ready_ = false;
    return;
  }
  std::ostringstream sig;
  sig << motor.free_speed_mps << "|" << motor.stall_force_N << "|"
      << motor.vehicle_mass_kg << "|" << motor.wheel_radius_m << "|"
      << motor.track_width_m << "|" << motor.wheel_base_m << "|"
      << motor.brake_max_torque_Nm << "|" << motor.brake_apply_time_s << "|"
      << motor.brake_release_time_s << "|" << boxes.size() << "|";
  for (const auto& b : boxes) {
    sig << b.x0 << "," << b.y0 << "," << b.x1 << "," << b.y1 << "," << b.height_m
        << ";";
  }
  const std::string signature = sig.str();
  if (ever_built_ && signature == built_signature_ &&
      geometry_version == built_geometry_version_ && valid()) {
    return;
  }
  // Rebuild: drop the old model first (deterministic from-scratch state).
  // Preserve the dynamic state across the rebuild when the model shape
  // matches (motor-only change): qpos/qvel sizes are identical because
  // static boxes carry no DOFs and M1 is always 2 drives.
  std::vector<double> saved_qpos, saved_qvel;
  if (valid()) {
    mjModel* old_model = impl_->model;
    mjData* old_data = impl_->data;
    saved_qpos.assign(old_data->qpos, old_data->qpos + old_model->nq);
    saved_qvel.assign(old_data->qvel, old_data->qvel + old_model->nv);
  }
  impl_.reset(new Impl());
  ready_ = false;
  drive_count_ = 0;
  drive_dof_[0] = drive_dof_[1] = drive_dof_[2] = drive_dof_[3] = -1;

  const double r = std::max(1e-3, motor.wheel_radius_m);
  const double track = std::max(0.1, motor.track_width_m);
  const double base_len = std::max(0.1, motor.wheel_base_m);
  const double base_mass =
      std::max(0.5, motor.vehicle_mass_kg - 1.0);  // ~1 kg in the wheels
  const double deck_half_h = 0.05;

  std::ostringstream xml;
  xml << "<mujoco>\n"
      << "  <option timestep=\"0.001\"/>\n"
      << "  <worldbody>\n"
      << "    <geom name=\"floor\" type=\"plane\" size=\"20 20 0.1\" friction=\""
      << kFloorSlidingFriction << " 0.01 0.001\"/>\n"
      << "    <body name=\"base\" pos=\"0 0 " << fmt_double(r) << "\">\n"
      << "      <freejoint/>\n"
      << "      <geom name=\"deck\" type=\"box\" size=\"" << fmt_double(track / 2)
      << " " << fmt_double(base_len / 2) << " " << fmt_double(deck_half_h)
      << "\" mass=\"" << fmt_double(base_mass) << "\"/>\n";
  for (int i = 0; i < 2; ++i) {
    const double y = (i == 0 ? track / 2 : -track / 2);
    xml << "      <body name=\"wheel" << i << "\" pos=\"0 " << fmt_double(y)
        << " 0\">\n"
        << "        <joint name=\"drive" << i
        << "\" type=\"hinge\" axis=\"0 1 0\" armature=\""
        << fmt_double(std::max(1e-6, motor.wheel_equiv_mass_kg * r * r))
        << "\"/>\n"
        << "        <geom name=\"tire" << i << "\" type=\"cylinder\" size=\""
        << fmt_double(r) << " 0.04\" euler=\"90 0 0\" density=\"400\""
        << " friction=\"" << kWheelSlidingFriction << " "
        << kWheelTorsionalFriction << " " << kWheelRollingFriction << "\"/>\n"
        << "      </body>\n";
  }
  // Ball casters fore/aft, like the real differential base: unactuated free
  // balls that roll instead of gripping, so motor reaction torque settles
  // into weight transfer (milliradians of rock) instead of slamming the
  // deck edge onto the floor. Without them the free base pitches ~11 deg.
  const double caster_r = 0.03;
  const double caster_x = base_len / 2 * 0.9;
  const double caster_z = -(r - caster_r);
  const char* caster_names[2] = {"caster_f", "caster_r"};
  for (int i = 0; i < 2; ++i) {
    const double cx = (i == 0 ? caster_x : -caster_x);
    xml << "      <body name=\"" << caster_names[i] << "\" pos=\""
        << fmt_double(cx) << " 0 " << fmt_double(caster_z) << "\">\n"
        << "        <joint name=\"casterj" << i << "\" type=\"ball\"/>\n"
        << "        <geom name=\"ball" << i << "\" type=\"sphere\" size=\""
        << fmt_double(caster_r) << "\" mass=\"0.1\""
        << " friction=\"0.05 0.005 0.0005\"/>\n"
        << "      </body>\n";
  }
  xml << "    </body>\n";
  // Surveyed static geometry: fixed worldbody boxes (no joints).
  int bi = 0;
  for (const auto& b : boxes) {
    const double hx = std::max(0.01, (b.x1 - b.x0) / 2);
    const double hy = std::max(0.01, (b.y1 - b.y0) / 2);
    const double hh = std::max(0.05, (b.height_m > 0.0 ? b.height_m : 0.5) / 2);
    xml << "    <geom name=\"static" << (bi++) << "\" type=\"box\" size=\""
        << fmt_double(hx) << " " << fmt_double(hy) << " " << fmt_double(hh)
        << "\" pos=\"" << fmt_double((b.x0 + b.x1) / 2) << " "
        << fmt_double((b.y0 + b.y1) / 2) << " " << fmt_double(hh)
        << "\" friction=\"1.0 0.01 0.001\"/>\n";
  }
  xml << "  </worldbody>\n</mujoco>\n";

  const std::string path =
      (std::filesystem::temp_directory_path() / "mujoco_backend.xml").string();
  {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    out << xml.str();
  }
  char error[1024] = "";
  mjModel* model = mj_loadXML(path.c_str(), nullptr, error, sizeof(error));
  std::filesystem::remove(path);
  if (model == nullptr) {
    std::fprintf(stderr, "mujoco_backend: mj_loadXML failed: %s\n", error);
    return;  // valid() stays false: never step a broken model
  }
  mjData* data = mj_makeData(model);
  if (data == nullptr) {
    mj_deleteModel(model);
    return;
  }
  impl_->model = model;
  impl_->data = data;
  // Brake lag shapes from the motor contract (same mapping as analytic).
  const ServiceBrakeParams bp = brake_params_from_motor(motor);
  for (auto& b : impl_->brakes) b = ServiceBrakePlant{bp};
  for (int i = 0; i < 4; ++i) {
    char name[16];
    std::snprintf(name, sizeof(name), "drive%d", i);
    const int jid = mj_name2id(model, mjOBJ_JOINT, name);
    drive_dof_[i] = (jid >= 0 && i < 2) ? model->jnt_dofadr[jid] : -1;
  }
  drive_count_ = 2;
  ready_ = true;
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
  // Fresh mjData starts at the XML initial pose with zero velocity.
  read_back();
}

void MujocoBackend::step(double dt_s, const MotorParams& motor,
                         const std::array<double, 4>& drive_duty,
                         const std::array<double, 4>& brake_request,
                         double incline_grade) {
  if (!valid() || drive_count_ != 2 || !(dt_s > 0.0)) return;
  mjModel* model = impl_->model;
  mjData* data = impl_->data;
  const double r = std::max(1e-6, motor.wheel_radius_m);
  // Grade as gravity tilt around the current heading (same body-forward
  // semantics as the analytic plant: uphill forward pulls backward).
  const double g = 9.81;
  const double grade = std::isfinite(incline_grade) ? incline_grade : 0.0;
  model->opt.gravity[0] = -g * grade * std::cos(body_yaw_);
  model->opt.gravity[1] = -g * grade * std::sin(body_yaw_);
  model->opt.gravity[2] = -g;
  // Main-stack duty path, verbatim: duty-implied motor force only (the
  // resistance/traction-limit subtraction lives on main's PID path, not
  // the duty path our actuator commands take). Rolling losses come from
  // the engine contact parameters, not from an analytic R(v).
  //
  // Split-step contract, verbatim from main's physics step: mj_step1
  // computes position/velocity-dependent quantities first, then the
  // freshly resolved controls and brake forces are accepted before the
  // acceleration/constraint solve in mj_step2. qfrc_applied is restored
  // every substep and zeroed at the end, exactly like main.
  const double h = 0.001;
  int n = static_cast<int>(std::round(dt_s / h));
  n = std::max(1, std::min(n, 1000));
  const double step_dt = dt_s / n;
  // Brake lag state advances at the outer 20 ms cadence (same as main's
  // per-tick plant step); capacity resolves per substep below.
  double brake_capacity_Nm[2] = {0.0, 0.0};
  for (int i = 0; i < 2; ++i) {
    brake_capacity_Nm[i] =
        impl_->brakes[static_cast<std::size_t>(i)].step(brake_request[i],
                                                        dt_s);
  }
  // Low-speed smoothing for the brake fade (matches the ServiceBrakePlant
  // default; M2 feeds it from the drive profile like main's content data).
  constexpr double kBrakeLowSpeedSmoothingRadps = 0.6;
  for (int s = 0; s < n; ++s) {
    model->opt.timestep = step_dt;
    mj_step1(model, data);
    for (int i = 0; i < 2; ++i) {
      const int dof = drive_dof_[i];
      if (dof < 0) continue;
      // Recomputed every substep from substep velocity (qfrc_applied does
      // not survive the solve); zero the slot first.
      data->qfrc_applied[dof] = 0.0;
      const double v = data->qvel[dof] * r;
      const double motor_N = dc_motor_force_N(drive_duty[i], v,
                                              motor.stall_force_N,
                                              motor.free_speed_mps);
      data->qfrc_applied[dof] = motor_N * r;
      // Service brake, verbatim main: passive capacity against shaft
      // velocity with low-speed smoothing (dissipative, fade to zero at
      // rest rather than an exact clamp).
      if (brake_capacity_Nm[i] > 0.0) {
        const double direction = std::clamp(
            data->qvel[dof] / kBrakeLowSpeedSmoothingRadps, -1.0, 1.0);
        data->qfrc_applied[dof] -= direction * brake_capacity_Nm[i];
      }
    }
    mj_step2(model, data);
  }
  model->opt.timestep = 0.001;
  for (int i = 0; i < model->nv; ++i) data->qfrc_applied[i] = 0.0;
  mj_forward(model, data);
  read_back();
}

void MujocoBackend::read_back() {
  if (!valid()) return;
  mjModel* model = impl_->model;
  mjData* data = impl_->data;
  (void)model;
  body_x_ = data->qpos[0];
  body_y_ = data->qpos[1];
  body_z_ = data->qpos[2];
  const double qw = data->qpos[3], qx = data->qpos[4], qy = data->qpos[5],
               qz = data->qpos[6];
  body_yaw_ = std::atan2(2.0 * (qw * qz + qx * qy),
                         1.0 - 2.0 * (qy * qy + qz * qz));
  body_roll_ = std::atan2(2.0 * (qw * qx + qy * qz),
                          1.0 - 2.0 * (qx * qx + qy * qy));
  const double sinp =
      std::clamp(2.0 * (qw * qy - qz * qx), -1.0, 1.0);
  body_pitch_ = std::asin(sinp);
  const double vx = data->qvel[0], vy = data->qvel[1];
  forward_mps_ =
      vx * std::cos(body_yaw_) + vy * std::sin(body_yaw_);
  lateral_mps_ =
      -vx * std::sin(body_yaw_) + vy * std::cos(body_yaw_);
  yaw_rate_ = data->qvel[5];
  for (int i = 0; i < 2; ++i) {
    const int dof = drive_dof_[i];
    if (dof < 0) continue;
    // Hinge DOF address == joint qpos address for these joints.
    const int jid = mj_name2id(model, mjOBJ_JOINT,
                               i == 0 ? "drive0" : "drive1");
    const int qadr = (jid >= 0) ? model->jnt_qposadr[jid] : -1;
    if (qadr >= 0) drive_angle_[i] = data->qpos[qadr];
    drive_radps_[i] = data->qvel[dof];
  }
}

}  // namespace lost_signal::sim::world

#endif  // LOST_SIGNAL_WITH_MUJOCO
