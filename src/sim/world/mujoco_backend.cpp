// MuJoCo backend root: object lifetime (Impl + ctors) and the sensor
// read-out side (true-3D raycast primitive + joint/state read-back).
// Plant assembly lives in mujoco_build.cpp, the duty loop in
// mujoco_step.cpp — one backend, three translation units.
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


MujocoBackend::MujocoBackend() : impl_(std::make_unique<Impl>()) {}

MujocoBackend::~MujocoBackend() = default;

MujocoBackend::MujocoBackend(MujocoBackend&&) = default;

MujocoBackend& MujocoBackend::operator=(MujocoBackend&&) = default;

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
  // Ackermann idler spin (free-spinning fronts): same hinge read-back
  // pattern as the drives (DOF address == qpos address here).
  for (int j = 0; j < idler_count_ && j < 2; ++j) {
    const int dof = idler_dof_[j];
    if (dof < 0) continue;
    const int jid = mj_name2id(model, mjOBJ_JOINT,
                               j == 0 ? "idlerj0" : "idlerj1");
    const int qadr = (jid >= 0) ? model->jnt_qposadr[jid] : -1;
    if (qadr >= 0) idler_angle_[j] = data->qpos[qadr];
  }
 for (int i = 0; i < support_count_ && i < 4; ++i) {
 const int sdof = support_swivel_dof_[i];
 if (sdof >= 0) {
 const int sjid = model->dof_jntid[sdof];
 const int sqadr = sjid >= 0 ? model->jnt_qposadr[sjid] : -1;
 if (sqadr >= 0) support_swivel_[i] = data->qpos[sqadr];
 }
 const int rdof = support_roll_dof_[i];
 if (rdof >= 0) {
 const int rjid = model->dof_jntid[rdof];
 const int rqadr = rjid >= 0 ? model->jnt_qposadr[rjid] : -1;
 if (rqadr >= 0) support_roll_[i] = data->qpos[rqadr];
 }
 }
}

}  // namespace lost_signal::sim::world

#endif  // LOST_SIGNAL_WITH_MUJOCO
