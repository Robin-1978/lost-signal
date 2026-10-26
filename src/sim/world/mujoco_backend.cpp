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
  if (base_qpos_ < 0 || base_dof_ < 0) return;
  body_x_ = data->qpos[base_qpos_ + 0];
  body_y_ = data->qpos[base_qpos_ + 1];
  body_z_ = data->qpos[base_qpos_ + 2];
  const double qw = data->qpos[base_qpos_ + 3];
  const double qx = data->qpos[base_qpos_ + 4];
  const double qy = data->qpos[base_qpos_ + 5];
  const double qz = data->qpos[base_qpos_ + 6];
  body_yaw_ = std::atan2(2.0 * (qw * qz + qx * qy),
                         1.0 - 2.0 * (qy * qy + qz * qz));
  body_roll_ = std::atan2(2.0 * (qw * qx + qy * qz),
                          1.0 - 2.0 * (qx * qx + qy * qy));
  const double sinp = std::clamp(2.0 * (qw * qy - qz * qx), -1.0, 1.0);
  body_pitch_ = std::asin(sinp);
  const double vx = data->qvel[base_dof_ + 0];
  const double vy = data->qvel[base_dof_ + 1];
  forward_mps_ = vx * std::cos(body_yaw_) + vy * std::sin(body_yaw_);
  lateral_mps_ = -vx * std::sin(body_yaw_) + vy * std::cos(body_yaw_);
  yaw_rate_ = data->qvel[base_dof_ + 5];
 const int base_body = mj_name2id(model, mjOBJ_BODY, "base");
 total_mass_kg_ = mj_getTotalmass(model);
 system_com_x_m_ = base_body >= 0
 ? static_cast<double>(data->subtree_com[3 * base_body + 0]) : 0.0;
 system_com_y_m_ = base_body >= 0
 ? static_cast<double>(data->subtree_com[3 * base_body + 1]) : 0.0;
 system_com_z_m_ = base_body >= 0
 ? static_cast<double>(data->subtree_com[3 * base_body + 2]) : 0.0;
 const mjtNum* base_position = base_body >= 0
 ? data->xpos + 3 * base_body : nullptr;
 const mjtNum* base_quaternion = base_body >= 0
 ? data->xquat + 4 * base_body : nullptr;
 const auto rotate_world_to_base = [&](double wx, double wy, double wz,
 double& bx, double& by, double& bz) {
 if (base_position == nullptr || base_quaternion == nullptr) {
 bx = by = bz = 0.0;
 return;
 }
 const double vx = wx - base_position[0];
 const double vy = wy - base_position[1];
 const double vz = wz - base_position[2];
 const double iw = base_quaternion[0];
 const double ix = -base_quaternion[1];
 const double iy = -base_quaternion[2];
 const double iz = -base_quaternion[3];
 const double tx = 2.0 * (iy * vz - iz * vy);
 const double ty = 2.0 * (iz * vx - ix * vz);
 const double tz = 2.0 * (ix * vy - iy * vx);
 bx = vx + iw * tx + (iy * tz - iz * ty);
 by = vy + iw * ty + (iz * tx - ix * tz);
 bz = vz + iw * tz + (ix * ty - iy * tx);
 };
 rotate_world_to_base(system_com_x_m_, system_com_y_m_, system_com_z_m_,
 system_com_relative_x_m_, system_com_relative_y_m_,
 system_com_relative_z_m_);
 const auto read_relative_body_pose = [&](int body_id, LinkPoseViz& out) {
 out = LinkPoseViz{};
 if (base_body < 0 || body_id < 0 || body_id >= model->nbody) return;
 const mjtNum* bp = data->xpos + 3 * base_body;
 const mjtNum* bq = data->xquat + 4 * base_body; // w, x, y, z
 const mjtNum* lp = data->xpos + 3 * body_id;
 const mjtNum* lq = data->xquat + 4 * body_id;
 const double iw = bq[0], ix = -bq[1], iy = -bq[2], iz = -bq[3];
 const double vx = lp[0] - bp[0];
 const double vy = lp[1] - bp[1];
 const double vz = lp[2] - bp[2];
 // Rotate by inverse(base quaternion): v' = v + w*t + u x t,
 // t = 2(u x v). This retains the exact engine body transform while
 // expressing it in the base frame consumed by the viewer.
 const double tx = 2.0 * (iy * vz - iz * vy);
 const double ty = 2.0 * (iz * vx - ix * vz);
 const double tz = 2.0 * (ix * vy - iy * vx);
 out.x_m = vx + iw * tx + (iy * tz - iz * ty);
 out.y_m = vy + iw * ty + (iz * tx - ix * tz);
 out.z_m = vz + iw * tz + (ix * ty - iy * tx);
 // q_rel = inverse(q_base) * q_link.
 out.qw = iw * lq[0] - ix * lq[1] - iy * lq[2] - iz * lq[3];
 out.qx = iw * lq[1] + ix * lq[0] + iy * lq[3] - iz * lq[2];
 out.qy = iw * lq[2] - ix * lq[3] + iy * lq[0] + iz * lq[1];
 out.qz = iw * lq[3] + ix * lq[2] - iy * lq[1] + iz * lq[0];
 const double qn = std::sqrt(out.qw * out.qw + out.qx * out.qx +
 out.qy * out.qy + out.qz * out.qz);
 if (!(qn > 1e-12) || !std::isfinite(qn)) {
 out = LinkPoseViz{};
 return;
 }
 out.qw /= qn;
 out.qx /= qn;
 out.qy /= qn;
 out.qz /= qn;
 out.valid = true;
 };
 const auto read_joint_axis = [&](int joint_id, int dof, JointAxisViz& out) {
 out = JointAxisViz{};
 if (joint_id < 0 || joint_id >= model->njnt || dof < 0 ||
 dof >= model->nv || base_position == nullptr || base_quaternion == nullptr) {
 return;
 }
 const int body_id = model->jnt_bodyid[joint_id];
 if (body_id < 0 || body_id >= model->nbody) return;
 read_relative_body_pose(body_id, out.link_pose);
 if (!out.link_pose.valid) return;
 const mjtNum* anchor = data->xanchor + 3 * joint_id;
 rotate_world_to_base(anchor[0], anchor[1], anchor[2], out.link_pose.x_m,
 out.link_pose.y_m, out.link_pose.z_m);
 const mjtNum* axis = data->xaxis + 3 * joint_id;
 rotate_world_to_base(base_position[0] + axis[0],
 base_position[1] + axis[1], base_position[2] + axis[2],
 out.axis_x, out.axis_y, out.axis_z);
 // rotate_world_to_base() subtracts the base origin, so the endpoint above
 // is a vector rotation when the base origin is added back first.
 const double norm = std::sqrt(out.axis_x * out.axis_x +
 out.axis_y * out.axis_y + out.axis_z * out.axis_z);
 if (!(norm > 1e-12) || !std::isfinite(norm)) {
 out = JointAxisViz{};
 return;
 }
 out.axis_x /= norm;
 out.axis_y /= norm;
 out.axis_z /= norm;
 const int qadr = model->jnt_qposadr[joint_id];
 const int vadr = model->jnt_dofadr[joint_id];
 if (qadr >= 0 && qadr < model->nq) out.angle_rad = data->qpos[qadr];
 if (vadr >= 0 && vadr < model->nv) out.rate_radps = data->qvel[vadr];
 out.valid = true;
 };
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
 read_relative_body_pose(drive_body_[d], drive_pose_[d]);
 read_joint_axis(jid, dof, drive_joint_[d]);
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
 read_joint_axis(jid, dof, steer_joint_[t]);
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
 idler_radps_[j] = data->qvel[dof];
 read_relative_body_pose(idler_body_[j], idler_pose_[j]);
 read_joint_axis(jid, dof, idler_joint_[j]);
  }
 for (int i = 0; i < support_count_ && i < 4; ++i) {
 const int sdof = support_swivel_dof_[i];
 if (sdof >= 0) {
 const int sjid = model->dof_jntid[sdof];
 const int sqadr = sjid >= 0 ? model->jnt_qposadr[sjid] : -1;
 if (sqadr >= 0) support_swivel_[i] = data->qpos[sqadr];
 read_joint_axis(sjid, sdof, support_swivel_joint_[i]);
 }
 const int rdof = support_roll_dof_[i];
 if (rdof >= 0) {
 const int rjid = model->dof_jntid[rdof];
 const int rqadr = rjid >= 0 ? model->jnt_qposadr[rjid] : -1;
 if (rqadr >= 0) support_roll_[i] = data->qpos[rqadr];
 read_joint_axis(rjid, rdof, support_roll_joint_[i]);
 }
 read_relative_body_pose(support_fork_body_[i], support_fork_pose_[i]);
 read_relative_body_pose(support_wheel_body_[i], support_wheel_pose_[i]);
 }
 read_contacts();
}

void MujocoBackend::read_contacts() {
 if (!valid()) return;
 mjModel* model = impl_->model;
 mjData* data = impl_->data;
 contact_viz_count_ = 0;
 for (int ci = 0; ci < data->ncon && contact_viz_count_ <
 static_cast<int>(contact_viz_.size()); ++ci) {
 const mjContact& contact = data->contact[ci];
 mjtNum force[6] = {};
 mj_contactForce(model, data, ci, force);
 ContactViz evidence;
 evidence.valid = true;
 evidence.x_m = contact.pos[0];
 evidence.y_m = contact.pos[1];
 evidence.z_m = contact.pos[2];
 evidence.normal_x = contact.frame[0];
 evidence.normal_y = contact.frame[1];
 evidence.normal_z = contact.frame[2];
 evidence.normal_force_N = std::max(0.0, static_cast<double>(force[0]));
 contact_viz_[contact_viz_count_++] = evidence;
 }
 const auto point_velocity = [&](int body_id, const mjtNum* point,
 double* velocity) {
 velocity[0] = velocity[1] = velocity[2] = 0.0;
 if (body_id <= 0 || body_id >= model->nbody) return;
 mjtNum spatial[6] = {};
 mj_objectVelocity(model, data, mjOBJ_BODY, body_id, spatial, 0);
 const mjtNum* center = data->xpos + 3 * body_id;
 const double rx = point[0] - center[0];
 const double ry = point[1] - center[1];
 const double rz = point[2] - center[2];
 velocity[0] = spatial[3] + spatial[1] * rz - spatial[2] * ry;
 velocity[1] = spatial[4] + spatial[2] * rx - spatial[0] * rz;
 velocity[2] = spatial[5] + spatial[0] * ry - spatial[1] * rx;
 };
 const auto read_contact = [&](int geom_id, int wheel_body_id,
 bool& valid, double& x, double& y, double& z, double& slip_mps,
 double& longitudinal_slip_mps, double& lateral_slip_mps,
 double& motion_x_mps, double& motion_y_mps, double& motion_z_mps,
 double& slip_x_mps, double& slip_y_mps, double& slip_z_mps,
 double& normal_N, double& friction_utilization) {
 valid = false;
 x = y = z = slip_mps = longitudinal_slip_mps =
 lateral_slip_mps = motion_x_mps = motion_y_mps = motion_z_mps =
 slip_x_mps = slip_y_mps = slip_z_mps = normal_N =
 friction_utilization = 0.0;
 if (geom_id < 0 || wheel_body_id <= 0 || wheel_body_id >= model->nbody) {
 return;
 }
 double weight_sum = 0.0;
 for (int ci = 0; ci < data->ncon; ++ci) {
 const mjContact& contact = data->contact[ci];
 if (contact.geom1 != geom_id && contact.geom2 != geom_id) continue;
 const int body1 = model->geom_bodyid[contact.geom1];
 const int body2 = model->geom_bodyid[contact.geom2];
 double velocity1[3], velocity2[3];
 point_velocity(body1, contact.pos, velocity1);
 point_velocity(body2, contact.pos, velocity2);
 const bool tire_is_first = contact.geom1 == geom_id;
 const double rvx = tire_is_first
 ? velocity1[0] - velocity2[0]
 : velocity2[0] - velocity1[0];
 const double rvy = tire_is_first
 ? velocity1[1] - velocity2[1]
 : velocity2[1] - velocity1[1];
 const double rvz = tire_is_first
 ? velocity1[2] - velocity2[2]
 : velocity2[2] - velocity1[2];
 double nx = contact.frame[0];
 double ny = contact.frame[1];
 double nz = contact.frame[2];
 if (nz < 0.0) {
 nx = -nx;
 ny = -ny;
 nz = -nz;
 }
 const double normal_v = rvx * nx + rvy * ny + rvz * nz;
 const double tx = rvx - normal_v * nx;
 const double ty = rvy - normal_v * ny;
 const double tz = rvz - normal_v * nz;
 double module_velocity[3] = {};
 const mjtNum* wheel_center = data->xpos + 3 * wheel_body_id;
 point_velocity(wheel_body_id, wheel_center, module_velocity);
 const int surface_body = tire_is_first ? body2 : body1;
 double surface_velocity[3] = {};
 point_velocity(surface_body, contact.pos, surface_velocity);
 const double module_rvx = module_velocity[0] - surface_velocity[0];
 const double module_rvy = module_velocity[1] - surface_velocity[1];
 const double module_rvz = module_velocity[2] - surface_velocity[2];
 const double module_normal_v =
 module_rvx * nx + module_rvy * ny + module_rvz * nz;
 const double module_tx = module_rvx - module_normal_v * nx;
 const double module_ty = module_rvy - module_normal_v * ny;
 const double module_tz = module_rvz - module_normal_v * nz;
 const mjtNum* wheel_matrix = data->xmat + 9 * wheel_body_id;
 const double axle_x = wheel_matrix[1];
 const double axle_y = wheel_matrix[4];
 const double axle_z = wheel_matrix[7];
 double forward_x = axle_y * nz - axle_z * ny;
 double forward_y = axle_z * nx - axle_x * nz;
 double forward_z = axle_x * ny - axle_y * nx;
 const double forward_norm = std::sqrt(
 forward_x * forward_x + forward_y * forward_y +
 forward_z * forward_z);
 if (forward_norm > 1e-12) {
 forward_x /= forward_norm;
 forward_y /= forward_norm;
 forward_z /= forward_norm;
 }
 const double side_x = ny * forward_z - nz * forward_y;
 const double side_y = nz * forward_x - nx * forward_z;
 const double side_z = nx * forward_y - ny * forward_x;
 mjtNum force[6] = {};
 mj_contactForce(model, data, ci, force);
 const double contact_normal_N =
 std::max(0.0, static_cast<double>(force[0]));
 const double weight = std::max(1e-9, contact_normal_N);
 x += weight * contact.pos[0];
 y += weight * contact.pos[1];
 z += weight * contact.pos[2];
 slip_mps += weight * std::sqrt(tx * tx + ty * ty + tz * tz);
 longitudinal_slip_mps += weight *
 (tx * forward_x + ty * forward_y + tz * forward_z);
 lateral_slip_mps += weight *
 (tx * side_x + ty * side_y + tz * side_z);
 motion_x_mps += weight * module_tx;
 motion_y_mps += weight * module_ty;
 motion_z_mps += weight * module_tz;
 slip_x_mps += weight * tx;
 slip_y_mps += weight * ty;
 slip_z_mps += weight * tz;
 normal_N += contact_normal_N;
 if (contact_normal_N > 1e-9 && contact.dim >= 3) {
 const double mu1 = std::max(1e-9, contact.friction[0]);
 const double mu2 = std::max(1e-9, contact.friction[1]);
 const double scaled1 = std::abs(static_cast<double>(force[1])) / mu1;
 const double scaled2 = std::abs(static_cast<double>(force[2])) / mu2;
 const double demand = model->opt.cone == mjCONE_ELLIPTIC
 ? std::hypot(scaled1, scaled2)
 : scaled1 + scaled2;
 friction_utilization = std::max(
 friction_utilization, demand / contact_normal_N);
 }
 weight_sum += weight;
 }
 if (weight_sum <= 0.0) return;
 x /= weight_sum;
 y /= weight_sum;
 z /= weight_sum;
 slip_mps /= weight_sum;
 longitudinal_slip_mps /= weight_sum;
 lateral_slip_mps /= weight_sum;
 motion_x_mps /= weight_sum;
 motion_y_mps /= weight_sum;
 motion_z_mps /= weight_sum;
 slip_x_mps /= weight_sum;
 slip_y_mps /= weight_sum;
 slip_z_mps /= weight_sum;
 valid = true;
 };
 for (int d = 0; d < 4; ++d) {
 read_contact(tire_geom_[d], drive_body_[d], drive_contact_valid_[d],
 drive_contact_x_[d], drive_contact_y_[d], drive_contact_z_[d],
 drive_contact_slip_mps_[d], drive_contact_longitudinal_slip_mps_[d],
 drive_contact_lateral_slip_mps_[d],
 drive_contact_motion_x_mps_[d], drive_contact_motion_y_mps_[d],
 drive_contact_motion_z_mps_[d], drive_contact_slip_x_mps_[d],
 drive_contact_slip_y_mps_[d], drive_contact_slip_z_mps_[d],
 drive_contact_normal_N_[d],
 drive_contact_friction_utilization_[d]);
 }
 for (int j = 0; j < 2; ++j) {
 read_contact(idler_geom_[j], idler_body_[j], idler_contact_valid_[j],
 idler_contact_x_[j], idler_contact_y_[j], idler_contact_z_[j],
 idler_contact_slip_mps_[j], idler_contact_longitudinal_slip_mps_[j],
 idler_contact_lateral_slip_mps_[j],
 idler_contact_motion_x_mps_[j], idler_contact_motion_y_mps_[j],
 idler_contact_motion_z_mps_[j], idler_contact_slip_x_mps_[j],
 idler_contact_slip_y_mps_[j], idler_contact_slip_z_mps_[j],
 idler_contact_normal_N_[j],
 idler_contact_friction_utilization_[j]);
 }
}

}  // namespace lost_signal::sim::world

#endif  // LOST_SIGNAL_WITH_MUJOCO
