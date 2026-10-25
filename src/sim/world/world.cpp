#include "sim/world/world.hpp"

#include <algorithm>
#include <cmath>

namespace lost_signal::sim::world {

World::World() = default;

void World::set_motor_params(const MotorParams& params) {
  motor_ = params;
  brake_request_ = {};
}

void World::set_brake_request(int wheel, double normalized) {
  if (wheel < 0 || wheel >= 4) return;
  brake_request_[wheel] = std::clamp(normalized, 0.0, 1.0);
}

void World::set_incline(double forward_grade) {
  incline_grade_ = std::isfinite(forward_grade) ? forward_grade : 0.0;
}

void World::apply(const robot::iface::ActuatorCommand& cmd) {
  truth_.drive_count = robot::iface::actuator_drive_count(cmd.mode);
  truth_.steer_count = robot::iface::actuator_steer_count(cmd.mode);
  for (int i = 0; i < 4; ++i) {
    truth_.drive_duty[i] =
        i < truth_.drive_count ? std::clamp(cmd.drive[i], -1.0, 1.0) : 0.0;
    truth_.steer_duty[i] =
        i < truth_.steer_count ? std::clamp(cmd.steer[i], -1.0, 1.0) : 0.0;
    // Mode switch retires channels: stale slip flags must not linger.
    if (i >= truth_.drive_count) truth_.drive_slipping[i] = false;
  }
}

void World::step_engine(double dt_s) {
  mujoco_.sync(motor_, static_boxes_, materials_, material_zones_,
               terrain_patches_, support_slabs_, geometry_version_,
               truth_.drive_count, truth_.steer_count);
  if (!mujoco_.valid() || !(dt_s > 0.0)) return;
  mujoco_.step(dt_s, motor_, truth_.drive_duty, truth_.steer_duty,
               brake_request_, incline_grade_, material_zones_, materials_,
               environment_, sim_time_s_);
  truth_.body_x_m = mujoco_.body_x();
  truth_.body_y_m = mujoco_.body_y();
  truth_.body_yaw_rad = mujoco_.body_yaw();
  truth_.forward_mps = mujoco_.forward_mps();
  truth_.lateral_mps = mujoco_.lateral_mps();
  truth_.yaw_rate_radps = mujoco_.yaw_rate();
  truth_.body_z_m = mujoco_.body_z();
  truth_.body_roll_rad = mujoco_.body_roll();
  truth_.body_pitch_rad = mujoco_.body_pitch();
  for (int i = 0; i < truth_.drive_count && i < 4; ++i) {
    truth_.drive_angle_rad[i] = mujoco_.drive_angle(i);
    truth_.drive_radps[i] = mujoco_.drive_radps(i);
    truth_.drive_slipping[i] = mujoco_.drive_slipping(i);
  }
  for (int i = 0; i < truth_.steer_count && i < 4; ++i) {
    truth_.steer_rad[i] = mujoco_.steer_angle(i);
    truth_.steer_vel_radps[i] = mujoco_.steer_vel(i);
  }
}

void World::step(double dt_s) {
  truth_.step += 1;
  if (dt_s > 0.0 && std::isfinite(dt_s)) sim_time_s_ += dt_s;
  step_engine(dt_s);
 // Actors stay kinematic (no engine bodies for them); the judge reads
 // the same truth. An explicitly yielding actor runs its own continuous
 // collision predictor against the physical robot state before integration;
 // this changes the simulated environment, never a ROBOT decision input.
 const double c = std::cos(truth_.body_yaw_rad);
 const double s = std::sin(truth_.body_yaw_rad);
 const double robot_vx = truth_.forward_mps * c - truth_.lateral_mps * s;
 const double robot_vy = truth_.forward_mps * s + truth_.lateral_mps * c;
 const double robot_radius = std::hypot(
 std::max(0.0, motor_.body_half_length_m),
 std::max(0.0, motor_.body_half_width_m));
  for (auto& a : actors_) {
    step_actor(a, a.behavior, dt_s);
 apply_actor_robot_yield(a, a.behavior,
 truth_.body_x_m, truth_.body_y_m, robot_vx, robot_vy,
 robot_radius, dt_s);
    a.x += a.vx_mps * dt_s;
    a.y += a.vy_mps * dt_s;
  }
}

}  // namespace lost_signal::sim::world
