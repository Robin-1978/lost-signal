#include "sim/world/world.hpp"

#include <algorithm>
#include <cmath>

#include "robot/platform/holonomic_kinematics.hpp"

namespace lost_signal::sim::world {

void World::apply(const robot::iface::ActuatorCommand& cmd) {
  last_mode_ = cmd.mode;
  truth_.drive_count = robot::iface::actuator_drive_count(cmd.mode);
  truth_.steer_count = robot::iface::actuator_steer_count(cmd.mode);
  for (int i = 0; i < 4; ++i) {
    truth_.drive_duty[i] =
        i < truth_.drive_count ? std::clamp(cmd.drive[i], -1.0, 1.0) : 0.0;
    truth_.steer_duty[i] =
        i < truth_.steer_count ? std::clamp(cmd.steer[i], -1.0, 1.0) : 0.0;
  }
}

void World::step(double dt_s) {
  truth_.step += 1;
  // First-order motor lag toward the duty-implied wheel speeds.
  const double free_radps =
      motor_.free_speed_mps / std::max(1e-6, motor_.wheel_radius_m);
  const double alpha =
      dt_s <= 0.0 ? 1.0
                  : 1.0 - std::exp(-dt_s / std::max(1e-6, motor_.time_constant_s));
  for (int i = 0; i < truth_.drive_count; ++i) {
    truth_.drive_radps[i] +=
        (truth_.drive_duty[i] * free_radps - truth_.drive_radps[i]) * alpha;
  }
  // Steering servo: duty implies a rate demand (P-controlled upstream);
  // the rate first-order tracks the demand and the angle integrates it,
  // clamped to the mechanical envelope. Velocity-servo + P control gives
  // zero steady-state error; a position-servo plant would droop.
  const double steer_alpha =
      dt_s <= 0.0 ? 1.0
                  : 1.0 - std::exp(-dt_s / std::max(1e-6, motor_.steer_time_constant_s));
  for (int i = 0; i < truth_.steer_count; ++i) {
    const double rate_target =
        std::clamp(truth_.steer_duty[i], -1.0, 1.0) * motor_.max_steer_rate_radps;
    double vel = truth_.steer_vel_radps[i] + (rate_target - truth_.steer_vel_radps[i]) * steer_alpha;
    double angle = truth_.steer_rad[i] + vel * dt_s;
    if (angle > motor_.max_steer_rad) {
      angle = motor_.max_steer_rad;
      vel = 0.0;  // end-stop: no windup against the mechanical limit
    } else if (angle < -motor_.max_steer_rad) {
      angle = -motor_.max_steer_rad;
      vel = 0.0;
    }
    truth_.steer_rad[i] = angle;
    truth_.steer_vel_radps[i] = vel;
  }
  // Rigid body follows the joints by chassis kinematics.
  const auto wheel_mps = [&](int i) {
    return truth_.drive_radps[i] * motor_.wheel_radius_m;
  };
  switch (last_mode_) {
    case robot::iface::ActuatorMode::kDiffDuty: {
      const double vl = wheel_mps(0), vr = wheel_mps(1);
      truth_.forward_mps = 0.5 * (vl + vr);
      truth_.lateral_mps = 0.0;
      truth_.yaw_rate_radps =
          (vr - vl) / std::max(1e-6, motor_.track_width_m);
      break;
    }
    case robot::iface::ActuatorMode::kAckermannDuty: {
      const double vl = wheel_mps(0), vr = wheel_mps(1);
      truth_.forward_mps = 0.5 * (vl + vr);
      truth_.lateral_mps = 0.0;
      const double delta =
          0.5 * (truth_.steer_rad[0] + truth_.steer_rad[1]);
      truth_.yaw_rate_radps = truth_.forward_mps * std::tan(delta) /
                              std::max(0.05, motor_.wheel_base_m);
      break;
    }
    case robot::iface::ActuatorMode::kSwerveDuty: {
      std::array<double, 4> w = {wheel_mps(0), wheel_mps(1), wheel_mps(2),
                                 wheel_mps(3)};
      std::array<double, 4> s = {truth_.steer_rad[0], truth_.steer_rad[1],
                                 truth_.steer_rad[2], truth_.steer_rad[3]};
      double fwd = 0.0, lat = 0.0, yaw = 0.0;
      robot::platform::swerve_forward(w, s, motor_.wheel_base_m,
                                      motor_.track_width_m, fwd, lat, yaw);
      truth_.forward_mps = fwd;
      truth_.lateral_mps = lat;
      truth_.yaw_rate_radps = yaw;
      break;
    }
    case robot::iface::ActuatorMode::kMecanumDuty: {
      std::array<double, 4> w = {wheel_mps(0), wheel_mps(1), wheel_mps(2),
                                 wheel_mps(3)};
      double fwd = 0.0, lat = 0.0, yaw = 0.0;
      robot::platform::mecanum_forward(w, motor_.wheel_base_m,
                                       motor_.track_width_m, fwd, lat, yaw);
      truth_.forward_mps = fwd;
      truth_.lateral_mps = lat;
      truth_.yaw_rate_radps = yaw;
      break;
    }
  }
  truth_.body_yaw_rad += truth_.yaw_rate_radps * dt_s;
  const double heading_mid =
      truth_.body_yaw_rad;  // yaw already advanced; 20 ms step, no midpoint split
  truth_.body_x_m += (truth_.forward_mps * std::cos(heading_mid) -
                      truth_.lateral_mps * std::sin(heading_mid)) *
                     dt_s;
  truth_.body_y_m += (truth_.forward_mps * std::sin(heading_mid) +
                      truth_.lateral_mps * std::cos(heading_mid)) *
                     dt_s;
  for (auto& a : actors_) {
    a.x += a.vx_mps * dt_s;
    a.y += a.vy_mps * dt_s;
  }
}

}  // namespace lost_signal::sim::world
