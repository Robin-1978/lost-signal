#include "sim/world/world.hpp"

#include <algorithm>
#include <cmath>

#include "robot/platform/holonomic_kinematics.hpp"
#include "sim/world/motor.hpp"

namespace lost_signal::sim::world {

namespace {
ServiceBrakeParams brake_params_from(const MotorParams& m) {
  ServiceBrakeParams p;
  p.maximum_torque_Nm = m.brake_max_torque_Nm;
  p.apply_time_s = m.brake_apply_time_s;
  p.release_time_s = m.brake_release_time_s;
  return p;
}
}  // namespace

World::World() : brakes_{ServiceBrakePlant{brake_params_from(motor_)},
                         ServiceBrakePlant{brake_params_from(motor_)},
                         ServiceBrakePlant{brake_params_from(motor_)},
                         ServiceBrakePlant{brake_params_from(motor_)}} {}

void World::set_motor_params(const MotorParams& params) {
  motor_ = params;
  const ServiceBrakeParams bp = brake_params_from(motor_);
  for (auto& b : brakes_) b = ServiceBrakePlant{bp};
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
  // Force-driven wheels: motor + gravity - contact = accel, then the
  // dissipative service brake. Stiction, drag, and brake each clamp to an
  // exact zero instead of jittering across it.
  //
  // Mass model (rigid coupling, consistent with the main-stack vehicle):
  // every patch force acts on the vehicle mass, split evenly over the
  // driven wheels. A lone wheel carries M/n, so symmetric motion integrates
  // exactly ΣF/M while per-wheel force balances (stiction thresholds, drag
  // equilibrium, brake authority) stay untouched. Wheel spin inertia is
  // neglected at body level — same as main; wheel_equiv_mass is retained
  // for the native feedforward contract.
  const double wheel_r = std::max(1e-6, motor_.wheel_radius_m);
  const double n_wheels =
      std::max(1, static_cast<int>(truth_.drive_count));
  const double mass =
      std::max(0.5, motor_.vehicle_mass_kg) / n_wheels;
  if (!(dt_s > 0.0)) {
    for (int i = 0; i < truth_.drive_count; ++i) {
      brakes_[i].reset();
      truth_.drive_radps[i] =
          truth_.drive_duty[i] * motor_.free_speed_mps / wheel_r;
    }
  } else {
    // Force-driven wheels: motor + gravity - contact = accel. Stiction
    // owns the patch below the smoothing speed (exact hold, no creep);
    // rolling resistance is dissipative above it (never reverses motion).
    // Load and gravity use the full vehicle mass (n_wheels is the outer
    // M/n split); total_mass floors at 0 for the load model while the
    // integration mass above floors at 0.5 kg (same as main).
    const double total_mass = std::max(0.0, motor_.vehicle_mass_kg);
    WheelContactParams contact;
    contact.rolling_resist = motor_.rolling_resist;
    contact.load_per_wheel_N = total_mass * kGravityMps2 / n_wheels;
    contact.coulomb_smoothing_mps = motor_.coulomb_smoothing_mps;
    contact.viscous_drag_N_per_mps = motor_.viscous_drag_N_per_mps;
    const double gravity_N =
        -total_mass * kGravityMps2 * incline_grade_ / n_wheels;
    const double capacity_N = contact_static_capacity_N(contact);
    const double smooth_mps =
        std::max(1e-4, motor_.coulomb_smoothing_mps);
    for (int i = 0; i < truth_.drive_count; ++i) {
      const double v = truth_.drive_radps[i] * wheel_r;
      const double motor_f = dc_motor_force_N(truth_.drive_duty[i], v,
                                              motor_.stall_force_N,
                                              motor_.free_speed_mps);
      const double applied_f = motor_f + gravity_N;
      double v_new;
      if (std::abs(v) <= smooth_mps && std::abs(applied_f) <= capacity_N) {
        v_new = 0.0;  // stiction holds (no creep, exact rest)
      } else {
        const double v_free = v + applied_f / mass * dt_s;
        const double resist_N = contact_resistance_N(v, contact);
        if (resist_N == 0.0 || v_free == 0.0) {
          v_new = v_free;
        } else if (((resist_N < 0.0) != (v_free < 0.0)) &&
                   std::abs(resist_N) * dt_s >= mass * std::abs(v_free)) {
          v_new = 0.0;  // drag covers the stop: exact, never reversed
        } else {
          v_new = v_free + resist_N / mass * dt_s;
        }
      }
      // Service brake resolves against the contact-stage velocity and only
      // dissipates: authority covering the stop lands exactly at zero
      // (never reversed), anything less subtracts along the motion.
      const double capacity_Nm = brakes_[i].step(brake_request_[i], dt_s);
      const double brake_N = capacity_Nm / wheel_r;
      if (brake_N > 0.0 && v_new != 0.0) {
        if (brake_N * dt_s >= mass * std::abs(v_new)) {
          v_new = 0.0;
        } else {
          v_new -= std::copysign(brake_N * dt_s / mass, v_new);
        }
      }
      truth_.drive_radps[i] = v_new / wheel_r;
      truth_.drive_angle_rad[i] += truth_.drive_radps[i] * dt_s;
    }
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
    step_actor(a, a.behavior, dt_s);
    a.x += a.vx_mps * dt_s;
    a.y += a.vy_mps * dt_s;
  }
}

}  // namespace lost_signal::sim::world
