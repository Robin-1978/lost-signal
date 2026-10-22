#pragma once

#include <array>

#include "robot/control/feedforward.hpp"
#include "robot/control/pid.hpp"

namespace lost_signal::robot::control {

// Closed-loop Ackermann controller: body (v, steer-angle) in, per-actuator
// normalized duties out. Rear drives close their speed loops (PID + model
// feedforward, same primitives as the differential controller); front
// steers close a position->rate cascade against measured joint feedback.
// Without valid feedback the controller holds zero: loops never close on
// stale joints.
struct AckermannControllerConfig {
  double wheel_base_m = 0.62;
  double track_width_m = 0.58;
  double wheel_radius_m = 0.1;
  double max_wheel_speed_mps = 2.4;
  double max_steer_rad = 0.55;
  double max_steer_rate_radps = 2.0;
  double duty_limit = 1.0;
  double integral_limit = 2.0;
  double lead_time_s = 0.15;
  double kp = 0.6;
  double ki = 2.0;
  double kd = 0.0;
  // Steering cascade: position error -> rate demand -> duty.
  double steer_kp = 2.5;  // rate demand per rad of angle error
  FeedforwardParams ff{};
};

struct AckermannFeedback {
  double rear_left_radps = 0.0;
  double rear_right_radps = 0.0;
  double front_left_steer_rad = 0.0;
  double front_right_steer_rad = 0.0;
  double front_left_steer_vel_radps = 0.0;
  double front_right_steer_vel_radps = 0.0;
  bool valid = false;
};

struct AckermannCommand {
  double rear_left_duty = 0.0;
  double rear_right_duty = 0.0;
  double front_left_steer_duty = 0.0;
  double front_right_steer_duty = 0.0;
};

class AckermannController {
 public:
  explicit AckermannController(
      AckermannControllerConfig config = AckermannControllerConfig{});

  void reset();
  AckermannCommand update(double target_v_mps, double target_steer_rad,
                          const AckermannFeedback& feedback, double dt_s);

 private:
  AckermannControllerConfig config_;
  PID rear_left_pid_;
  PID rear_right_pid_;
};

}  // namespace lost_signal::robot::control
