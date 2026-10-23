#pragma once

#include "robot/control/feedforward.hpp"
#include "robot/control/pid.hpp"
#include "robot/iface/motion.hpp"
#include "robot/platform/differential_kinematics.hpp"

namespace lost_signal::robot::control {

// Closed-loop differential wheel controller: body twist in, per-wheel
// normalized duty out. Feedforward carries the model; two PIDs trim the
// residual against measured wheel speeds. Adapted from the main-stack wheel
// controller onto native chain types (DiffCmd/DiffState kept verbatim).
struct WheelControllerConfig {
  double wheel_radius_m = 0.1;
  double track_width_m = 0.5;
  double max_wheel_speed_mps = 3.0;
  double duty_limit = 1.0;
  double integral_limit = 0.5;
  double lead_time_s = 0.15;
  // Default PID gains suit the sim plant (small direct-drive wheels);
  // native bring-up re-tunes against measured step response. ki is
  // per-second and matches main's tuning: the chassis used to (wrongly)
  // step the loop with a fixed dt of 0.1 s at the 20 ms tick, i.e. 5x
  // over-integration, which is also what widened the autonomy's turn arcs.
  double kp = 0.6;
  double ki = 2.0;
  double kd = 0.0;
  FeedforwardParams ff{};
};

struct WheelFeedback {
  double left_radps = 0.0;
  double right_radps = 0.0;
  bool valid = false;
};

struct WheelCommand {
  double left_duty = 0.0;
  double right_duty = 0.0;
};

class DifferentialWheelController {
 public:
  explicit DifferentialWheelController(
      WheelControllerConfig config = WheelControllerConfig{});

  void reset();
  WheelCommand update(double target_v_mps, double target_w_radps,
                      const WheelFeedback& feedback, double dt_s);

 private:
  WheelControllerConfig config_;
  PID left_pid_;
  PID right_pid_;
};

}  // namespace lost_signal::robot::control
