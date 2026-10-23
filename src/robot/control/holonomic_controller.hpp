#pragma once

#include <array>

#include "robot/control/feedforward.hpp"
#include "robot/control/pid.hpp"

namespace lost_signal::robot::control {

// Closed-loop holonomic controller: body (v, lateral, yaw) in, per-module
// normalized duties out. Drive wheels close PID speed loops against measured
// joint rates; swerve steers close a position->rate cascade. Mecanum has no
// steers. Without valid feedback the controller holds zero: loops never
// close on stale joints.
enum class HolonomicKind { Swerve, Mecanum };

struct HolonomicControllerConfig {
  HolonomicKind kind = HolonomicKind::Swerve;
  double wheel_base_m = 0.56;
  double track_width_m = 0.56;
  double wheel_radius_m = 0.1;
  double max_wheel_speed_mps = 2.4;
  double duty_limit = 1.0;
  double integral_limit = 2.0;
  double lead_time_s = 0.15;
  // ki is per-second, matching main's tuning (see the wheel loop note:
  // the old fixed 0.1 s step at 20 ms cadence meant 5x over-integration).
  double kp = 0.6;
  double ki = 2.0;
  double kd = 0.0;
  double steer_kp = 2.5;
  double max_steer_rate_radps = 2.0;
  FeedforwardParams ff{};
};

struct HolonomicFeedback {
  std::array<double, 4> drive_radps = {};
  std::array<double, 4> steer_rad = {};
  std::array<double, 4> steer_vel_radps = {};
  bool valid = false;
};

struct HolonomicCommand {
  std::array<double, 4> drive_duty = {};
  std::array<double, 4> steer_duty = {};
};

class HolonomicController {
 public:
  explicit HolonomicController(
      HolonomicControllerConfig config = HolonomicControllerConfig{});

  void reset();
  HolonomicCommand update(double target_v_mps, double target_lateral_mps,
                          double target_yaw_radps,
                          const HolonomicFeedback& feedback, double dt_s);
  HolonomicKind kind() const { return config_.kind; }

 private:
  HolonomicControllerConfig config_;
  std::array<PID, 4> drive_pids_;
};

}  // namespace lost_signal::robot::control
