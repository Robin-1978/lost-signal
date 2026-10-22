#include <cassert>
#include <cmath>
#include <cstdio>

#include "robot/autonomy/control/stop_latch.hpp"
#include "robot/control/feedforward.hpp"
#include "robot/control/pid.hpp"
#include "robot/control/wheel_controller.hpp"
#include "robot/platform/chassis.hpp"

int main() {
  using namespace lost_signal::robot;

  // PID converges a step error (integral action needs a few hundred
  // ticks at this dt; the loop itself is stable and windup-free).
  {
    control::PID pid(1.0, 2.0, 0.0, 1.0, 5.0,
                     control::AntiWindupMode::ConditionalIntegration);
    double y = 0.0;
    for (int i = 0; i < 600; ++i) {
      const double u = pid.update(1.0 - y, 0.01);
      y += (u - y) * 0.1;  // first-order plant
    }
    assert(std::abs(y - 1.0) < 0.05);
  }

  // Feedforward: positive desired accel + stationary wheel -> positive duty.
  {
    control::FeedforwardParams p;
    p.wheel_mass_kg = 3.0;
    p.stall_force_N = 54.0;
    p.free_speed_mps = 3.0;
    auto out = control::wheel_feedforward(2.0, 0.0, p);
    assert(out.pwm_ff > 0.0 && out.pwm_ff <= 1.0);
  }

  // Wheel controller: no feedback -> safe zero (never drives blind).
  {
    control::DifferentialWheelController ctl;
    control::WheelFeedback fb;  // invalid
    auto cmd = ctl.update(1.0, 0.0, fb, 0.1);
    assert(cmd.left_duty == 0.0 && cmd.right_duty == 0.0);
  }

  // Wheel controller: closed loop tracks, correct turning polarity.
  {
    control::DifferentialWheelController ctl;
    double wl = 0.0, wr = 0.0;  // rad/s plant
    for (int i = 0; i < 200; ++i) {
      control::WheelFeedback fb{wl, wr, true};
      auto cmd = ctl.update(2.0, 0.5, fb, 0.02);
      assert(cmd.left_duty >= -1.0 && cmd.left_duty <= 1.0);
      assert(cmd.right_duty >= -1.0 && cmd.right_duty <= 1.0);
      // First-order plant, tau = 0.1 s, free speed 3 m/s, r = 0.1 m.
      wl += (cmd.left_duty * 30.0 - wl) * (1.0 - std::exp(-0.02 / 0.1));
      wr += (cmd.right_duty * 30.0 - wr) * (1.0 - std::exp(-0.02 / 0.1));
    }
    // Left wheel slower than right for a left (+yaw) turn.
    assert(wr > wl);
    const double v = 0.5 * (wl + wr) * 0.1;
    assert(v > 1.0);  // converged near the 2 m/s target
  }

  // Chassis to_actuator: stop -> safe zero; motion without feedback -> zero.
  {
    platform::DifferentialChassis chassis(
        0.1, 0.5, {.max_forward_mps = 2.0, .max_yaw_rate_radps = 1.5,
                   .footprint_radius_m = 0.4});
    iface::MotionCommand stop = iface::MotionCommand::SafeStop(100);
    iface::MotionFeedback fb;
    fb.wheels_valid = true;
    auto zero = chassis.to_actuator(stop, fb, 100);
    assert(zero.is_safe_zero());
    iface::MotionCommand go;
    go.space = iface::CommandSpace::kYawRate;
    go.stamp_ns = 100;
    go.expiry_ns = 500;
    go.forward_mps = 1.0;
    iface::MotionFeedback nofb;  // invalid: no joints, no drive
    assert(chassis.to_actuator(go, nofb, 100).is_safe_zero());
    fb.left_radps = 5.0;
    fb.right_radps = 5.0;
    auto act = chassis.to_actuator(go, fb, 100);
    assert(iface::valid_actuator_command(act, 100));
    assert(act.left > 0.0 && act.right > 0.0);
  }

  // Stop latch: withholds motion until tread settles, then releases.
  {
    autonomy::ControlledStopLatch latch;
    autonomy::ControlledStopLatch::Config cfg;
    assert(!latch.update(false, 0.0, cfg));  // not carrying: no withhold
    assert(latch.update(true, 0.0, cfg));    // carrying, settled x1
    assert(latch.update(true, 0.0, cfg));    // x2
    assert(!latch.update(true, 0.0, cfg));   // x3: released
    assert(latch.update(true, 1.0, cfg));    // moving again: withhold
  }

  std::printf("actuation ok\n");
  return 0;
}
