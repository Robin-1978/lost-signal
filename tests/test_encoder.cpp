#include <cassert>
#include <cmath>
#include <cstdio>

#include "robot/iface/motion.hpp"
#include "robot/platform/chassis.hpp"
#include "sim/providers/sim_motion.hpp"
#include "sim/world/world.hpp"

using namespace lost_signal;

namespace {

void apply_diff_duty(sim::world::World& w, double duty) {
  robot::iface::ActuatorCommand cmd;
  cmd.mode = robot::iface::ActuatorMode::kDiffDuty;
  cmd.stamp_ns = 0;
  cmd.expiry_ns = 1'000'000'000;
  cmd.drive[0] = cmd.drive[1] = duty;
  w.apply(cmd);
}

}  // namespace

int main() {
  using sim::providers::SimMotionEndpoint;

  // Quantum: 2*pi/2048 counts per 20 ms sample.
  constexpr double kQuantumRadps =
      (2.0 * 3.14159265358979323846 / 2048.0) / 0.02;

  // First sample has no history: zero velocity, valid joints.
  {
    sim::world::World w;
    apply_diff_duty(w, 1.0);
    for (int i = 0; i < 10; ++i) w.step(0.02);
    SimMotionEndpoint ep(&w);
    robot::iface::MotionFeedback fb;
    assert(ep.poll_feedback(fb));
    assert(fb.drive_count == 2);
    assert(fb.drive_radps[0] == 0.0 && fb.drive_radps[1] == 0.0);
    assert(fb.sequence == 1);
  }

  // Tracking: quantized velocity follows truth. The per-sample diff
  // lags during hard accel (solver owns the transient), so the tight
  // one-quantum bound applies at cruise; accel needs bounded lag plus
  // proof the channel is a measurement, not passthrough.
  {
    sim::world::World w;
    apply_diff_duty(w, 1.0);
    SimMotionEndpoint ep(&w);
    robot::iface::MotionFeedback fb;
    ep.poll_feedback(fb);
    double max_err = 0.0, cruise_err = 0.0;
    bool saw_quantization = false;
    for (int i = 0; i < 200; ++i) {
      w.step(0.02);
      assert(ep.poll_feedback(fb));
      for (int j = 0; j < 2; ++j) {
        const double err =
            std::abs(fb.drive_radps[j] - w.truth().drive_radps[j]);
        max_err = std::max(max_err, err);
        if (i >= 150) cruise_err = std::max(cruise_err, err);
        if (err > 1e-12) saw_quantization = true;
      }
    }
    assert(max_err < 1.0);  // lag bounded through the launch transient
    assert(cruise_err <= kQuantumRadps);  // tight at cruise
    assert(saw_quantization);  // measurement, not passthrough
    // Rest: joints settle ~still; quantized diff reads ~zero.
    // Poll every step like the 50 Hz loop does: velocity is a per-sample
    // difference, never a long-window average.
    apply_diff_duty(w, 0.0);
    for (int i = 0; i < 4000; ++i) {
      w.step(0.02);
      assert(ep.poll_feedback(fb));
    }
    assert(std::abs(w.truth().forward_mps) < 0.5);
    for (int j = 0; j < 2; ++j) {
      assert(std::abs(fb.drive_radps[j] - w.truth().drive_radps[j]) <=
             kQuantumRadps);  // tracks truth even at a crawl
    }
  }

  // Repeated stamp holds the previous velocity (never divides by zero).
  {
    sim::world::World w;
    apply_diff_duty(w, 1.0);
    for (int i = 0; i < 10; ++i) w.step(0.02);
    SimMotionEndpoint ep(&w);
    robot::iface::MotionFeedback a, b;
    ep.poll_feedback(a);
    w.step(0.02);
    ep.poll_feedback(a);
    const double held = a.drive_radps[0];
    assert(held != 0.0);
    ep.poll_feedback(b);  // same step, same stamp
    assert(b.drive_radps[0] == held);
  }

  // Steer angle is quantized to counts too.
  {
    sim::world::World w;
    robot::iface::ActuatorCommand cmd;
    cmd.mode = robot::iface::ActuatorMode::kAckermannDuty;
    cmd.stamp_ns = 0;
    cmd.expiry_ns = 1'000'000'000;
    cmd.drive[0] = cmd.drive[1] = 0.5;
    cmd.steer[0] = cmd.steer[1] = 0.5;
    w.apply(cmd);
    SimMotionEndpoint ep(&w);
    robot::iface::MotionFeedback fb;
    ep.poll_feedback(fb);
    for (int i = 0; i < 50; ++i) w.step(0.02);
    assert(ep.poll_feedback(fb));
    assert(fb.steer_count == 2);
    for (int j = 0; j < 2; ++j) {
      assert(std::abs(fb.steer_rad[j] - w.truth().steer_rad[j]) <=
             kQuantumRadps * 0.02);
    }
  }

  // Determinism: identical runs produce identical feedback.
  {
    sim::world::World w1, w2;
    apply_diff_duty(w1, 0.7);
    apply_diff_duty(w2, 0.7);
    SimMotionEndpoint e1(&w1), e2(&w2);
    for (int i = 0; i < 100; ++i) {
      w1.step(0.02);
      w2.step(0.02);
      robot::iface::MotionFeedback f1, f2;
      assert(e1.poll_feedback(f1) && e2.poll_feedback(f2));
      assert(f1.drive_radps[0] == f2.drive_radps[0]);
      assert(f1.drive_radps[1] == f2.drive_radps[1]);
    }
  }

  // Closed loop on quantized joints: chassis brakes to an exact stop.
  {
    sim::world::World w;
    apply_diff_duty(w, 1.0);
    for (int i = 0; i < 100; ++i) w.step(0.02);
    robot::platform::DifferentialChassis chassis(
        0.1, 0.5, {.max_forward_mps = 2.0, .max_yaw_rate_radps = 1.5,
                   .footprint_radius_m = 0.4});
    SimMotionEndpoint ep(&w);
    robot::iface::MotionFeedback fb;
    ep.poll_feedback(fb);  // no history yet: velocity 0, loop opens next poll
    bool stopped = false;
    for (int i = 0; i < 300; ++i) {
      w.step(0.02);
      assert(ep.poll_feedback(fb));
      robot::iface::MotionFeedback loop = fb;
      auto act = chassis.to_actuator(robot::iface::MotionCommand::SafeStop(
                                         static_cast<robot::core::MonotonicNs>(i * 20'000'000 + 1)),
                                     loop,
                                     static_cast<robot::core::MonotonicNs>(i * 20'000'000 + 1));
      assert(robot::iface::valid_actuator_command(
          act, static_cast<robot::core::MonotonicNs>(i * 20'000'000 + 1)));
      w.apply(act);
      if (std::abs(w.truth().forward_mps) < 0.01) {
        stopped = true;
        break;
      }
    }
    assert(stopped);  // servo pulls to ~still (exact zero is open-loop)
    // Holds at rest: keep closing the loop like the live tick does. The
    // hard brake-stop leaves the integrator saturated, so the first ~100
    // steps are a settling transient (bounded overshoot, decaying). The
    // pin is the settled state: a live integral servo on a quantized
    // sensor dithers within a couple of counts with no walk-off. Exact
    // zero belongs to the open-loop rest above, not to a live servo.
    robot::iface::MotionFeedback hold_fb = fb;
    for (int i = 300; i < 420; ++i) {
      w.step(0.02);
      assert(ep.poll_feedback(hold_fb));
      auto hold_act = chassis.to_actuator(
          robot::iface::MotionCommand::SafeStop(
              static_cast<robot::core::MonotonicNs>(i * 20'000'000 + 1)),
          hold_fb, static_cast<robot::core::MonotonicNs>(i * 20'000'000 + 1));
      w.apply(hold_act);
    }
    const double x_settled = w.truth().body_x_m;
    double max_fb = 0.0;
    double max_drift = 0.0;
    for (int i = 420; i < 440; ++i) {
      w.step(0.02);
      assert(ep.poll_feedback(hold_fb));
      max_fb = std::max(
          max_fb, std::max(std::abs(hold_fb.drive_radps[0]),
                            std::abs(hold_fb.drive_radps[1])));
      auto hold_act = chassis.to_actuator(
          robot::iface::MotionCommand::SafeStop(
              static_cast<robot::core::MonotonicNs>(i * 20'000'000 + 1)),
          hold_fb, static_cast<robot::core::MonotonicNs>(i * 20'000'000 + 1));
      w.apply(hold_act);
      max_drift =
          std::max(max_drift, std::abs(w.truth().body_x_m - x_settled));
    }
    assert(max_fb <= 2.0 * kQuantumRadps);
    assert(max_drift <= 0.005);
  }

  std::printf("encoder ok\n");
  return 0;
}
