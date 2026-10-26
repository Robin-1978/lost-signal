#include <cassert>
#include <cmath>
#include <cstdio>
#include <algorithm>

#include "robot/autonomy/control/stop_latch.hpp"
#include "robot/control/ackermann_controller.hpp"
#include "robot/control/feedforward.hpp"
#include "robot/control/holonomic_controller.hpp"
#include "robot/control/pid.hpp"
#include "robot/control/wheel_controller.hpp"
#include "robot/platform/ackermann_kinematics.hpp"
#include "robot/platform/chassis.hpp"
#include "sim/world/world.hpp"

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

  // Diff chassis to_actuator: stop -> safe zero; motion without feedback -> zero.
  {
    platform::DifferentialChassis chassis(
        0.1, 0.5, {.max_forward_mps = 2.0, .max_yaw_rate_radps = 1.5,
                   .footprint_radius_m = 0.4});
    iface::MotionCommand stop = iface::MotionCommand::SafeStop(100);
    iface::MotionFeedback fb;
    fb.drive_count = 2;
    auto zero = chassis.to_actuator(stop, fb, 100);
    assert(zero.is_safe_zero());
    iface::MotionCommand go;
    go.space = iface::CommandSpace::kYawRate;
    go.stamp_ns = 100;
    go.expiry_ns = 500;
    go.forward_mps = 1.0;
    iface::MotionFeedback nofb;  // invalid: no joints, no drive
    assert(chassis.to_actuator(go, nofb, 100).is_safe_zero());
    fb.drive_radps[0] = 5.0;
    fb.drive_radps[1] = 5.0;
    auto act = chassis.to_actuator(go, fb, 100);
    assert(act.mode == iface::ActuatorMode::kDiffDuty);
    assert(iface::valid_actuator_command(act, 100));
    assert(act.drive[0] > 0.0 && act.drive[1] > 0.0);
  }

  // Ackermann controller: no feedback -> safe zero; closed loop drives
  // forward with correct rear bias and steer polarity.
  {
    control::AckermannController ctl;
    control::AckermannFeedback nofb;
    auto zero = ctl.update(1.0, 0.2, nofb, 0.02);
    assert(zero.rear_left_duty == 0.0 && zero.rear_right_duty == 0.0 &&
           zero.front_left_steer_duty == 0.0 &&
           zero.front_right_steer_duty == 0.0);
    double rl = 0.0, rr = 0.0;  // rear wheel rad/s plant
    double sl = 0.0, sr = 0.0;  // steer angle plant
    control::AckermannCommand cmd;
    for (int i = 0; i < 300; ++i) {
      control::AckermannFeedback fb;
      fb.rear_left_radps = rl;
      fb.rear_right_radps = rr;
      fb.front_left_steer_rad = sl;
      fb.front_right_steer_rad = sr;
      fb.valid = true;
      cmd = ctl.update(1.5, 0.2, fb, 0.02);
      rl += (cmd.rear_left_duty * 30.0 - rl) * (1.0 - std::exp(-0.02 / 0.1));
      rr += (cmd.rear_right_duty * 30.0 - rr) * (1.0 - std::exp(-0.02 / 0.1));
      // Velocity-servo plant mirror: duty -> rate demand -> angle.
      sl = std::clamp(sl + cmd.front_left_steer_duty * 2.0 * 0.02, -0.55, 0.55);
      sr = std::clamp(sr + cmd.front_right_steer_duty * 2.0 * 0.02, -0.55, 0.55);
    }
    assert(0.5 * (rl + rr) * 0.1 > 0.8);  // rear drives converged
    assert(sl > 0.1 && sr > 0.1);        // steers track the left-turn demand
    assert(sl > sr);  // inner (left) wheel steers more on a left turn
    assert(rr > rl);  // outer rear wheel runs faster
  }

  // Ackermann chassis: native steer space + yaw-rate conversion + clamp.
  {
    platform::AckermannChassis chassis(
        0.1, 0.62, 0.58,
        {.max_forward_mps = 2.0,
         .max_yaw_rate_radps = 1.5,
         .footprint_radius_m = 0.4,
         .max_steer_rad = 0.5});
    iface::MotionFeedback fb;
    fb.drive_count = 2;
    fb.steer_count = 2;
    fb.drive_radps[0] = 5.0;
    fb.drive_radps[1] = 5.0;
    // Native steer space passes through, clamped.
    iface::MotionCommand steer;
    steer.space = iface::CommandSpace::kSteerAngle;
    steer.stamp_ns = 100;
    steer.expiry_ns = 500;
    steer.forward_mps = 1.0;
    steer.steer_angle_rad = 2.0;  // beyond the limit
    auto clamped = chassis.clamp_command(steer, 100);
    assert(clamped.steer_angle_rad == 0.5);
    auto act = chassis.to_actuator(clamped, fb, 100);
    assert(act.mode == iface::ActuatorMode::kAckermannDuty);
    assert(iface::valid_actuator_command(act, 100));
    assert(act.drive[0] > 0.0 && act.drive[1] > 0.0);
    assert(act.steer[0] > 0.0 && act.steer[1] > 0.0);
    // Yaw-rate space converts through the bicycle model.
    iface::MotionCommand yaw;
    yaw.space = iface::CommandSpace::kYawRate;
    yaw.stamp_ns = 100;
    yaw.expiry_ns = 500;
    yaw.forward_mps = 1.0;
    yaw.yaw_rate_radps = 0.5;
    auto act2 = chassis.to_actuator(yaw, fb, 100);
    assert(iface::valid_actuator_command(act2, 100));
    assert(act2.steer[0] > 0.0);
 // Reverse preserves body yaw-rate semantics. Since omega = v*tan(delta)/L,
 // negative v and negative omega require positive steer.
 yaw.forward_mps = -1.0;
 yaw.yaw_rate_radps = -0.5;
 auto reverse = chassis.to_actuator(yaw, fb, 120);
 assert(iface::valid_actuator_command(reverse, 120));
 assert(reverse.drive[0] < 0.0 && reverse.drive[1] < 0.0);
 assert(reverse.steer[0] > 0.0 && reverse.steer[1] > 0.0);
    // Wrong space fails closed to mode-correct zero.
    iface::MotionCommand holo;
    holo.space = iface::CommandSpace::kHolonomic;
    holo.stamp_ns = 100;
    holo.expiry_ns = 500;
    holo.forward_mps = 1.0;
    auto zero = chassis.to_actuator(holo, fb, 100);
    assert(zero.mode == iface::ActuatorMode::kAckermannDuty && zero.is_safe_zero());
  }

  // Holonomic swerve controller: lateral+yaw resolve to 4 duties; a pure
  // spin gives symmetric magnitudes with mirrored steers.
  {
    control::HolonomicControllerConfig cfg;
    cfg.kind = control::HolonomicKind::Swerve;
    control::HolonomicController ctl(cfg);
    control::HolonomicFeedback nofb;
    auto zero = ctl.update(1.0, 0.0, 0.0, nofb, 0.02);
    for (double d : zero.drive_duty) assert(d == 0.0);
    control::HolonomicFeedback fb;
    fb.valid = true;  // stationary joints, valid measurement
    auto spin = ctl.update(0.0, 0.0, 1.0, fb, 0.02);
    for (double d : spin.drive_duty) assert(std::isfinite(d));
    // All four modules drive on a pure spin.
    assert(std::abs(spin.drive_duty[0]) > 1e-6);
    // Mecanum: strafe resolves with the expected sign pattern.
    control::HolonomicControllerConfig mc;
    mc.kind = control::HolonomicKind::Mecanum;
    control::HolonomicController mctl(mc);
    auto strafe = mctl.update(0.0, 1.0, 0.0, fb, 0.02);
    assert(strafe.drive_duty[0] < 0.0 && strafe.drive_duty[1] > 0.0 &&
           strafe.drive_duty[2] > 0.0 && strafe.drive_duty[3] < 0.0);
  }

  // Holonomic chassis: factory kinds, clamp, mode-correct zero.
  {
    platform::RobotDescription desc;
    desc.chassis_kind = "swerve";
    desc.limits = {.max_forward_mps = 2.0,
                   .max_yaw_rate_radps = 1.5,
                   .footprint_radius_m = 0.4,
                   .max_steer_rad = 0.55,
                   .max_lateral_mps = 1.0};
    auto swerve = platform::make_chassis(desc);
    assert(swerve && std::string(swerve->chassis_id()) == "swerve");
    desc.chassis_kind = "mecanum";
    auto mecanum = platform::make_chassis(desc);
    assert(mecanum && std::string(mecanum->chassis_id()) == "mecanum");
    desc.chassis_kind = "ackermann";
    auto ack = platform::make_chassis(desc);
    assert(ack && std::string(ack->chassis_id()) == "ackermann");
    desc.chassis_kind = "nope";
    assert(platform::make_chassis(desc) == nullptr);

    iface::MotionFeedback fb;
    fb.drive_count = 4;
    fb.steer_count = 4;
    iface::MotionCommand yaw;
    yaw.space = iface::CommandSpace::kYawRate;
    yaw.stamp_ns = 100;
    yaw.expiry_ns = 500;
    yaw.forward_mps = 1.0;
    auto act = swerve->to_actuator(yaw, fb, 100);
    assert(act.mode == iface::ActuatorMode::kSwerveDuty);
    assert(iface::valid_actuator_command(act, 100));
    auto mact = mecanum->to_actuator(yaw, fb, 100);
    assert(mact.mode == iface::ActuatorMode::kMecanumDuty);
    // Swerve without steer feedback fails closed (steers are safety-relevant).
    iface::MotionFeedback no_steer;
    no_steer.drive_count = 4;
    assert(swerve->to_actuator(yaw, no_steer, 100).is_safe_zero());
  }

  // Sim plants: each mode integrates to the commanded body motion.
  {
    // Ackermann drives a left arc.
    ::lost_signal::sim::world::World w;
    ::lost_signal::sim::world::MotorParams mp;
    mp.wheel_base_m = 0.62;
    mp.track_width_m = 0.58;
    w.set_motor_params(mp);
    iface::ActuatorCommand cmd;
    cmd.mode = iface::ActuatorMode::kAckermannDuty;
    cmd.stamp_ns = 0;
    cmd.expiry_ns = 1'000'000'000;
    cmd.drive[0] = cmd.drive[1] = 0.5;
    cmd.steer[0] = cmd.steer[1] = 0.5;
    w.apply(cmd);
    for (int i = 0; i < 200; ++i) w.step(0.02);
    assert(w.truth().forward_mps > 0.8);
    assert(w.truth().yaw_rate_radps > 0.1);  // positive steer -> left arc
    assert(w.truth().body_yaw_rad > 0.0);
    // Mecanum strafes left with ~zero forward.
    ::lost_signal::sim::world::World m;
    iface::ActuatorCommand mc;
    mc.mode = iface::ActuatorMode::kMecanumDuty;
    mc.stamp_ns = 0;
    mc.expiry_ns = 1'000'000'000;
    mc.drive = {-0.5, 0.5, 0.5, -0.5};
    m.apply(mc);
    for (int i = 0; i < 200; ++i) m.step(0.02);
    assert(m.truth().lateral_mps > 0.5);
 // The resolved four-wheel contact may create small cross-axis leakage;
 // require a lateral-dominant response instead of an absolute speed tied
 // to one motor/contact calibration.
 assert(std::abs(m.truth().forward_mps) <
 0.1 * std::abs(m.truth().lateral_mps));
 assert(std::abs(m.truth().yaw_rate_radps) < 0.01);
    assert(m.truth().body_y_m > 0.3);
  }

  // Full HIL loop for the new chassis: chassis -> plant -> feedback.
  {
    // Ackermann closed loop drives a left arc.
    ::lost_signal::sim::world::World w;
    ::lost_signal::sim::world::MotorParams mp;
    mp.wheel_base_m = 0.62;
    mp.track_width_m = 0.58;
 mp.steer_stall_torque_Nm = 80.0;
    w.set_motor_params(mp);
    platform::AckermannChassis chassis(
        0.1, 0.62, 0.58,
        {.max_forward_mps = 2.0,
         .max_yaw_rate_radps = 1.5,
         .footprint_radius_m = 0.4,
         .max_steer_rad = 0.5});
    iface::MotionCommand yaw;
    yaw.space = iface::CommandSpace::kYawRate;
    yaw.forward_mps = 1.0;
    yaw.yaw_rate_radps = 0.4;
    iface::MotionFeedback fb;  // starts empty: first call holds zero
    for (int i = 0; i < 300; ++i) {
      w.step(0.02);
      fb.drive_count = w.truth().drive_count;
      fb.steer_count = w.truth().steer_count;
      fb.drive_radps = w.truth().drive_radps;
      fb.steer_rad = w.truth().steer_rad;
      fb.steer_vel_radps = w.truth().steer_vel_radps;
 yaw.stamp_ns = static_cast<core::MonotonicNs>(i) * 20'000'000LL;
      yaw.expiry_ns = yaw.stamp_ns + 200'000'000;
      auto act = chassis.to_actuator(chassis.clamp_command(yaw, yaw.stamp_ns),
                                     fb, yaw.stamp_ns);
      w.apply(act);
    }
    assert(w.truth().body_x_m > 1.0);
    const auto expected_steer = platform::ackermann_steering_angles(
        std::atan2(0.4 * 0.62, 1.0), 0.62, 0.58);
    assert(std::abs(w.truth().steer_rad[0] - expected_steer.left_rad) < 0.10);
    assert(std::abs(w.truth().steer_rad[1] - expected_steer.right_rad) < 0.10);
    assert(w.truth().yaw_rate_radps > 0.25);
    assert(w.truth().body_yaw_rad > 1.0);  // sustained left arc
 // Reverse yaw-rate commands use the same body-frame sign convention.
 // This closes the physical contract required by reverse Hybrid-A* edges.
 ::lost_signal::sim::world::World reverse_world;
 reverse_world.set_motor_params(mp);
 platform::AckermannChassis reverse_chassis(
 0.1, 0.62, 0.58,
 {.max_forward_mps = 2.0,
 .max_yaw_rate_radps = 1.5,
 .footprint_radius_m = 0.4,
 .max_steer_rad = 0.5});
 iface::MotionCommand reverse_yaw;
 reverse_yaw.space = iface::CommandSpace::kYawRate;
 reverse_yaw.forward_mps = -1.0;
 reverse_yaw.yaw_rate_radps = -0.4;
 iface::MotionFeedback reverse_fb;
 for (int i = 0; i < 300; ++i) {
 reverse_world.step(0.02);
 reverse_fb.drive_count = reverse_world.truth().drive_count;
 reverse_fb.steer_count = reverse_world.truth().steer_count;
 reverse_fb.drive_radps = reverse_world.truth().drive_radps;
 reverse_fb.steer_rad = reverse_world.truth().steer_rad;
 reverse_fb.steer_vel_radps = reverse_world.truth().steer_vel_radps;
 reverse_yaw.stamp_ns =
 static_cast<core::MonotonicNs>(i) * 20'000'000LL;
 reverse_yaw.expiry_ns = reverse_yaw.stamp_ns + 200'000'000;
 const auto reverse_act = reverse_chassis.to_actuator(
 reverse_chassis.clamp_command(reverse_yaw, reverse_yaw.stamp_ns),
 reverse_fb, reverse_yaw.stamp_ns);
 reverse_world.apply(reverse_act);
 }
 assert(reverse_world.truth().body_x_m < -1.0);
 assert(reverse_world.truth().yaw_rate_radps < -0.25);
 assert(reverse_world.truth().body_yaw_rad < -1.0);
 // Swerve closed loop drives forward on a yaw-rate command.
    ::lost_signal::sim::world::World s;
    platform::HolonomicChassis swerve(
        0.1, 0.56, 0.56,
        {.max_forward_mps = 2.0,
         .max_yaw_rate_radps = 1.5,
         .footprint_radius_m = 0.4,
         .max_steer_rad = 0.55,
         .max_lateral_mps = 1.0},
        [] {
          control::HolonomicControllerConfig hc;
          hc.kind = control::HolonomicKind::Swerve;
          return hc;
        }());
    iface::MotionFeedback sfb;
    for (int i = 0; i < 300; ++i) {
      s.step(0.02);
      sfb.drive_count = s.truth().drive_count;
      sfb.steer_count = s.truth().steer_count;
      sfb.drive_radps = s.truth().drive_radps;
      sfb.steer_rad = s.truth().steer_rad;
      sfb.steer_vel_radps = s.truth().steer_vel_radps;
 yaw.stamp_ns = static_cast<core::MonotonicNs>(i) * 20'000'000LL;
      yaw.expiry_ns = yaw.stamp_ns + 200'000'000;
      auto act = swerve.to_actuator(swerve.clamp_command(yaw, yaw.stamp_ns),
                                    sfb, yaw.stamp_ns);
      s.apply(act);
    }
    assert(s.truth().body_x_m > 0.5);
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
