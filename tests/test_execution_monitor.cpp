#include <cassert>
#include <cstdio>

#include "robot/autonomy/diagnostics/execution_monitor.hpp"

int main() {
  using namespace lost_signal::robot;
  using autonomy::EstimatedState;
  using autonomy::diagnostics::ExecutionFault;
  using autonomy::diagnostics::ExecutionMonitor;
  using autonomy::diagnostics::ExecutionMonitorConfig;
  using autonomy::diagnostics::ExecutionMonitorReport;

  constexpr core::MonotonicNs kPeriod = 20'000'000;
  constexpr core::MonotonicNs kMs = 1'000'000;
  const ExecutionMonitorConfig config{
      .velocity_deadband_mps = 0.08,
      .yaw_rate_deadband_radps = 0.08,
      .response_delay_s = 0.10,
      .safety_reaction_time_s = 0.20,
      .brake_deceleration_mps2 = 2.0,
      .confirmation_s = 0.20,
  };

  EstimatedState stationary;
  stationary.valid = true;
  stationary.stamp_ns = 1'000'000'000;

  iface::MotionCommand drive;
  drive.space = iface::CommandSpace::kYawRate;
  drive.stamp_ns = stationary.stamp_ns;
  drive.expiry_ns = stationary.stamp_ns + 500 * kMs;
  drive.forward_mps = 0.5;

  // A responsive drive remains inside the monitor envelope.
  {
    ExecutionMonitor monitor(config);
    EstimatedState moving = stationary;
    moving.forward_mps = 0.5;
    for (int i = 0; i < 30; ++i) {
      const auto now = stationary.stamp_ns + i * kPeriod;
      drive.stamp_ns = now;
      drive.expiry_ns = now + 500 * kMs;
      moving.stamp_ns = now;
      const auto report = monitor.update(drive, moving, now);
      assert(report.active && !report.stop_required);
      assert(report.fault == ExecutionFault::kNone);
    }
  }

  // Sustained no-response is a fault only after the declared actuator delay
  // and safety reaction interval; a single lagging sample cannot stop ROBOT.
  {
    ExecutionMonitor monitor(config);
    EstimatedState no_response = stationary;
    auto report = monitor.update(drive, stationary, stationary.stamp_ns);
    assert(!report.stop_required);
    assert(report.pending_fault == ExecutionFault::kNoResponse);
    for (int i = 1; i <= 24; ++i) {
      const auto now = stationary.stamp_ns + i * kPeriod;
      drive.stamp_ns = now;
      drive.expiry_ns = now + 500 * kMs;
      no_response.stamp_ns = now;
      report = monitor.update(drive, no_response, now);
      assert(!report.stop_required);
    }
    drive.stamp_ns = stationary.stamp_ns + 25 * kPeriod;
    drive.expiry_ns = drive.stamp_ns + 500 * kMs;
    no_response.stamp_ns = drive.stamp_ns;
    report = monitor.update(drive, no_response,
                            stationary.stamp_ns + 25 * kPeriod);
    assert(report.stop_required);
    assert(report.fault == ExecutionFault::kNoResponse);
    // A critical fault latches until explicit recovery, even if feedback
    // later claims motion.
    EstimatedState recovered = stationary;
    recovered.stamp_ns += 26 * kPeriod;
    recovered.forward_mps = 0.5;
    report = monitor.update(drive, recovered, recovered.stamp_ns);
    assert(report.stop_required && report.fault == ExecutionFault::kNoResponse);
    monitor.clear_fault();
    report = monitor.update(drive, recovered, recovered.stamp_ns);
    assert(!report.stop_required);
  }

  // A low-speed command can be met while remaining below the absolute
  // observation band. The band is an uncertainty on both sides of the
  // request; this is measurable response, not actuator failure.
  {
    ExecutionMonitor monitor(config);
    iface::MotionCommand crawl = drive;
    crawl.forward_mps = 0.12;
    EstimatedState partial = stationary;
    partial.forward_mps = 0.07;
    ExecutionMonitorReport report;
    for (int i = 0; i <= 20; ++i) {
      const auto now = stationary.stamp_ns + i * kPeriod;
      crawl.stamp_ns = now;
      crawl.expiry_ns = now + 500 * kMs;
      partial.stamp_ns = now;
      report = monitor.update(crawl, partial, now);
    }
    assert(!report.stop_required);
    assert(report.fault == ExecutionFault::kNone);
  }

  // Once the command is materially above the observation band, a response
  // that still sits inside the band is indistinguishable from no motion and
  // must not keep the monitor disarmed indefinitely.
  {
    ExecutionMonitor monitor(config);
    iface::MotionCommand crawl = drive;
    crawl.forward_mps = 0.30;
    EstimatedState inside_band = stationary;
    inside_band.forward_mps = 0.05;
    ExecutionMonitorReport report;
    for (int i = 0; i <= 26; ++i) {
      const auto now = stationary.stamp_ns + i * kPeriod;
      crawl.stamp_ns = now;
      crawl.expiry_ns = now + 500 * kMs;
      inside_band.stamp_ns = now;
      report = monitor.update(crawl, inside_band, now);
    }
    assert(report.stop_required);
    assert(report.fault == ExecutionFault::kNoResponse);
  }

  // Wrong direction is not a valid response to a forward command.
  {
    ExecutionMonitor monitor(config);
    EstimatedState backwards = stationary;
    backwards.forward_mps = -0.5;
    ExecutionMonitorReport report;
    for (int i = 0; i <= 45; ++i) {
      const auto now = stationary.stamp_ns + i * kPeriod;
      drive.stamp_ns = now;
      drive.expiry_ns = now + 500 * kMs;
      backwards.stamp_ns = now;
      report = monitor.update(drive, backwards, now);
    }
    assert(report.stop_required);
    assert(report.fault == ExecutionFault::kDirectionMismatch);
  }

  // Changing gear is a braking transient. Motion opposite to the new request
  // is allowed to dissipate under the declared brake deceleration before the
  // monitor calls it an actuator direction fault.
  {
    ExecutionMonitor monitor(config);
    iface::MotionCommand forward = drive;
    forward.forward_mps = 0.12;
    EstimatedState reversing = stationary;
    reversing.forward_mps = -0.10;
    ExecutionMonitorReport report;
    for (int i = 0; i <= 22; ++i) {
      const auto now = stationary.stamp_ns + i * kPeriod;
      forward.stamp_ns = now;
      forward.expiry_ns = now + 500 * kMs;
      reversing.stamp_ns = now;
      report = monitor.update(forward, reversing, now);
      assert(!report.stop_required);
    }
  }

  // A slow arc has translation intent even when its forward component is
  // below the arrival band. Yaw rate is then a coupled curvature response,
  // not an independent turn-in-place actuator channel.
  {
    ExecutionMonitor monitor(config);
    iface::MotionCommand arc = drive;
    arc.forward_mps = 0.07;
    arc.yaw_rate_radps = 0.34;
    EstimatedState turning = stationary;
    turning.forward_mps = 0.038;
    turning.yaw_rate_radps = 0.06;
    ExecutionMonitorReport report;
    for (int i = 0; i <= 20; ++i) {
      const auto now = stationary.stamp_ns + i * kPeriod;
      arc.stamp_ns = now;
      arc.expiry_ns = now + 500 * kMs;
      turning.stamp_ns = now;
      report = monitor.update(arc, turning, now);
    }
    assert(!report.stop_required);
    assert(report.fault == ExecutionFault::kNone);
  }

  // A stop command with sustained measured motion is an actuator or brake
  // failure, independent of planner intent.
  {
    ExecutionMonitor monitor(config);
    const iface::MotionCommand stop =
        iface::MotionCommand::SafeStop(stationary.stamp_ns);
    EstimatedState moving = stationary;
    moving.forward_mps = 0.5;
    ExecutionMonitorReport report;
    for (int i = 0; i <= 50; ++i) {
      const auto now = stationary.stamp_ns + i * kPeriod;
      moving.stamp_ns = now;
      report = monitor.update(stop, moving, now);
    }
    assert(report.stop_required);
    assert(report.fault == ExecutionFault::kUncommandedMotion);
  }

  // Lost feedback after operation is a fault: without a fresh measurement
  // the controller cannot prove that the last command is no longer acting.
  {
    ExecutionMonitor monitor(config);
    EstimatedState old = stationary;
    old.forward_mps = 0.5;
    for (int i = 0; i <= 16; ++i) {
      const auto now = stationary.stamp_ns + i * kPeriod;
      drive.stamp_ns = now;
      drive.expiry_ns = now + 500 * kMs;
      const auto report = monitor.update(drive, old, now);
      if (i == 16) {
        assert(report.stop_required);
        assert(report.fault == ExecutionFault::kFeedbackMissing);
      }
    }
  }

  // Yaw command and opposite yaw feedback use the same operational band.
  {
    ExecutionMonitor monitor(config);
    iface::MotionCommand turn = drive;
    turn.forward_mps = 0.0;
    turn.yaw_rate_radps = 0.6;
    EstimatedState opposite = stationary;
    opposite.yaw_rate_radps = -0.4;
    ExecutionMonitorReport report;
    for (int i = 0; i <= 26; ++i) {
      const auto now = stationary.stamp_ns + i * kPeriod;
      turn.stamp_ns = now;
      turn.expiry_ns = now + 500 * kMs;
      opposite.stamp_ns = now;
      report = monitor.update(turn, opposite, now);
    }
    assert(report.stop_required);
    assert(report.fault == ExecutionFault::kDirectionMismatch);
  }

  std::puts("execution monitor ok");
  return 0;
}
