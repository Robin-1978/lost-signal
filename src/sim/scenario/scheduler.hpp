#pragma once

// Deterministic multi-rate scheduler (SIM orchestration, never ROBOT input).
// Mechanics ported verbatim from the main-stack deterministic_scheduler:
// each task owns an independent simulation-time deadline. Adapted naming:
// the third deadline is the episode RECORD cadence (replay is a provider
// in this framework, not a scheduled task).
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace lost_signal::sim::scenario {

class PeriodicDeadline {
 public:
  void reset(double period_s, double first_deadline_s = 0.0) {
    if (period_s < 0.0 || !std::isfinite(period_s) ||
        !std::isfinite(first_deadline_s)) {
      throw std::invalid_argument(
          "periodic deadline requires finite non-negative timing");
    }
    period_s_ = period_s;
    next_s_ = first_deadline_s;
  }

  bool due(double now_s) {
    if (period_s_ == 0.0) return false;
    if (!std::isfinite(now_s) || now_s + 1e-9 < next_s_) return false;
    next_s_ += period_s_;
    return true;
  }

  double next_s() const { return next_s_; }
 bool enabled() const { return period_s_ > 0.0; }

 private:
  double period_s_ = 0.0;
  double next_s_ = 0.0;
};

class DeterministicScheduler {
 public:
  void reset(double scan_period_s, double decision_period_s,
             double record_period_s) {
 reset(scan_period_s, scan_period_s, scan_period_s, decision_period_s,
 record_period_s);
 }

 void reset(double scan_period_s, double motion_period_s,
 double imu_period_s, double decision_period_s, double record_period_s) {
    scan_.reset(scan_period_s);
 motion_.reset(motion_period_s);
 imu_.reset(imu_period_s);
    decision_.reset(decision_period_s);
    record_.reset(record_period_s);
  }

 // Runtime form: the first event occurs after one complete period. The
 // zero-origin reset above remains available to callers that explicitly
 // need an immediate event.
 void reset_after(double now_s, double scan_period_s,
 double motion_period_s, double imu_period_s,
 double decision_period_s, double record_period_s) {
 auto next = [now_s](double period_s) {
 return period_s > 0.0 ? now_s + period_s : now_s;
 };
 scan_.reset(scan_period_s, next(scan_period_s));
 motion_.reset(motion_period_s, next(motion_period_s));
 imu_.reset(imu_period_s, next(imu_period_s));
 decision_.reset(decision_period_s, next(decision_period_s));
 record_.reset(record_period_s, next(record_period_s));
 }

  bool scan_due(double now_s) { return scan_.due(now_s); }
 bool motion_due(double now_s) { return motion_.due(now_s); }
 bool imu_due(double now_s) { return imu_.due(now_s); }
  bool decision_due(double now_s) { return decision_.due(now_s); }
  bool record_due(double now_s) { return record_.due(now_s); }

  const PeriodicDeadline& scan() const { return scan_; }
 const PeriodicDeadline& motion() const { return motion_; }
 const PeriodicDeadline& imu() const { return imu_; }
  const PeriodicDeadline& decision() const { return decision_; }
  const PeriodicDeadline& record() const { return record_; }

 // Recording remains an outer-loop concern so one public session step
 // produces at most one replay transition.
 double next_runtime_event_s() const {
 double next = std::numeric_limits<double>::infinity();
 auto include = [&next](const PeriodicDeadline& deadline) {
 if (deadline.enabled()) next = std::min(next, deadline.next_s());
 };
 include(scan_);
 include(motion_);
 include(imu_);
 include(decision_);
 return next;
 }

 private:
  PeriodicDeadline scan_;
 PeriodicDeadline motion_;
 PeriodicDeadline imu_;
  PeriodicDeadline decision_;
  PeriodicDeadline record_;
};

}  // namespace lost_signal::sim::scenario
