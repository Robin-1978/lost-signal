#pragma once

// Deterministic multi-rate scheduler (SIM orchestration, never ROBOT input).
// Mechanics ported verbatim from the main-stack deterministic_scheduler:
// each task owns an independent simulation-time deadline. Adapted naming:
// the third deadline is the episode RECORD cadence (replay is a provider
// in this framework, not a scheduled task).
#include <cmath>
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

 private:
  double period_s_ = 0.0;
  double next_s_ = 0.0;
};

class DeterministicScheduler {
 public:
  void reset(double scan_period_s, double decision_period_s,
             double record_period_s) {
    scan_.reset(scan_period_s);
    decision_.reset(decision_period_s);
    record_.reset(record_period_s);
  }

  bool scan_due(double now_s) { return scan_.due(now_s); }
  bool decision_due(double now_s) { return decision_.due(now_s); }
  bool record_due(double now_s) { return record_.due(now_s); }

  const PeriodicDeadline& scan() const { return scan_; }
  const PeriodicDeadline& decision() const { return decision_; }
  const PeriodicDeadline& record() const { return record_; }

 private:
  PeriodicDeadline scan_;
  PeriodicDeadline decision_;
  PeriodicDeadline record_;
};

}  // namespace lost_signal::sim::scenario
