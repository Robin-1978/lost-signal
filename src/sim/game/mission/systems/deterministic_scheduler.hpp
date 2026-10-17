#pragma once

#include <cmath>
#include <stdexcept>

namespace mission {

// Deterministic multi-rate scheduler for the single-thread reference runner.
// Each task owns an independent simulation-time deadline. The same deadlines
// can later drive worker threads or processes without changing sensor/control
// cadence or substituting wall-clock arrival time for acquisition time.
class PeriodicDeadline {
 public:
 void reset(double period_s, double first_deadline_s = 0.0) {
 if (period_s < 0.0 || !std::isfinite(period_s) || !std::isfinite(first_deadline_s))
 throw std::invalid_argument("periodic deadline requires finite non-negative timing");
 period_s_ = period_s;
 next_s_ = first_deadline_s;
 }

 bool due(double now_s) {
 if (period_s_ == 0.0)
 return false;
 if (!std::isfinite(now_s) || now_s + 1e-9 < next_s_)
 return false;
 next_s_ += period_s_;
 return true;
 }

 double next_s() const { return next_s_; }

 private:
 double period_s_{0.0};
 double next_s_{0.0};
};

class DeterministicScheduler {
 public:
 void reset(double scan_period_s, double decision_period_s,
 double replay_period_s) {
 scan_.reset(scan_period_s);
 decision_.reset(decision_period_s);
 replay_.reset(replay_period_s);
 }

 bool scan_due(double now_s) { return scan_.due(now_s); }
 bool decision_due(double now_s) { return decision_.due(now_s); }
 bool replay_due(double now_s) { return replay_.due(now_s); }

 const PeriodicDeadline& scan() const { return scan_; }
 const PeriodicDeadline& decision() const { return decision_; }
 const PeriodicDeadline& replay() const { return replay_; }

 private:
 PeriodicDeadline scan_;
 PeriodicDeadline decision_;
 PeriodicDeadline replay_;
};

} // namespace mission
