#pragma once

// Deterministic multi-rate scheduler (SIM orchestration, never ROBOT input).
// Deadlines are integer nanoseconds so sensor cadence cannot accumulate
// floating-point drift or change the fixed-step physics partition.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace lost_signal::sim::scenario {

using SimTimeNs = std::int64_t;

inline constexpr SimTimeNs kNsPerSecond = 1'000'000'000;
inline constexpr SimTimeNs kPhysicsQuantumNs = 1'000'000; // 1 ms
inline constexpr SimTimeNs kOuterStepNs = 20'000'000; // 20 ms

inline SimTimeNs seconds_to_sim_ns(double seconds) {
 if (!std::isfinite(seconds) || seconds < 0.0 ||
 seconds > static_cast<double>(std::numeric_limits<SimTimeNs>::max()) /
 static_cast<double>(kNsPerSecond)) {
 throw std::invalid_argument(
 "simulation timing requires finite non-negative seconds");
 }
 return static_cast<SimTimeNs>(
 std::llround(seconds * static_cast<double>(kNsPerSecond)));
}

inline double sim_ns_to_seconds(SimTimeNs stamp_ns) {
 return static_cast<double>(stamp_ns) /
 static_cast<double>(kNsPerSecond);
}

inline bool on_physics_grid(double period_s) {
 try {
 const SimTimeNs period_ns = seconds_to_sim_ns(period_s);
 return period_ns == 0 || period_ns % kPhysicsQuantumNs == 0;
 } catch (const std::invalid_argument&) {
 return false;
 }
}

class PeriodicDeadline {
 public:
 void reset_ns(SimTimeNs period_ns, SimTimeNs first_deadline_ns = 0) {
 if (period_ns < 0 || first_deadline_ns < 0) {
 throw std::invalid_argument(
 "periodic deadline requires non-negative timing");
 }
 period_ns_ = period_ns;
 next_ns_ = first_deadline_ns;
 }

 void reset(double period_s, double first_deadline_s = 0.0) {
 reset_ns(seconds_to_sim_ns(period_s),
 seconds_to_sim_ns(first_deadline_s));
 }

 bool due_ns(SimTimeNs now_ns) {
 if (period_ns_ == 0 || now_ns < next_ns_) return false;
 next_ns_ += period_ns_;
 return true;
 }

 bool due(double now_s) { return due_ns(seconds_to_sim_ns(now_s)); }

 SimTimeNs next_ns() const { return next_ns_; }
 double next_s() const { return sim_ns_to_seconds(next_ns_); }
 bool enabled() const { return period_ns_ > 0; }

 private:
 SimTimeNs period_ns_ = 0;
 SimTimeNs next_ns_ = 0;
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
 reset_ns(seconds_to_sim_ns(scan_period_s),
 seconds_to_sim_ns(motion_period_s), seconds_to_sim_ns(imu_period_s),
 seconds_to_sim_ns(decision_period_s),
 seconds_to_sim_ns(record_period_s));
 }

 void reset_ns(SimTimeNs scan_period_ns, SimTimeNs motion_period_ns,
 SimTimeNs imu_period_ns, SimTimeNs decision_period_ns,
 SimTimeNs record_period_ns) {
 scan_.reset_ns(scan_period_ns);
 motion_.reset_ns(motion_period_ns);
 imu_.reset_ns(imu_period_ns);
 decision_.reset_ns(decision_period_ns);
 record_.reset_ns(record_period_ns);
 }

 // Runtime form: the first event occurs after one complete period. The
 // zero-origin reset above remains available to callers that explicitly
 // need an immediate event.
 void reset_after_ns(SimTimeNs now_ns, SimTimeNs scan_period_ns,
 SimTimeNs motion_period_ns, SimTimeNs imu_period_ns,
 SimTimeNs decision_period_ns, SimTimeNs record_period_ns) {
 auto next = [now_ns](SimTimeNs period_ns) {
 return period_ns > 0 ? now_ns + period_ns : now_ns;
 };
 scan_.reset_ns(scan_period_ns, next(scan_period_ns));
 motion_.reset_ns(motion_period_ns, next(motion_period_ns));
 imu_.reset_ns(imu_period_ns, next(imu_period_ns));
 decision_.reset_ns(decision_period_ns, next(decision_period_ns));
 record_.reset_ns(record_period_ns, next(record_period_ns));
 }

 void reset_after(double now_s, double scan_period_s,
 double motion_period_s, double imu_period_s,
 double decision_period_s, double record_period_s) {
 reset_after_ns(seconds_to_sim_ns(now_s),
 seconds_to_sim_ns(scan_period_s), seconds_to_sim_ns(motion_period_s),
 seconds_to_sim_ns(imu_period_s), seconds_to_sim_ns(decision_period_s),
 seconds_to_sim_ns(record_period_s));
 }

 bool scan_due_ns(SimTimeNs now_ns) { return scan_.due_ns(now_ns); }
 bool motion_due_ns(SimTimeNs now_ns) { return motion_.due_ns(now_ns); }
 bool imu_due_ns(SimTimeNs now_ns) { return imu_.due_ns(now_ns); }
 bool decision_due_ns(SimTimeNs now_ns) {
 return decision_.due_ns(now_ns);
 }
 bool record_due_ns(SimTimeNs now_ns) { return record_.due_ns(now_ns); }

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
 SimTimeNs next_runtime_event_ns() const {
 SimTimeNs next = std::numeric_limits<SimTimeNs>::max();
 auto include = [&next](const PeriodicDeadline& deadline) {
 if (deadline.enabled()) next = std::min(next, deadline.next_ns());
 };
 include(scan_);
 include(motion_);
 include(imu_);
 include(decision_);
 return next;
 }

 double next_runtime_event_s() const {
 const SimTimeNs next = next_runtime_event_ns();
 return next == std::numeric_limits<SimTimeNs>::max()
 ? std::numeric_limits<double>::infinity()
 : sim_ns_to_seconds(next);
 }

 private:
 PeriodicDeadline scan_;
 PeriodicDeadline motion_;
 PeriodicDeadline imu_;
 PeriodicDeadline decision_;
 PeriodicDeadline record_;
};

} // namespace lost_signal::sim::scenario
