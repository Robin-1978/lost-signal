#pragma once

#include <cstdint>

#include "robot/autonomy/estimation/estimator.hpp"
#include "robot/iface/command.hpp"

namespace lost_signal::robot::autonomy::diagnostics {

// Caller-owned execution envelope. The composition root derives these values
// from the declared navigation margin, chassis geometry, and decision period;
// this diagnostic does not carry a second set of vehicle tuning constants.
struct TrajectoryTrackingLimits {
 double position_m{0.0};
 double heading_rad{0.0};
 double forward_speed_mps{0.0};
 double yaw_rate_radps{0.0};
};

struct TrajectoryTrackingReport {
 std::uint64_t sequence{0};
 bool active{false};
 bool within_envelope{false};
 double elapsed_s{0.0};
 double position_error_m{0.0};
 double heading_error_rad{0.0};
 double forward_speed_error_mps{0.0};
 double yaw_rate_error_radps{0.0};
};

// Observes execution of the last accepted body-twist command. The expected
// pose uses the same exact constant-twist convention as the independent
// validator. Actual pose and velocity come only from the ROBOT estimator.
// The report is diagnostic evidence; it never authorizes or vetoes motion.
class TrajectoryTrackingSystem {
 public:
 void reset();

 // Accepting the same stamped command is idempotent so plant-rate updates do
 // not continually move the trajectory anchor between decision ticks.
 std::uint64_t accept(const iface::MotionCommand& command,
 const EstimatedState& estimate,
 TrajectoryTrackingLimits limits);

 TrajectoryTrackingReport observe(const EstimatedState& estimate) const;

 private:
 std::uint64_t sequence_{0};
 bool active_{false};
 iface::MotionCommand command_{};
 EstimatedState anchor_{};
 TrajectoryTrackingLimits limits_{};
};

} // namespace lost_signal::robot::autonomy::diagnostics
