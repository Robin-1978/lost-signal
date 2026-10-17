#pragma once

#include "robot/autonomy/planning/local/dwa.hpp"

#include <cstdint>

namespace sim {

struct TrajectoryTrackingLimits {
 double position_m{0.25};
 double heading_rad{0.25};
 double speed_mps{0.5};
};

struct TrajectoryTrackingReport {
 std::uint64_t sequence{0};
 bool active{false};
 bool within_envelope{false};
 double position_error_m{0.0};
 double heading_error_rad{0.0};
 double speed_error_mps{0.0};
};

// Owns the accepted local trajectory between decision ticks. It compares the
// next estimated state with the exact rollout previously handed to control;
// it never advances physics or modifies commands.
class TrajectoryTrackingSystem {
 public:
 void reset();
 std::uint64_t accept(double generated_at_s, const ExecutableTrajectory& trajectory,
                      const TrajectoryTrackingLimits& limits);
 TrajectoryTrackingReport observe(double time_s, Vec2 position, double heading_rad,
                                  double forward_mps) const;

 private:
 double generated_at_s_{0.0};
 std::uint64_t sequence_{0};
 ExecutableTrajectory trajectory_;
 TrajectoryTrackingLimits limits_;
};

} // namespace sim
