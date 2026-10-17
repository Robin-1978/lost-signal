#include "robot/autonomy/control/trajectory_tracking_system.hpp"

#include "robot/core/math.hpp"

#include <algorithm>
#include <cmath>

namespace sim {

void TrajectoryTrackingSystem::reset() {
 generated_at_s_ = 0.0;
 sequence_ = 0;
 trajectory_ = {};
 limits_ = {};
}

std::uint64_t TrajectoryTrackingSystem::accept(double generated_at_s,
                                               const ExecutableTrajectory& trajectory,
                                               const TrajectoryTrackingLimits& limits) {
 generated_at_s_ = generated_at_s;
 trajectory_ = trajectory;
 limits_ = limits;
 return ++sequence_;
}

TrajectoryTrackingReport TrajectoryTrackingSystem::observe(double time_s, Vec2 position,
                                                           double heading_rad, double forward_mps) const {
 TrajectoryTrackingReport out;
 out.sequence = sequence_;
 if (trajectory_.samples.size() < 2 || time_s < generated_at_s_ - 1e-9)
  return out;
 const double elapsed = time_s - generated_at_s_;
 if (elapsed > trajectory_.samples.back().t_s + 1e-9)
  return out;

 const auto upper =
   std::lower_bound(trajectory_.samples.begin(), trajectory_.samples.end(), elapsed,
                    [](const TrajectorySample& sample, double value) { return sample.t_s < value; });
 TrajectorySample expected;
 if (upper == trajectory_.samples.begin()) {
  expected = *upper;
 } else if (upper == trajectory_.samples.end()) {
  expected = trajectory_.samples.back();
 } else {
  const auto& before = *(upper - 1);
  const auto& after = *upper;
  const double blend = (elapsed - before.t_s) / std::max(1e-9, after.t_s - before.t_s);
  expected.t_s = elapsed;
  expected.position = {
    before.position.x + (after.position.x - before.position.x) * blend,
    before.position.y + (after.position.y - before.position.y) * blend,
  };
  expected.heading_rad =
    wrap_angle(before.heading_rad + wrap_angle(after.heading_rad - before.heading_rad) * blend);
  expected.forward_mps = before.forward_mps + (after.forward_mps - before.forward_mps) * blend;
 }

 out.active = true;
 out.position_error_m = std::hypot(position.x - expected.position.x, position.y - expected.position.y);
 out.heading_error_rad = std::abs(wrap_angle(heading_rad - expected.heading_rad));
 out.speed_error_mps = std::abs(forward_mps - expected.forward_mps);
 out.within_envelope = out.position_error_m <= limits_.position_m + 1e-9 &&
                       out.heading_error_rad <= limits_.heading_rad + 1e-9 &&
                       out.speed_error_mps <= limits_.speed_mps + 1e-9;
 return out;
}

} // namespace sim
