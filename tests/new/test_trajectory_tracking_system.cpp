#include "autonomy/control/trajectory_tracking_system.hpp"

#include <cmath>
#include <iostream>

#define CHECK(condition)                                                                                     \
 do {                                                                                                        \
  if (!(condition)) {                                                                                        \
   std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n";                           \
   return 1;                                                                                                 \
  }                                                                                                          \
 } while (false)

int main() {
 sim::TrajectoryTrackingSystem tracker;
 const auto inactive = tracker.observe(1.0, {}, 0.0, 0.0);
 CHECK(!inactive.active);
 CHECK(inactive.sequence == 0);

 sim::ExecutableTrajectory trajectory;
 trajectory.command.valid = true;
 trajectory.samples = {
   {0.0, {1.0, 2.0}, 3.10, 0.5, 0.0, 0.0, 0.0},
   {1.0, {3.0, 4.0}, -3.10, 1.5, 0.0, 0.0, 0.0},
 };
 const sim::TrajectoryTrackingLimits limits{0.20, 0.10, 0.15};
 CHECK(tracker.accept(10.0, trajectory, limits) == 1);

 // Heading interpolation takes the short path across +/-pi.
 const double midpoint_heading = sim::kPi;
 const auto exact = tracker.observe(10.5, {2.0, 3.0}, midpoint_heading, 1.0);
 CHECK(exact.active);
 CHECK(exact.within_envelope);
 CHECK(exact.position_error_m < 1e-12);
 CHECK(exact.heading_error_rad < 1e-12);
 CHECK(exact.speed_error_mps < 1e-12);

 const auto position_error = tracker.observe(10.5, {2.21, 3.0}, midpoint_heading, 1.0);
 CHECK(position_error.position_error_m > limits.position_m);
 CHECK(position_error.heading_error_rad < 1e-12);
 CHECK(position_error.speed_error_mps < 1e-12);
 CHECK(!position_error.within_envelope);

 const auto heading_error = tracker.observe(10.5, {2.0, 3.0}, midpoint_heading + 0.11, 1.0);
 CHECK(heading_error.position_error_m < 1e-12);
 CHECK(heading_error.heading_error_rad > limits.heading_rad);
 CHECK(heading_error.speed_error_mps < 1e-12);
 CHECK(!heading_error.within_envelope);

 const auto speed_error = tracker.observe(10.5, {2.0, 3.0}, midpoint_heading, 1.16);
 CHECK(speed_error.position_error_m < 1e-12);
 CHECK(speed_error.heading_error_rad < 1e-12);
 CHECK(speed_error.speed_error_mps > limits.speed_mps);
 CHECK(!speed_error.within_envelope);

 CHECK(!tracker.observe(9.9, {}, 0.0, 0.0).active);
 CHECK(!tracker.observe(11.1, {}, 0.0, 0.0).active);
 CHECK(tracker.accept(20.0, trajectory, limits) == 2);
 tracker.reset();
 const auto reset = tracker.observe(20.0, {}, 0.0, 0.0);
 CHECK(!reset.active);
 CHECK(reset.sequence == 0);
 return 0;
}
