#include <cassert>
#include <cmath>
#include <cstdio>

#include "robot/autonomy/diagnostics/trajectory_tracking.hpp"
#include "robot/core/geometry.hpp"

int main() {
 using namespace lost_signal::robot;
 using autonomy::diagnostics::TrajectoryTrackingLimits;
 using autonomy::diagnostics::TrajectoryTrackingSystem;

 TrajectoryTrackingSystem tracker;
 autonomy::EstimatedState anchor;
 anchor.valid = true;
 anchor.stamp_ns = 1'000'000'000;
 anchor.x_m = 1.0;
 anchor.y_m = 2.0;
 anchor.yaw_rad = core::kPi - 0.1;

 assert(!tracker.observe(anchor).active);

 iface::MotionCommand command;
 command.space = iface::CommandSpace::kHolonomic;
 command.stamp_ns = anchor.stamp_ns;
 command.expiry_ns = 2'000'000'000;
 command.forward_mps = 1.0;
 command.lateral_mps = 0.2;
 command.yaw_rate_radps = 0.4;
 const TrajectoryTrackingLimits limits{0.10, 0.08, 0.20, 0.15};
 assert(tracker.accept(command, anchor, limits) == 1);
 // Re-publishing the same decision is idempotent and keeps the original
 // execution anchor.
 autonomy::EstimatedState later_anchor = anchor;
 later_anchor.x_m = 99.0;
 assert(tracker.accept(command, later_anchor, limits) == 1);

 autonomy::EstimatedState exact = anchor;
 exact.stamp_ns = 1'500'000'000;
 const double elapsed_s = 0.5;
 const double final_heading = anchor.yaw_rad + command.yaw_rate_radps * elapsed_s;
 exact.x_m +=
 (command.forward_mps *
 (std::sin(final_heading) - std::sin(anchor.yaw_rad)) +
 command.lateral_mps *
 (std::cos(final_heading) - std::cos(anchor.yaw_rad))) /
 command.yaw_rate_radps;
 exact.y_m +=
 (-command.forward_mps *
 (std::cos(final_heading) - std::cos(anchor.yaw_rad)) +
 command.lateral_mps *
 (std::sin(final_heading) - std::sin(anchor.yaw_rad))) /
 command.yaw_rate_radps;
 exact.yaw_rad = core::wrap_angle(final_heading);
 exact.forward_mps = command.forward_mps;
 exact.yaw_rate_radps = command.yaw_rate_radps;
 exact.position_sigma_m = 0.01;
 exact.heading_sigma_rad = 0.01;
 const auto exact_report = tracker.observe(exact);
 assert(exact_report.active && exact_report.within_envelope);
 assert(std::abs(exact_report.elapsed_s - elapsed_s) < 1e-12);
 assert(exact_report.position_error_m < 1e-12);
 assert(exact_report.heading_error_rad < 1e-12);

 autonomy::EstimatedState deviated = exact;
 deviated.x_m += 0.12;
 deviated.forward_mps += 0.21;
 const auto deviated_report = tracker.observe(deviated);
 assert(deviated_report.active && !deviated_report.within_envelope);
 assert(deviated_report.position_error_m > limits.position_m);
 assert(deviated_report.forward_speed_error_mps > limits.forward_speed_mps);

 autonomy::EstimatedState expired = exact;
 expired.stamp_ns = command.expiry_ns + 1;
 assert(!tracker.observe(expired).active);

 // A delayed observation is advanced to command time with measured ROBOT
 // odometry before tracking starts; latency itself is not execution error.
 TrajectoryTrackingSystem delayed_tracker;
 autonomy::EstimatedState delayed_anchor;
 delayed_anchor.valid = true;
 delayed_anchor.stamp_ns = 3'000'000'000;
 delayed_anchor.forward_mps = 0.5;
 iface::MotionCommand delayed_command;
 delayed_command.space = iface::CommandSpace::kYawRate;
 delayed_command.stamp_ns = 3'100'000'000;
 delayed_command.expiry_ns = 3'500'000'000;
 delayed_command.forward_mps = 1.0;
 assert(delayed_tracker.accept(delayed_command, delayed_anchor, limits) == 1);
 autonomy::EstimatedState delayed_exact = delayed_anchor;
 delayed_exact.stamp_ns = 3'200'000'000;
 delayed_exact.x_m = 0.05 + 0.10;
 delayed_exact.forward_mps = 1.0;
 const auto delayed_report = delayed_tracker.observe(delayed_exact);
 assert(delayed_report.active && delayed_report.within_envelope);
 assert(delayed_report.position_error_m < 1e-12);

 const iface::MotionCommand stop =
 iface::MotionCommand::SafeStop(2'100'000'000);
 assert(tracker.accept(stop, exact, limits) == 1);
 assert(!tracker.observe(exact).active);

 std::printf("trajectory tracking ok\n");
 return 0;
}
