#include "robot/autonomy/diagnostics/trajectory_tracking.hpp"

#include <algorithm>
#include <cmath>

#include "robot/core/geometry.hpp"

namespace lost_signal::robot::autonomy::diagnostics {
namespace {

bool same_command(const iface::MotionCommand& first,
 const iface::MotionCommand& second) {
 return first.space == second.space && first.stamp_ns == second.stamp_ns &&
 first.expiry_ns == second.expiry_ns &&
 first.forward_mps == second.forward_mps &&
 first.yaw_rate_radps == second.yaw_rate_radps &&
 first.steer_angle_rad == second.steer_angle_rad &&
 first.lateral_mps == second.lateral_mps;
}

double finite_nonnegative(double value) {
 return std::isfinite(value) ? std::max(0.0, value) : 0.0;
}

void integrate_body_twist(double forward_mps, double lateral_mps,
 double yaw_rate_radps, double elapsed_s,
 double& x_m, double& y_m, double& yaw_rad) {
 if (std::abs(yaw_rate_radps) > 1e-12) {
 const double final_heading = yaw_rad + yaw_rate_radps * elapsed_s;
 x_m +=
 (forward_mps * (std::sin(final_heading) - std::sin(yaw_rad)) +
 lateral_mps * (std::cos(final_heading) - std::cos(yaw_rad))) /
 yaw_rate_radps;
 y_m +=
 (-forward_mps * (std::cos(final_heading) - std::cos(yaw_rad)) +
 lateral_mps * (std::sin(final_heading) - std::sin(yaw_rad))) /
 yaw_rate_radps;
 } else {
 x_m += (forward_mps * std::cos(yaw_rad) -
 lateral_mps * std::sin(yaw_rad)) * elapsed_s;
 y_m += (forward_mps * std::sin(yaw_rad) +
 lateral_mps * std::cos(yaw_rad)) * elapsed_s;
 }
 yaw_rad = core::wrap_angle(yaw_rad + yaw_rate_radps * elapsed_s);
}

} // namespace

void TrajectoryTrackingSystem::reset() {
 sequence_ = 0;
 active_ = false;
 command_ = {};
 anchor_ = {};
 limits_ = {};
}

std::uint64_t TrajectoryTrackingSystem::accept(
 const iface::MotionCommand& command, const EstimatedState& estimate,
 TrajectoryTrackingLimits limits) {
 if (active_ && same_command(command_, command)) return sequence_;

 // A stop is covered by the braking/settling diagnostics. Steering-angle
 // commands are resolved below autonomy by the chassis adapter, while this
 // monitor owns only the body-twist contract published by autonomy.
 if (!estimate.valid || estimate.stamp_ns == core::kInvalidTime ||
 command.is_safe_stop() ||
 (command.space != iface::CommandSpace::kYawRate &&
 command.space != iface::CommandSpace::kHolonomic) ||
 command.stamp_ns == core::kInvalidTime ||
 estimate.stamp_ns > command.stamp_ns ||
 !iface::valid_motion_command(command, command.stamp_ns)) {
 active_ = false;
 command_ = command;
 return sequence_;
 }

 limits.position_m = finite_nonnegative(limits.position_m);
 limits.heading_rad = finite_nonnegative(limits.heading_rad);
 limits.forward_speed_mps = finite_nonnegative(limits.forward_speed_mps);
 limits.yaw_rate_radps = finite_nonnegative(limits.yaw_rate_radps);
 command_ = command;
 anchor_ = estimate;
 // Observation delivery may lag the decision clock. Advance the ROBOT pose
 // estimate to command issue time using its own measured twist; otherwise
 // transport latency would appear as a controller tracking error.
 const double anchor_dt_s =
 static_cast<double>(command.stamp_ns - estimate.stamp_ns) * 1e-9;
 integrate_body_twist(
 estimate.forward_mps, 0.0, estimate.yaw_rate_radps, anchor_dt_s,
 anchor_.x_m, anchor_.y_m, anchor_.yaw_rad);
 anchor_.stamp_ns = command.stamp_ns;
 limits_ = limits;
 active_ = true;
 return ++sequence_;
}

TrajectoryTrackingReport TrajectoryTrackingSystem::observe(
 const EstimatedState& estimate) const {
 TrajectoryTrackingReport out;
 out.sequence = sequence_;
 if (!active_ || !estimate.valid ||
 estimate.stamp_ns == core::kInvalidTime ||
 estimate.stamp_ns < command_.stamp_ns ||
 estimate.stamp_ns > command_.expiry_ns) {
 return out;
 }

 const double elapsed_s =
 static_cast<double>(estimate.stamp_ns - command_.stamp_ns) * 1e-9;
 double expected_x = anchor_.x_m;
 double expected_y = anchor_.y_m;
 const double forward = command_.forward_mps;
 const double lateral = command_.space == iface::CommandSpace::kHolonomic
 ? command_.lateral_mps
 : 0.0;
 const double yaw_rate = command_.yaw_rate_radps;
 double expected_heading = anchor_.yaw_rad;
 integrate_body_twist(
 forward, lateral, yaw_rate, elapsed_s,
 expected_x, expected_y, expected_heading);

 out.active = true;
 out.elapsed_s = elapsed_s;
 out.position_error_m =
 std::hypot(estimate.x_m - expected_x, estimate.y_m - expected_y);
 out.heading_error_rad =
 std::abs(core::wrap_angle(estimate.yaw_rad - expected_heading));
 out.forward_speed_error_mps =
 std::abs(estimate.forward_mps - command_.forward_mps);
 out.yaw_rate_error_radps =
 std::abs(estimate.yaw_rate_radps - command_.yaw_rate_radps);

 // Estimator uncertainty widens only the corresponding pose envelope. It
 // cannot hide a speed/yaw-rate tracking error, and SIM truth is absent.
 const double position_limit =
 limits_.position_m + finite_nonnegative(estimate.position_sigma_m);
 const double heading_limit =
 limits_.heading_rad + finite_nonnegative(estimate.heading_sigma_rad);
 out.within_envelope =
 out.position_error_m <= position_limit + 1e-12 &&
 out.heading_error_rad <= heading_limit + 1e-12 &&
 out.forward_speed_error_mps <= limits_.forward_speed_mps + 1e-12 &&
 out.yaw_rate_error_radps <= limits_.yaw_rate_radps + 1e-12;
 return out;
}

} // namespace lost_signal::robot::autonomy::diagnostics
