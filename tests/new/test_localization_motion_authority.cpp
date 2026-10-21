#include "robot/autonomy/planning/local/local_planning_system.hpp"

#include <cmath>
#include <iostream>

#define CHECK(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ \
 << ": check failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

namespace {

sim::LocalPlanningRequest request() {
 sim::LocalPlanningRequest value;
 value.position = {2.0, 2.0};
 value.target = {5.0, 2.0};
 value.max_speed_mps = 0.8;
 value.max_yaw_rate_radps = 1.2;
 value.max_accel_mps2 = 1.0;
 value.max_decel_mps2 = 1.0;
 value.max_yaw_accel_radps2 = 2.0;
 value.prediction_horizon_s = 1.2;
 value.step_s = 0.1;
 value.robot_radius_m = 0.25;
 value.required_clearance_m = 0.05;
 return value;
}

} // namespace

int main() {
 sim::GridMap map(120, 120, 0.1);
 sim::LocalPlanningSystem planner;

 const auto nominal_request = request();
 const auto nominal = planner.plan(nominal_request, map, {});
 CHECK(nominal.selection.validation.accepted());
 CHECK(nominal.selection.trajectory.command.valid);
 CHECK(nominal.selection.trajectory.command.v > 0.0);
CHECK(nominal.status.mode == sim::LocalMotionMode::Nominal);
CHECK(nominal.status.motion_authorized);

 auto reduced_request = nominal_request;
 reduced_request.localization_confidence = 0.50;
 const auto reduced = planner.plan(reduced_request, map, {});
 CHECK(reduced.selection.validation.accepted());
 CHECK(reduced.selection.trajectory.command.valid);
 CHECK(reduced.selection.trajectory.command.v > 0.0);
 CHECK(reduced.selection.trajectory.command.v <=
 0.70 * nominal_request.max_speed_mps + 1e-9);
CHECK(reduced.status.mode == sim::LocalMotionMode::LocalizationReduced);

 // An executive hold closes the drive window. A moving chassis still emits a
 // physically reachable braking tail rather than pretending its measured
 // speed can jump to zero.
 auto held_request = nominal_request;
 held_request.forward_mps = 0.40;
 held_request.localization_confidence = 0.10;
 const auto held = planner.plan(held_request, map, {});
 CHECK(held.selection.validation.accepted());
 CHECK(held.selection.trajectory.command.valid);
 CHECK(held.selection.trajectory.command.v >= -1e-9);
 CHECK(held.selection.trajectory.command.v < held_request.forward_mps);
 CHECK(held.selection.trajectory.command.v <=
 held_request.forward_mps -
 held_request.max_decel_mps2 * held_request.step_s + 1e-9);
CHECK(held.status.mode == sim::LocalMotionMode::LocalizationHold);
CHECK(held.status.speed_cap_mps == 0.0);
CHECK(!held.status.recovery_condition.empty());

 auto stale_request = nominal_request;
 stale_request.forward_mps = 0.40;
 stale_request.localization_confidence = 0.50;
 stale_request.localization_age_s = 2.5;
 const auto stale = planner.plan(stale_request, map, {});
 CHECK(stale.selection.validation.accepted());
 CHECK(stale.selection.trajectory.command.valid);
 CHECK(stale.selection.trajectory.command.v <=
 0.70 * stale_request.max_speed_mps + 1e-9);

 // A stale map correction with bounded encoder/IMU covariance is degraded,
 // but it is not an unconditional stop. The planner retains the reduced
 // envelope until the quality itself falls below the hold floor.
 auto old_but_bounded = nominal_request;
 old_but_bounded.localization_confidence = 0.50;
 old_but_bounded.localization_age_s = 40.0;
 const auto old_bounded = planner.plan(old_but_bounded, map, {});
 CHECK(old_bounded.selection.validation.accepted());
 CHECK(old_bounded.status.mode == sim::LocalMotionMode::LocalizationReduced);
 CHECK(old_bounded.selection.trajectory.command.v > 0.0);

 // Invalid localization cannot be converted into ordinary motion by a
 // recovery branch. At standstill the safe command is zero.
 auto invalid_request = nominal_request;
 invalid_request.localization_valid = false;
 invalid_request.localization_confidence = 0.10;
 const auto invalid = planner.plan(invalid_request, map, {});
 CHECK(invalid.selection.validation.accepted());
 CHECK(invalid.selection.trajectory.command.valid);
 CHECK(std::abs(invalid.selection.trajectory.command.v) <= 1e-9);
CHECK(invalid.status.mode == sim::LocalMotionMode::LocalizationHold);
 return 0;
}
