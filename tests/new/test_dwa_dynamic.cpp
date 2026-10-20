#include "robot/autonomy/planning/local/dwa.hpp"
#include "robot/autonomy/planning/local/ackermann_local.hpp"
#include "robot/autonomy/planning/local/holonomic_local.hpp"
#include "robot/autonomy/planning/local/dynamic_clearance.hpp"
#include "robot/autonomy/planning/validation/trajectory_validator.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

#define CHECK(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

namespace {

const sim::DwaRollout* rollout_near(
 const sim::DwaResult& result, double velocity_mps, double yaw_rate_radps) {
 const sim::DwaRollout* best = nullptr;
 double error = 1e9;
 for (const auto& rollout : result.rollouts) {
 const double candidate_error =
 std::abs(rollout.trajectory.command.v - velocity_mps) +
 std::abs(rollout.trajectory.command.w - yaw_rate_radps);
 if (candidate_error < error) {
 error = candidate_error;
 best = &rollout;
 }
 }
 return best;
}

} // namespace

int main() {
 sim::GridMap map(100, 100, 0.1);
 const sim::DwaWeights weights{1.0, 2.0, 0.3, 1.2, 0.8, 0.25};

 // The command published now must belong to the next control period's
 // dynamic window. A long prediction horizon is not permission to send its
 // terminal velocity immediately. If a route transition drops the policy
 // cap below the measured speed, DWA emits the nearest reachable braking
 // command and stages the stop over subsequent decisions.
 const auto staged_stop = sim::dwa_compute_trajectory(
 {1.0, 1.0}, 0.0, 0.75, 0.74, {1.0, 1.0}, map, weights,
 {}, 0.0, 1.2, 3.0, 1.5, 0.1, 0.1,
 0.0, 0.20, 4.0, {}, 0.0, std::nullopt, {}, 0.50, 2.0);
 CHECK(staged_stop.best.command.valid);
 CHECK(!staged_stop.rollouts.empty());
 for (const auto& rollout : staged_stop.rollouts) {
 CHECK(std::abs(rollout.trajectory.command.v - 0.75) <= 3.0 * 0.1 + 1e-9);
 CHECK(std::abs(rollout.trajectory.command.w - 0.74) <= 4.0 * 0.1 + 1e-9);
 CHECK(rollout.trajectory.command.v >= 0.45 - 1e-9);
 const double current_left = 0.75 - 0.74 * 0.25;
 const double current_right = 0.75 + 0.74 * 0.25;
 const double target_left = rollout.trajectory.command.v -
 rollout.trajectory.command.w * 0.25;
 const double target_right = rollout.trajectory.command.v +
 rollout.trajectory.command.w * 0.25;
 CHECK(std::abs(target_left - current_left) <= 3.0 * 0.1 + 1e-9);
 CHECK(std::abs(target_right - current_right) <= 3.0 * 0.1 + 1e-9);
 }
 CHECK(staged_stop.best.command.v >= 0.45 - 1e-9);

 // Skid-steer wheel reachability is measured in encoder space. Tread scrub
 // can leave body yaw far below the kinematic rate while the tracks have
 // already reached the previous command. Inferring track speed from body yaw
 // would repeatedly reopen only the first angular sample and permanently cap
 // a real tracked vehicle at 0.3 rad/s.
 const auto body_twist_inferred = sim::dwa_compute_trajectory(
 {5.0, 5.0}, 0.0, 0.0, 0.03, {5.0, 8.0}, map, weights,
 {}, 0.0, 1.5, 4.2, 1.2, 0.1, 0.1,
 0.0, 0.42, 8.0, {}, 0.0, sim::kPi * 0.5, {}, 2.4, 2.4);
 const auto encoder_reachable = sim::dwa_compute_trajectory(
 {5.0, 5.0}, 0.0, 0.0, 0.03, {5.0, 8.0}, map, weights,
 {}, 0.0, 1.5, 4.2, 1.2, 0.1, 0.1,
 0.0, 0.42, 8.0, {}, 0.0, sim::kPi * 0.5, {}, 2.4, 2.4,
 0.0, -1.0, 0.0, sim::DifferentialWheelSpeeds{-0.36, 0.36});
 const auto maximum_yaw = [](const sim::DwaResult& result) {
 double value = 0.0;
 for (const auto& rollout : result.rollouts)
 value = std::max(value, rollout.trajectory.command.w);
 return value;
 };
 CHECK(maximum_yaw(body_twist_inferred) <= 0.40);
 CHECK(maximum_yaw(encoder_reachable) >= 0.59);

 // A loaded track can lag its requested tread speed. The reachable yaw
 // boundary then lies between the fixed 0.3 rad/s samples. It must still be
 // proposed: otherwise the controller remains below breakaway effort and the
 // planner can spend a minute on a turn that the same chassis performs in a
 // few seconds at a reachable higher setpoint.
 const auto loaded_track_boundary = sim::dwa_compute_trajectory(
 {5.0, 5.0}, 0.0, 0.0, -0.05, {5.0, 2.0}, map, weights,
 {}, 0.0, 1.5, 6.0, 1.2, 0.1, 0.1,
 0.0, 0.42, 8.0, {}, 0.0, -sim::kPi * 0.5, {}, 2.4, 2.4,
 0.0, -1.0, 0.0, sim::DifferentialWheelSpeeds{0.10, -0.10});
 double minimum_loaded_yaw = 0.0;
 for (const auto& rollout : loaded_track_boundary.rollouts)
 minimum_loaded_yaw = std::min(minimum_loaded_yaw, rollout.trajectory.command.w);
 CHECK(minimum_loaded_yaw < -0.55);
 CHECK(minimum_loaded_yaw >= -0.59);

 // Normal acceleration and normal braking are independent actuator limits.
 // A planner may accelerate by 0.30 m/s in this update but may reduce speed
 // by only 0.05 m/s; the dynamic window must preserve that asymmetry.
 const auto asymmetric_window = sim::dwa_compute_trajectory(
 {1.0, 1.0}, 0.0, 0.75, 0.0, {8.0, 1.0}, map, weights,
 {}, 2.0, 1.2, 3.0, 1.0, 0.1, 0.1,
 0.0, 0.20, 4.0, {}, 0.0, std::nullopt, {}, 0.0, 0.0, 0.0, 0.5);
 CHECK(asymmetric_window.best.command.valid);
 for (const auto& rollout : asymmetric_window.rollouts) {
 CHECK(rollout.trajectory.command.v >= 0.70 - 1e-9);
 CHECK(rollout.trajectory.command.v <= 1.05 + 1e-9);
 }

 // Braking an opposite-direction disturbance cannot spend unused braking
 // authority as acceleration in the commanded route direction. The first
 // update reaches zero; yaw remains reachable so a spin-capable chassis is
 // not locked out of an in-place heading correction.
 const auto rollback_window = sim::dwa_compute_trajectory(
 {1.0, 1.0}, 0.0, -0.02, 0.0, {8.0, 1.0}, map, weights,
 {}, 2.0, 1.2, 3.0, 1.0, 0.1, 0.1,
 0.0, 0.20, 4.0, {}, 0.0, std::nullopt, {}, 0.58, 2.4, 0.0, 4.0);
 CHECK(rollback_window.best.command.valid);
 for (const auto& rollout : rollback_window.rollouts)
 CHECK(rollout.trajectory.command.v <= 1e-12);
 // The goal is already straight ahead: the selected command should not yaw,
 // but the sampler must still contain turn candidates.
 CHECK(std::abs(rollback_window.best.command.w) <= 1e-9);
 CHECK(std::any_of(rollback_window.rollouts.begin(), rollback_window.rollouts.end(),
 [](const auto& rollout) {
 return std::abs(rollout.trajectory.command.w) > 0.1;
 }));

 // A heading error must still produce a turn while the longitudinal
 // recovery holds v at zero. Otherwise the vehicle can never rotate out of
 // the error and the route follower's zero-speed spin request deadlocks.
 const auto rollback_turn = sim::dwa_compute_trajectory(
 {1.0, 1.0}, 0.0, -0.02, 0.0, {1.0, 8.0}, map, weights,
 {}, 2.0, 1.2, 3.0, 1.0, 0.1, 0.1,
 0.0, 0.20, 4.0, {}, 0.0, sim::kPi * 0.5, {}, 0.58, 2.4, 0.0, 4.0);
 CHECK(rollback_turn.best.command.valid);
 CHECK(rollback_turn.best.command.v <= 1e-12);
 CHECK(rollback_turn.best.command.w > 0.1);

 const auto forward_disturbance_on_reverse_route = sim::dwa_compute_trajectory(
 {1.0, 1.0}, 0.0, 0.02, 0.0, {0.0, 1.0}, map, weights,
 {}, 0.0, 1.2, 3.0, 1.0, 0.1, 0.1,
 0.0, 0.20, 4.0, {}, -2.0, std::nullopt, {}, 0.0, 0.0, 0.0, 4.0);
 CHECK(forward_disturbance_on_reverse_route.best.command.valid);
 for (const auto& rollout : forward_disturbance_on_reverse_route.rollouts)
 CHECK(std::abs(rollout.trajectory.command.v) <= 1e-12);

 // A sub-5 mm/s estimator residual is measured standstill, not a permanent
 // forward gear. The command window may engage reverse while rollout
 // integration still starts from the unmodified positive measurement and
 // models the actual sign transition.
 const auto noisy_standstill_on_reverse_route = sim::dwa_compute_trajectory(
 {1.0, 1.0}, 0.0, 0.001, 0.0, {0.0, 1.0}, map, weights,
 {}, 0.0, 1.2, 3.0, 1.0, 0.1, 0.1,
 0.0, 0.20, 4.0, {}, -2.0, std::nullopt, {}, 0.0, 0.0, 0.0, 4.0);
 CHECK(noisy_standstill_on_reverse_route.best.command.valid);
 CHECK(noisy_standstill_on_reverse_route.best.command.v < 0.0);

 // Once the opposite-direction disturbance is stopped, the reverse gear's
 // speed critic rewards magnitude in that declared direction. Signed speed
 // used as a forward-only reward makes zero a permanent local optimum.
 const auto reverse_progress = sim::dwa_compute_trajectory(
 {1.0, 1.0}, 0.0, 0.0, 0.0, {0.0, 1.0}, map, weights,
 {}, 0.0, 1.2, 1.0, 0.6, 0.1, 0.1,
 0.0, 0.20, 4.0, {}, -0.5, 0.0, {}, 0.0, 0.0, 0.0, 1.0);
 CHECK(reverse_progress.best.command.valid);
 CHECK(reverse_progress.best.command.v < 0.0);

 // Static clearance is a soft preference after feasibility. Near an inflated
 // boundary, a stationary command may have slightly more terminal clearance
 // than a safe command that follows the reference path. That preference must
 // not dominate goal and path progress forever.
 sim::GridMap boundary_map(100, 100, 0.1);
 boundary_map.addBox(0.0, 0.0, 10.0, 0.65);
 sim::DwaReference boundary_reference;
 boundary_reference.path = {{1.0, 1.0}, {4.0, 1.0}};
 const auto boundary_progress = sim::dwa_compute_trajectory(
 {1.0, 1.0}, 0.0, 0.0, 0.0, {4.0, 1.0}, boundary_map, weights,
 {}, 0.5, 1.0, 2.5, 1.0, 0.1, 0.1,
 0.0, 0.20, 4.0, {}, 0.0, std::nullopt, boundary_reference);
 CHECK(boundary_progress.best.command.valid);
 CHECK(boundary_progress.best.command.v > 0.0);

 // With equivalent progress and feasibility, the same critic still prefers
 // the rollout that ends with more resolved static margin.
 sim::GridMap asymmetric_clearance(100, 100, 0.1);
asymmetric_clearance.addBox(0.2, 0.2, 9.8, 1.0);
 const auto clearance_rank = sim::dwa_compute_trajectory(
 {2.0, 1.05}, sim::kPi * 0.5, 0.0, 0.0, {2.0, 4.0}, asymmetric_clearance,
 sim::DwaWeights{0.0, 2.0, 0.0, 0.0, 0.0, 0.0},
 {}, 0.25, 1.0, 2.5, 1.0, 0.1, 0.1,
 0.0, 0.20, 4.0, {}, 0.0);
 const auto* stopped_at_margin = rollout_near(clearance_rank, 0.0, 0.0);
 const auto* moving_along_margin = rollout_near(clearance_rank, 0.25, 0.0);
 CHECK(stopped_at_margin && moving_along_margin);
 CHECK(moving_along_margin->trajectory.command.score >
 stopped_at_margin->trajectory.command.score);

 // Differential wheel kinematics are centred on the driven axle, while the
 // published base_link may be longitudinally offset. A pure wheel-space yaw
 // therefore moves base_link on a circle but leaves the axle point fixed.
 const auto offset_rollout = sim::dwa_compute_trajectory(
 {1.0, 1.0}, 0.0, 0.0, 1.0, {1.0, 1.0}, map, weights,
 {}, 0.0, 1.0, 4.0, 0.5, 0.1, 0.1,
 0.0, 0.20, 8.0, {}, 0.0, 0.5, {}, 0.0, 0.0, 0.0, 4.0, 0.12);
 const auto* pure_yaw = rollout_near(offset_rollout, 0.0, 1.0);
 CHECK(pure_yaw != nullptr);
 const auto axle_point = [](const sim::TrajectorySample& sample) {
 return sim::Vec2{
 sample.position.x + 0.12 * std::cos(sample.heading_rad),
 sample.position.y + 0.12 * std::sin(sample.heading_rad),
 };
 };
 const auto initial_axle = axle_point(pure_yaw->trajectory.samples.front());
 const auto terminal_axle = axle_point(pure_yaw->trajectory.samples.back());
 CHECK(std::hypot(terminal_axle.x - initial_axle.x,
 terminal_axle.y - initial_axle.y) < 0.01);
 CHECK(std::abs(pure_yaw->trajectory.samples.back().lateral_mps + 0.12) < 1e-12);


 sim::AckermannLocalConfig ackermann_stop;
 ackermann_stop.max_speed_mps = 0.0;
 ackermann_stop.max_accel_mps2 = 2.0;
 ackermann_stop.max_steer_rad = 0.5;
 ackermann_stop.predict_time_s = 1.5;
 ackermann_stop.dt_s = 0.1;
 const auto staged_ackermann_stop = sim::ackermann_local_compute(
 {1.0, 1.0}, 0.0, 1.0, 0.0, 0.0, {1.0, 1.0}, map, weights,
 {}, ackermann_stop);
 CHECK(staged_ackermann_stop.best.command.valid);
 for (const auto& rollout : staged_ackermann_stop.rollouts) {
 CHECK(std::abs(rollout.trajectory.command.v - 1.0) <= 2.0 * 0.1 + 1e-9);
 CHECK(rollout.trajectory.command.v >= 0.8 - 1e-9);
 }

 sim::HolonomicLocalConfig holonomic_stop;
 holonomic_stop.max_speed_mps = 0.8;
 holonomic_stop.max_accel_mps2 = 2.0;
 holonomic_stop.max_yaw_radps = 1.5;
 holonomic_stop.max_yaw_accel_radps2 = 3.0;
 holonomic_stop.predict_time_s = 1.0;
 holonomic_stop.dt_s = 0.1;
 holonomic_stop.terminal_position = sim::Vec2{1.0, 1.0};
 const auto staged_holonomic_stop = sim::holonomic_local_compute(
 {1.0, 1.0}, 0.0, 0.8, 0.6, 0.7, {1.0, 1.0}, map, weights,
 {}, holonomic_stop);
 CHECK(staged_holonomic_stop.best.command.valid);
 for (const auto& rollout : staged_holonomic_stop.rollouts) {
 const double planar_delta = std::hypot(
 rollout.trajectory.command.v - 0.8,
 rollout.trajectory.command.lateral_v - 0.6);
 CHECK(planar_delta <= 2.0 * 0.1 + 1e-9);
 CHECK(std::abs(rollout.trajectory.command.w - 0.7) <= 3.0 * 0.1 + 1e-9);
 }

 // Holonomic terminal planning is a pose-settling problem, not another
 // forward-only route sample. On either side of the same destination the
 // selected correction must change sign, remain slow, and avoid the bearing
 // flip that used to command a needless half-turn after overshoot.
 sim::HolonomicLocalConfig terminal_config;
 terminal_config.max_speed_mps = 0.8;
 terminal_config.max_yaw_radps = 1.5;
 terminal_config.max_accel_mps2 = 2.0;
 terminal_config.max_yaw_accel_radps2 = 4.0;
 terminal_config.predict_time_s = 0.8;
 terminal_config.dt_s = 0.1;
 terminal_config.terminal_position = sim::Vec2{5.0, 5.0};
 terminal_config.terminal_position_tolerance_m = 0.10;
 terminal_config.terminal_max_speed_mps = 0.05;
 const auto approach_terminal = sim::holonomic_local_compute(
 {4.82, 5.0}, 0.0, 0.10, 0.0, 0.0, {5.0, 5.0}, map, weights,
 {}, terminal_config);
 CHECK(approach_terminal.best.command.valid);
 CHECK(approach_terminal.best.command.v >= 0.0);
 CHECK(approach_terminal.best.command.v < 0.35);
 CHECK(std::abs(approach_terminal.best.command.w) < 1e-12);
 const auto correct_overshoot = sim::holonomic_local_compute(
 {5.18, 5.0}, 0.0, 0.10, 0.0, 0.0, {5.0, 5.0}, map, weights,
 {}, terminal_config);
 CHECK(correct_overshoot.best.command.valid);
 CHECK(correct_overshoot.best.command.v < 0.0);
 CHECK(std::abs(correct_overshoot.best.command.v) < 0.35);
 CHECK(std::abs(correct_overshoot.best.command.w) < 1e-12);
 auto pose_terminal_config = terminal_config;
 pose_terminal_config.terminal_heading_rad = sim::kPi * 0.5;
 const auto align_terminal = sim::holonomic_local_compute(
 {5.0, 5.0}, 0.0, 0.0, 0.0, 0.0, {5.0, 5.0}, map, weights,
 {}, pose_terminal_config);
 CHECK(align_terminal.best.command.valid);
 CHECK(align_terminal.best.command.w > 0.0);
 CHECK(std::abs(sim::wrap_angle(
 sim::kPi * 0.5 - align_terminal.best.samples.back().heading_rad)) <
 sim::kPi * 0.5);

 // A small residual just outside the arrival tolerance still requires a
 // real alignment command. This is the terminal condition that previously
 // left RO-4 stopped forever with a valid position but a 0.20 rad yaw error.
 auto fine_align_config = terminal_config;
 fine_align_config.terminal_heading_rad = 0.20;
 fine_align_config.terminal_heading_tolerance_rad = 0.12;
 const auto fine_align_terminal = sim::holonomic_local_compute(
 {5.0, 5.0}, 0.0, 0.0, 0.0, 0.0, {5.0, 5.0}, map, weights,
 {}, fine_align_config);
 CHECK(fine_align_terminal.best.command.valid);
 CHECK(fine_align_terminal.best.command.w > 0.0);

 // Exact-time interpolation is mandatory. The target crosses the stationary
 // ego between two prediction frames; endpoint-only or floor-index checking
 // would incorrectly publish a valid command.
 const std::vector<std::vector<sim::DynamicObjectState>> interframe_crossing{
 {{{0.0, 1.0}, {2.0, 0.0}, 0.10}},
 {{{2.0, 1.0}, {2.0, 0.0}, 0.10}},
 };
 const auto interframe = sim::dwa_compute_trajectory(
 {1.0, 1.0}, 0.0, 0.0, 0.0, {1.0, 1.0}, map, weights,
 interframe_crossing, 0.0, 0.0, 1.0, 0.5, 0.5, 1.0,
 0.0, 0.10, 1.0, {}, 0.0);
 CHECK(!interframe.rollouts.empty());
 CHECK(interframe.rollouts.front().collision);
 CHECK(!interframe.best.command.valid);

 // A crossing actor blocks the fast straight rollout, while a dynamically
 // safe decelerating candidate remains selectable. Safety arbitration checks
 // all candidates before score ordering.
 const std::vector<std::vector<sim::DynamicObjectState>> crossing_actor{
 {{{1.95, 0.0}, {0.0, 2.0}, 0.20}},
 {{{1.95, 1.0}, {0.0, 2.0}, 0.20}},
 {{{1.95, 2.0}, {0.0, 2.0}, 0.20}},
 };
 const auto crossing_plan = sim::dwa_compute_trajectory(
 {1.0, 1.0}, 0.0, 1.0, 0.0, {4.0, 1.0}, map, weights,
 crossing_actor, 1.0, 1.5, 3.0, 1.0, 0.1, 0.5,
 0.10, 0.20, 8.0, {}, 0.0);
 const auto* straight_fast = rollout_near(crossing_plan, 1.0, 0.0);
 CHECK(straight_fast && !straight_fast->collision);
 CHECK(sim::validate_executable_trajectory(
 straight_fast->trajectory, map, crossing_actor, 0.5, 0.20, 0.10).verdict ==
 sim::TrajectoryVerdict::DynamicCollision);
 const auto crossing_selection = sim::select_executable_trajectory(
 crossing_plan, map, crossing_actor, 0.5, 0.20, 0.10);
 CHECK(crossing_selection.validation.accepted());
 CHECK(crossing_selection.accepted_candidates > 0);
 CHECK(crossing_selection.validation.min_dynamic_clearance_m + 1e-9 >= 0.10);

 // A currently clear hold can become unsafe later in the maintained
 // forecast. Pose recovery must inspect the full time-indexed hold, rather
 // than hard-locking a zero-speed alignment from frame zero alone.
 const auto crossing_hold = sim::minimum_stationary_clearance(
 {1.0, 1.0}, crossing_actor, 0.5, 0.20);
 CHECK(crossing_hold.physical_m > -1e-9);
CHECK(crossing_hold.conservative_m < 0.60);

 // A reversal inside the horizon remains time-indexed; it cannot be replaced
 // by constant-velocity extrapolation from the first frame.
 const std::vector<std::vector<sim::DynamicObjectState>> reversing_actor{
 {{{1.7, 0.0}, {0.0, 2.0}, 0.20}},
 {{{1.7, 2.0}, {0.0, -2.0}, 0.20}},
 {{{1.7, 0.0}, {0.0, -2.0}, 0.20}},
 };
 const auto reversing_plan = sim::dwa_compute_trajectory(
 {1.0, 1.0}, 0.0, 1.0, 0.0, {4.0, 1.0}, map, weights,
 reversing_actor, 1.0, 1.5, 3.0, 2.0, 0.1, 1.0,
 0.10, 0.20, 8.0, {}, 0.0);
 const auto* reversing_straight = rollout_near(reversing_plan, 1.0, 0.0);
 CHECK(reversing_straight);
 CHECK(!sim::validate_executable_trajectory(
 reversing_straight->trajectory, map, reversing_actor, 1.0, 0.20, 0.10).accepted());

 // Sudden target slowdown is represented by positions at each future time,
 // rather than trusting the first measured velocity forever. The stopped
 // target occupies the ego route later in the horizon, so continuing at the
 // current speed is inadmissible.
 const std::vector<std::vector<sim::DynamicObjectState>> slowing_actor{
 {{{2.0, 1.0}, {1.0, 0.0}, 0.20}},
 {{{2.5, 1.0}, {0.0, 0.0}, 0.20}},
 {{{2.5, 1.0}, {0.0, 0.0}, 0.20}},
 {{{2.5, 1.0}, {0.0, 0.0}, 0.20}},
 {{{2.5, 1.0}, {0.0, 0.0}, 0.20}},
 };
 const auto slowing_plan = sim::dwa_compute_trajectory(
 {1.0, 1.0}, 0.0, 1.0, 0.0, {5.0, 1.0}, map, weights,
 slowing_actor, 1.0, 1.5, 3.0, 2.0, 0.1, 0.5,
 0.10, 0.20, 8.0, {}, 0.0);
 const auto* slowing_straight = rollout_near(slowing_plan, 1.0, 0.0);
 CHECK(slowing_straight);
 CHECK(!sim::validate_executable_trajectory(
 slowing_straight->trajectory, map, slowing_actor, 0.5, 0.20, 0.10).accepted());

 // Waiting is itself a trajectory. If an actor enters the hold position in
 // the future, the zero command is unsafe even though the position is clear
 // now; DWA must seek an escape or report no admissible candidate.
 const std::vector<std::vector<sim::DynamicObjectState>> invading_wait{
 {{{3.0, 1.0}, {-2.0, 0.0}, 0.20}},
 {{{2.0, 1.0}, {-2.0, 0.0}, 0.20}},
 {{{1.0, 1.0}, {-2.0, 0.0}, 0.20}},
 };
 const auto wait_plan = sim::dwa_compute_trajectory(
 {1.0, 1.0}, 0.0, 0.0, 0.0, {1.0, 1.0}, map, weights,
 invading_wait, 1.0, 1.5, 3.0, 1.0, 0.1, 0.5,
 0.10, 0.20, 8.0, {}, 0.0);
 const auto* stationary_wait = rollout_near(wait_plan, 0.0, 0.0);
 CHECK(stationary_wait && stationary_wait->collision);

 // Prediction uncertainty is separate from physical geometry. The sampler
 // may generate a physically non-overlapping hold, but the independent hard
 // validator rejects it when the uncertainty envelope violates the margin.
 const std::vector<std::vector<sim::DynamicObjectState>> uncertain_actor(
 2, {{{1.0, 1.5}, {}, 0.10, 0.30}});
 const auto uncertain_plan = sim::dwa_compute_trajectory(
 {1.0, 1.0}, 0.0, 0.0, 0.0, {1.0, 1.0}, map, weights,
 uncertain_actor, 0.0, 0.0, 1.0, 0.5, 0.5, 0.5,
 0.10, 0.10, 1.0, {}, 0.0);
 CHECK(uncertain_plan.best.command.valid);
 const auto uncertain_selection = sim::select_executable_trajectory(
 uncertain_plan, map, uncertain_actor, 0.5, 0.10, 0.10);
 CHECK(!uncertain_selection.validation.accepted());
 CHECK(uncertain_selection.validation.verdict == sim::TrajectoryVerdict::DynamicCollision);

 // Physical overlap has no admissible rollout. A planner failure must remain
 // invalid so the caller enters its zero-command safety state; score cannot
 // resurrect a colliding candidate.
 const std::vector<std::vector<sim::DynamicObjectState>> overlap(
 6, {{{1.0, 1.0}, {}, 0.30}});
 const auto trapped = sim::dwa_compute_trajectory(
 {1.0, 1.0}, 0.0, 0.0, 0.0, {3.0, 1.0}, map, weights,
 overlap, 1.0, 1.5, 2.0, 0.5, 0.1, 0.1,
 0.10, 0.20, 8.0, {}, 0.0);
 CHECK(!trapped.best.command.valid);
 CHECK(std::all_of(trapped.rollouts.begin(), trapped.rollouts.end(),
 [](const auto& rollout) { return rollout.collision; }));

 // Pose-constrained stops use the explicit body heading. A strong ordinary
 // route-tangent critic must not keep a differential robot frozen at the
 // correct position but wrong yaw.
 sim::DwaReference stop_reference;
 stop_reference.path = {{2.0, 2.0}, {4.0, 2.0}};
 const sim::DwaWeights path_dominant{1.0, 2.0, 0.0, 10.0, 0.0, 0.0};
 const auto align = sim::dwa_compute_trajectory(
 {2.0, 2.0}, 0.0, 0.0, 0.0, {2.0, 2.0}, map, path_dominant,
 {}, 0.0, 2.0, 2.0, 1.0, 0.1, 0.1,
 0.0, 0.20, 8.0, {}, 0.0, sim::kPi * 0.5, stop_reference);
 CHECK(align.best.command.valid);
 CHECK(align.best.command.w > 0.0);
 CHECK(std::abs(sim::wrap_angle(
 sim::kPi * 0.5 - align.best.samples.back().heading_rad)) < sim::kPi * 0.5);

 // Differential candidates must be feasible in wheel space. A body command
 // can satisfy independent v/w limits while asking one wheel to exceed the
 // selected motor speed; such a rollout must never reach validation/control.
 constexpr double track_m = 0.6;
 constexpr double wheel_limit_mps = 1.0;
 const auto wheel_limited = sim::dwa_compute_trajectory(
 {2.0, 2.0}, 0.0, 0.0, 0.0, {8.0, 2.0}, map, weights,
 {}, 2.0, 2.0, 3.0, 1.0, 0.1, 0.1,
 0.0, 0.20, 8.0, {}, 0.0, std::nullopt, {}, track_m, wheel_limit_mps);
 CHECK(!wheel_limited.rollouts.empty());
 for (const auto& rollout : wheel_limited.rollouts) {
 const double v = rollout.trajectory.command.v;
 const double w = rollout.trajectory.command.w;
 CHECK(std::abs(v - w * track_m * 0.5) <= wheel_limit_mps + 1e-9);
 CHECK(std::abs(v + w * track_m * 0.5) <= wheel_limit_mps + 1e-9);
 }

 // Identical inputs produce identical candidate order, commands and scores.
 const auto repeated = sim::dwa_compute_trajectory(
 {1.0, 1.0}, 0.0, 1.0, 0.0, {4.0, 1.0}, map, weights,
 crossing_actor, 1.0, 1.5, 3.0, 1.0, 0.1, 0.5,
 0.10, 0.20, 8.0, {}, 0.0);
 CHECK(repeated.rollouts.size() == crossing_plan.rollouts.size());
 for (std::size_t index = 0; index < repeated.rollouts.size(); ++index) {
 const auto& first = crossing_plan.rollouts[index];
 const auto& second = repeated.rollouts[index];
 CHECK(first.collision == second.collision);
 CHECK(first.trajectory.command.v == second.trajectory.command.v);
 CHECK(first.trajectory.command.w == second.trajectory.command.w);
 CHECK(first.trajectory.command.score == second.trajectory.command.score);
 }

 std::cout << "dynamic DWA contracts green\n";
 return 0;
}
