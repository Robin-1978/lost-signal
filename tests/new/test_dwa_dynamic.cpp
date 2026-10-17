#include "autonomy/planning/local/dwa.hpp"
#include "autonomy/planning/validation/trajectory_validator.hpp"

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
 {{{1.7, 0.0}, {0.0, 2.0}, 0.20}},
 {{{1.7, 1.0}, {0.0, 2.0}, 0.20}},
 {{{1.7, 2.0}, {0.0, 2.0}, 0.20}},
 };
 const auto crossing_plan = sim::dwa_compute_trajectory(
 {1.0, 1.0}, 0.0, 1.0, 0.0, {4.0, 1.0}, map, weights,
 crossing_actor, 1.0, 1.5, 3.0, 1.0, 0.1, 0.5,
 0.10, 0.20, 8.0, {}, 0.0);
 const auto* straight_fast = rollout_near(crossing_plan, 1.0, 0.0);
 CHECK(straight_fast && straight_fast->collision);
 const auto crossing_selection = sim::select_executable_trajectory(
 crossing_plan, map, crossing_actor, 0.5, 0.20, 0.10);
 CHECK(crossing_selection.validation.accepted());
 CHECK(crossing_selection.accepted_candidates > 0);
 CHECK(crossing_selection.validation.min_dynamic_clearance_m + 1e-9 >= 0.10);

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
 CHECK(reversing_straight && reversing_straight->collision);

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
 CHECK(slowing_straight && slowing_straight->collision);

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
