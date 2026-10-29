#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <deque>
#include <limits>

#include "robot/autonomy/control/controller.hpp"
#include "robot/autonomy/decision/motion_pipeline.hpp"

namespace {

using namespace lost_signal::robot;
using namespace lost_signal::robot::autonomy;

class ClearBehavior final : public IBehaviorPolicy {
 public:
 void reset() override {}
 BehaviorDecision decide(const EstimatedState&, const PerceptionResult&,
 const PredictionBatch&, const Route&,
 const platform::ChassisLimits& limits) override {
 return {false, limits.max_forward_mps, "clear"};
 }
};

class OneShotConflictBehavior final : public IBehaviorPolicy {
 public:
 void reset() override { calls_ = 0; }
 BehaviorDecision decide(const EstimatedState&, const PerceptionResult&,
 const PredictionBatch&, const Route&,
 const platform::ChassisLimits& limits) override {
 ++calls_;
 return {false, limits.max_forward_mps,
 calls_ == 1 ? "route-conflict-no-safe-hold" : "clear"};
 }

 private:
 int calls_ = 0;
};

class HoldBehavior final : public IBehaviorPolicy {
 public:
 void reset() override {}
 BehaviorDecision decide(const EstimatedState&, const PerceptionResult&,
 const PredictionBatch&, const Route&,
 const platform::ChassisLimits&) override {
 return {true, 0.0, "track-conflict-hold"};
 }
};

class RankedPlanner final : public ILocalPlanner {
 public:
 void reset() override {}
 LocalPlan plan(const EstimatedState&, const PerceptionResult&,
 const PredictionBatch&, const LocalMap&, const Route&,
 const platform::ChassisLimits&) override {
 LocalPlan result;
 result.has_plan = true;
 result.local_costmap.resolution_m = 0.1;
 result.local_costmap.width = 1;
 result.local_costmap.height = 1;
 result.local_costmap.cells = {0};
 LocalCandidate fast;
 fast.feasible = true;
 fast.forward_mps = 1.0;
 LocalCandidate safe = fast;
 safe.forward_mps = 0.2;
 result.best = fast;
 result.candidates = {fast, safe};
 return result;
 }
};

class EmptyPlanner final : public ILocalPlanner {
 public:
 void reset() override {}
 LocalPlan plan(const EstimatedState&, const PerceptionResult&,
 const PredictionBatch&, const LocalMap&, const Route&,
 const platform::ChassisLimits&) override {
 return {};
 }
};

class HoldPlanner final : public ILocalPlanner {
 public:
 void reset() override {}
 LocalPlan plan(const EstimatedState&, const PerceptionResult&,
 const PredictionBatch&, const LocalMap&, const Route&,
 const platform::ChassisLimits&) override {
 LocalPlan result;
 result.has_plan = true;
 result.best.feasible = true;
 result.candidates.push_back(result.best);
 return result;
 }
};

class LimitEchoPlanner final : public ILocalPlanner {
 public:
 void reset() override {}
 LocalPlan plan(const EstimatedState&, const PerceptionResult&,
 const PredictionBatch&, const LocalMap&, const Route&,
 const platform::ChassisLimits& limits) override {
 last_limits = limits;
 LocalPlan result;
 result.has_plan = true;
 result.best.feasible = true;
 result.best.forward_mps = limits.max_forward_mps;
 result.candidates.push_back(result.best);
 return result;
 }
 platform::ChassisLimits last_limits;
};

class SwitchablePlanner final : public ILocalPlanner {
 public:
 void reset() override {}
 LocalPlan plan(const EstimatedState&, const PerceptionResult&,
 const PredictionBatch&, const LocalMap&, const Route&,
 const platform::ChassisLimits&) override {
 if (!available) return {};
 LocalPlan result;
 result.has_plan = true;
 result.best.feasible = true;
 result.best.yaw_rate_radps = -0.2;
 result.candidates.push_back(result.best);
 return result;
 }
 bool available = false;
};

class TurnOnlyValidator final : public IValidator {
 public:
 ValidationResult validate(const LocalCandidate& candidate,
 const EstimatedState&, const PerceptionResult&,
 const PredictionBatch&, const iface::ObservationFrame&) override {
 ValidationResult result;
 if (!candidate.feasible || std::abs(candidate.yaw_rate_radps) < 0.01) {
 result.reason = "turn required";
 return result;
 }
 result.accepted = true;
 result.verdict = TrajectoryVerdict::kAccepted;
 result.min_clearance_m = 1.0;
 result.min_dynamic_clearance_m = 1.0;
 result.reason = "clear";
 return result;
 }
};

class SpeedValidator final : public IValidator {
 public:
 ValidationResult validate(const LocalCandidate& candidate,
 const EstimatedState&, const PerceptionResult&,
 const PredictionBatch&, const iface::ObservationFrame&) override {
 ValidationResult result;
 if (!candidate.feasible) {
 result.reason = "infeasible";
 return result;
 }
 if (candidate.forward_mps > 0.5) {
 result.verdict = TrajectoryVerdict::kUnsafeEmergencyStop;
 result.reason = "braking tail rejected";
 return result;
 }
 result.accepted = true;
 result.verdict = TrajectoryVerdict::kAccepted;
 result.min_dynamic_clearance_m = 1.0;
 result.swept_envelope_radius_m = 0.5;
 result.swept_centerline = {{0.0, 0.0}, {candidate.forward_mps, 0.0}};
 result.reason = "clear";
 return result;
 }
};

class DynamicClearanceValidator final : public IValidator {
 public:
 ValidationResult validate(const LocalCandidate& candidate,
 const EstimatedState&, const PerceptionResult&,
 const PredictionBatch&, const iface::ObservationFrame&) override {
 ValidationResult result;
 if (!candidate.feasible) return result;
 result.accepted = true;
 result.verdict = TrajectoryVerdict::kAccepted;
 result.min_clearance_m = 1.0;
 result.min_dynamic_clearance_m = candidate.forward_mps < -1e-9
 ? 0.8
 : candidate.forward_mps > 1e-9 ? 0.1 : 0.3;
 result.reason = "clear";
 return result;
 }
};

// A mover that crosses the robot's path and does not yield. Holding still and
// every motion that does not leave the lane meet it; a motion in an allowed
// direction leaves the lane in time only at `clear_speed_mps` or faster, and a
// slower one is the minimum-risk creep the real validator also admits.
// `fastest_validated_mps` records the fastest candidate the pipeline asked
// about, which is how the tests see whether the reachable-speed set was used.
class CrossingValidator final : public IValidator {
 public:
 double clear_speed_mps = 1.0;
 bool reverse_clears = true;
 bool lateral_clears = false;
 bool hold_is_safe = false;
 bool slow_motion_is_minimum_risk = true;
 double fastest_validated_mps = 0.0;

 ValidationResult validate(const LocalCandidate& candidate,
 const EstimatedState&, const PerceptionResult&,
 const PredictionBatch&, const iface::ObservationFrame&) override {
 ValidationResult result;
 if (!candidate.feasible) {
 result.reason = "infeasible";
 return result;
 }
 const double speed =
 std::hypot(candidate.forward_mps, candidate.lateral_mps);
 fastest_validated_mps = std::max(fastest_validated_mps, speed);
 if (speed < 1e-9 && hold_is_safe) {
 result.accepted = true;
 result.verdict = TrajectoryVerdict::kAccepted;
 result.min_dynamic_clearance_m = 1.0;
 result.reason = "clear";
 return result;
 }
 const bool reverses = reverse_clears && candidate.forward_mps < 0.0 &&
 candidate.lateral_mps == 0.0;
 const bool slides = lateral_clears && candidate.lateral_mps != 0.0 &&
 candidate.forward_mps == 0.0;
 if (speed < 1e-9 || !(reverses || slides)) {
 result.verdict = TrajectoryVerdict::kDynamicCollision;
 result.min_dynamic_clearance_m = -0.2;
 result.reason = "dynamic collision";
 return result;
 }
 if (speed >= clear_speed_mps) {
 result.accepted = true;
 result.verdict = TrajectoryVerdict::kAccepted;
 result.min_dynamic_clearance_m = 0.5;
 result.reason = "clear";
 return result;
 }
 if (!slow_motion_is_minimum_risk) {
 result.verdict = TrajectoryVerdict::kDynamicCollision;
 result.min_dynamic_clearance_m = -0.1;
 result.reason = "dynamic collision";
 return result;
 }
 result.accepted = true;
 result.verdict = TrajectoryVerdict::kMinimumRiskEscape;
 result.min_dynamic_clearance_m = -0.05;
 result.escape_clearance_gain_m = 0.1;
 result.reason = "minimum-risk";
 return result;
 }
};

class PersistentConflictBehavior final : public IBehaviorPolicy {
 public:
 void reset() override {}
 BehaviorDecision decide(const EstimatedState&, const PerceptionResult&,
 const PredictionBatch&, const Route&,
 const platform::ChassisLimits& limits) override {
 return {false, limits.max_forward_mps, "route-conflict-no-safe-hold"};
 }
};

// Closed loop through the real prediction, clearance validator and controller.
// The robot sits inside the declared observation envelope in the lane of a
// mover that does not yield, and the plant honours the same declared
// actuator contract the validator replays: response delay, then slew at the
// declared acceleration or braking. Returns the closest approach between the
// robot and mover discs (negative means contact) and the fastest speed the
// robot was driven at.
struct LaneCrossing {
 double closest_approach_m = std::numeric_limits<double>::infinity();
 double fastest_mps = 0.0;
};

LaneCrossing run_lane_crossing(bool mover_present) {
 constexpr double kPeriodS = 0.02;
 // A mover becomes a usable track only once it is confirmed, which in the
 // canonical mission is about 4.5 m away.
 constexpr double kConfirmationRangeM = 4.5;
 constexpr double kMoverSpeedMps = 1.4;
 constexpr double kMoverRadiusM = 0.7;
 const platform::MotionDynamicsPolicy motion;
 platform::ChassisLimits limits{.max_forward_mps = 2.0,
 .max_yaw_rate_radps = 1.5,
 .footprint_radius_m = 0.4};

 MotionDecisionPipeline::Config config;
 config.motion = motion;
 config.decision_period_s = kPeriodS;
 MotionDecisionPipeline pipeline(config);
 ClearanceValidatorConfig validator_config;
 validator_config.footprint_radius_m = limits.footprint_radius_m;
 validator_config.extra_margin_m = platform::kDefaultNavigationClearanceMarginM;
 validator_config.acceleration_mps2 = motion.acceleration_mps2;
 validator_config.brake_decel_mps2 = motion.brake_deceleration_mps2;
 validator_config.lateral_acceleration_mps2 = motion.lateral_acceleration_mps2;
 validator_config.yaw_acceleration_radps2 = motion.yaw_acceleration_radps2;
 validator_config.actuator_response_delay_s = motion.actuator_response_delay_s;
 ClearanceValidator validator(validator_config);
 ClearBehavior behavior;
 LimitEchoPlanner planner;
 DirectController controller;
 SurveyContext survey;

 LocalMap map;
 map.valid = true;
 map.provenance = LocalMapProvenance::kSensorRays;
 Route route;
 route.valid = true;
 route.points = {{0.0, -2.0, 0.0, 1}, {0.0, 20.0, 0.0, 1}};

 // The robot crawls along +y at the observation envelope; the mover runs
 // along -x through the robot's column.
 double robot_y = -1.4;
 double robot_v = config.recovery_speed_mps;
 double mover_x = 12.0;
 const std::size_t delay_steps = static_cast<std::size_t>(std::llround(
 motion.actuator_response_delay_s / kPeriodS));
 std::deque<double> pending(delay_steps, robot_v);
 LaneCrossing outcome;
 const double contact_distance_m =
 limits.footprint_radius_m + kMoverRadiusM;
 for (int step = 0; step < 1000 && mover_x > -4.0; ++step) {
 const core::MonotonicNs now_ns = 1'000'000'000 +
 static_cast<core::MonotonicNs>(step) * 20'000'000;
 EstimatedState estimate;
 estimate.valid = true;
 estimate.stamp_ns = now_ns;
 estimate.y_m = robot_y;
 estimate.yaw_rad = 0.5 * core::kPi;
 estimate.forward_mps = robot_v;
 PerceptionResult perception;
 perception.scan_ok = true;
 perception.stamp_ns = now_ns;
 if (mover_present && mover_x <= kConfirmationRangeM) {
 DynamicTrack mover;
 mover.id = 1;
 mover.x_m = mover_x;
 mover.vx_mps = -kMoverSpeedMps;
 mover.radius_m = kMoverRadiusM;
 mover.age_s = 1.0;
 mover.prediction_ready = true;
 mover.last_observed_ns = now_ns;
 perception.tracks.push_back(mover);
 }
 iface::ObservationFrame observation;
 observation.sequence = static_cast<std::uint64_t>(step + 1);
 observation.stamp_ns = now_ns;
 observation.frame = core::Frame::kLidarLink;
 observation.odometry_valid = true;
 observation.scan_valid = true;
 observation.scan_angle_min_rad = -core::kPi;
 observation.scan_angle_increment_rad = 2.0 * core::kPi / 32.0;
 observation.ranges_m.assign(32, 10.0f);
 observation.static_ranges_m = observation.ranges_m;
 observation.scan_static_valid = true;

 const MotionDecisionResult decision = pipeline.run(
 estimate, perception, map, route, limits, limits, observation, survey,
 now_ns, behavior, planner, validator, controller, false, true);
 pending.push_back(decision.command.forward_mps);
 const double target = pending.front();
 pending.pop_front();
 const bool slowing = std::abs(target) < std::abs(robot_v) ||
 target * robot_v < 0.0;
 const double slew_mps2 = slowing ? motion.brake_deceleration_mps2
 : motion.acceleration_mps2;
 robot_v += std::clamp(target - robot_v, -slew_mps2 * kPeriodS,
 slew_mps2 * kPeriodS);
 robot_y += robot_v * kPeriodS;
 mover_x -= kMoverSpeedMps * kPeriodS;
 outcome.fastest_mps = std::max(outcome.fastest_mps, std::abs(robot_v));
 if (mover_present) {
 outcome.closest_approach_m = std::min(
 outcome.closest_approach_m,
 std::hypot(mover_x, robot_y) - contact_distance_m);
 }
 }
 return outcome;
}

} // namespace

int main() {
 using namespace lost_signal::robot;
 using namespace lost_signal::robot::autonomy;

 EstimatedState estimate;
 estimate.valid = true;
 estimate.stamp_ns = 1'000'000'000;
 PerceptionResult perception;
 perception.scan_ok = true;
 perception.stamp_ns = estimate.stamp_ns;
 LocalMap map;
 map.valid = true;
 map.provenance = LocalMapProvenance::kSensorRays;
 Route route;
 route.valid = true;
 route.points = {{0.0, 0.0, 0.0, 1}, {2.0, 0.0, 0.0, 1}};
 platform::ChassisLimits limits{.max_forward_mps = 2.0,
 .max_yaw_rate_radps = 1.5,
 .footprint_radius_m = 0.4};
 iface::ObservationFrame observation;
 observation.scan_valid = true;
 observation.ranges_m.assign(16, 10.0f);
 SurveyContext survey;
 ClearBehavior behavior;
 SpeedValidator validator;
 DirectController controller;

 // A rejected highest-scoring proposal must fall through to the next
 // planner-ranked candidate without producing a stop or recovery pulse.
 MotionDecisionPipeline pipeline;
 RankedPlanner ranked;
 const MotionDecisionResult selected = pipeline.run(
 estimate, perception, map, route, limits, limits, observation, survey,
 estimate.stamp_ns, behavior, ranked, validator, controller);
 assert(!selected.command.is_safe_stop());
 assert(selected.command.forward_mps == 0.2);
 assert(selected.reason == "clear");
 assert(selected.local_costmap.valid());
 assert(selected.validated_swept_envelope_radius_m == 0.5);
 assert(selected.validated_swept_centerline.size() == 2);
 assert(selected.validated_swept_centerline.back().x == 0.2);

 // A true empty proposal set on a valid route invokes bounded recovery.
 // This is distinct from the no-goal case, which remains a hold.
 pipeline.reset();
 EmptyPlanner empty;
 const MotionDecisionResult recovered = pipeline.run(
 estimate, perception, map, route, limits, limits, observation, survey,
 estimate.stamp_ns, behavior, empty, validator, controller);
 assert(!recovered.command.is_safe_stop());
 assert(recovered.reason.find("recovery:") == 0);

 // A planner hold is also a stalled proposal when a translational goal is
 // still farther away than the declared footprint. Treating an accepted
 // zero command as progress created a permanent local minimum in front of
 // newly mapped obstacles. Recovery candidates remain validator-gated.
 pipeline.reset();
 HoldPlanner stalled_hold;
 const MotionDecisionResult escaped_hold = pipeline.run(
 estimate, perception, map, route, limits, limits, observation, survey,
 estimate.stamp_ns, behavior, stalled_hold, validator, controller);
 assert(!escaped_hold.command.is_safe_stop());
 assert(escaped_hold.reason.find("recovery:") == 0);

 // Inside the physical terminal envelope, a hold may be the correct next
 // command while arrival settles; recovery must not disturb it.
 Route near_terminal = route;
 near_terminal.points.back().x_m = 0.2;
 pipeline.reset();
 const MotionDecisionResult terminal_hold = pipeline.run(
 estimate, perception, map, near_terminal, limits, limits, observation,
 survey, estimate.stamp_ns, behavior, stalled_hold, validator, controller);
 assert(terminal_hold.command.is_safe_stop());

 // Global map uncertainty must not disable a recovery that is validated
 // entirely from current local sensor evidence. A robot waiting for global
 // relocalization can otherwise deadlock: it cannot move to reveal geometry,
 // while every ordinary planner candidate remains blocked. The recovery is
 // still crawl-speed and passes the independent validator before publication.
 EstimatedState globally_uncertain = estimate;
 globally_uncertain.position_sigma_m = 1.5;
 globally_uncertain.heading_sigma_rad = 0.4;
 globally_uncertain.confidence = 0.35;
 pipeline.reset();
 const MotionDecisionResult uncertain_recovery = pipeline.run(
 globally_uncertain, perception, map, route, limits, limits, observation,
 survey, estimate.stamp_ns, behavior, empty, validator, controller);
 assert(!uncertain_recovery.command.is_safe_stop());
 assert(uncertain_recovery.reason.find("recovery:") == 0);

 // If a tracked object collision-filters every moving local-plan rollout,
 // accepted recovery commands are ordered by increasing predicted dynamic
 // separation. Replanning the unchanged survey or creeping back toward the
 // route would create a reciprocal stop/replan loop.
 PerceptionResult blocked_perception = perception;
 DynamicTrack blocking_track;
 blocking_track.id = 7;
 blocking_track.x_m = 0.8;
 blocking_track.radius_m = 0.3;
 blocking_track.last_observed_ns = estimate.stamp_ns;
 blocked_perception.tracks.push_back(blocking_track);
 DynamicClearanceValidator dynamic_clearance;
 pipeline.reset();
 const MotionDecisionResult clearance_recovery = pipeline.run(
 estimate, blocked_perception, map, route, limits, limits, observation,
 survey, estimate.stamp_ns, behavior, stalled_hold, dynamic_clearance,
 controller);
 assert(clearance_recovery.constraint == "dynamic_clearance");
 assert(clearance_recovery.command.forward_mps < 0.0);

 // A supervised localization search keeps ordinary route/obstacle planning,
 // but bounds even a valid moving plan to the declared observation envelope.
 // If no moving plan exists, the validator-approved fallback crawl remains
 // available and respects Ackermann curvature.
 MotionDecisionPipeline::Config observation_config;
 observation_config.recovery_speed_mps = 0.25;
 observation_config.recovery_yaw_rate_radps = 0.6;
 MotionDecisionPipeline observation_pipeline(observation_config);
 LimitEchoPlanner echo_planner;
 const MotionDecisionResult bounded_observation = observation_pipeline.run(
 globally_uncertain, perception, map, route, limits, limits, observation,
 survey, estimate.stamp_ns, behavior, echo_planner, validator, controller,
 false, true);
 assert(!bounded_observation.command.is_safe_stop());
 assert(bounded_observation.command.forward_mps ==
 observation_config.recovery_speed_mps);
 assert(echo_planner.last_limits.max_forward_mps ==
 observation_config.recovery_speed_mps);
 assert(echo_planner.last_limits.max_lateral_mps == 0.0);
 assert(echo_planner.last_limits.max_yaw_rate_radps ==
 observation_config.recovery_yaw_rate_radps);

 pipeline.reset();
 HoldPlanner hold_planner;
 platform::ChassisLimits ackermann_limits = limits;
 ackermann_limits.max_curvature_inv_m = 1.2;
 const MotionDecisionResult observation_motion = pipeline.run(
 globally_uncertain, perception, map, route, ackermann_limits,
 ackermann_limits, observation, survey, estimate.stamp_ns, behavior,
 hold_planner, validator, controller, false, true);
 assert(!observation_motion.command.is_safe_stop());
 assert(observation_motion.constraint == "localization_observe");
 assert(std::abs(observation_motion.command.yaw_rate_radps) <=
 std::abs(observation_motion.command.forward_mps) *
 ackermann_limits.max_curvature_inv_m + 1e-9);

 Route no_route;
 pipeline.reset();
 const MotionDecisionResult held = pipeline.run(
 estimate, perception, map, no_route, limits, limits, observation, survey,
 estimate.stamp_ns, behavior, empty, validator, controller);
 assert(held.command.is_safe_stop());

 // Diagnostics must retain the behavior cause. Reporting only the hold
 // candidate's validator result hides whether contact, an immature track or
 // a predicted route conflict requested the stop.
 pipeline.reset();
 HoldBehavior hold_behavior;
 const MotionDecisionResult behavior_hold = pipeline.run(
 estimate, perception, map, route, limits, limits, observation, survey,
 estimate.stamp_ns, hold_behavior, ranked, validator, controller);
 assert(behavior_hold.command.is_safe_stop());
 assert(behavior_hold.reason == "track-conflict-hold");

 // Escape release is stateful but live: after the one-cycle threat clears,
 // consecutive safe-hold decisions release recovery. Continuing recovery
 // must not reset that counter.
 MotionDecisionPipeline escape_pipeline;
 OneShotConflictBehavior one_shot;
 for (int cycle = 0; cycle < 3; ++cycle) {
 const MotionDecisionResult escaping = escape_pipeline.run(
 estimate, perception, map, route, limits, limits, observation, survey,
 estimate.stamp_ns + cycle, one_shot, ranked, validator, controller);
 assert(escaping.reason.find("recovery:") == 0);
 }
 const MotionDecisionResult released = escape_pipeline.run(
 estimate, perception, map, route, limits, limits, observation, survey,
 estimate.stamp_ns + 3, one_shot, ranked, validator, controller);
 assert(released.reason == "clear");
 assert(released.command.forward_mps == 0.2);

 // Recovery samples use the declared motion envelope, and an ordinary plan
 // must remain available for one safety-reaction interval before it replaces
 // a still-valid manoeuvre. A one-frame planner/recovery alternation therefore
 // cannot publish opposite full-rate turns.
 MotionDecisionPipeline::Config continuity_config;
 continuity_config.decision_period_s = 0.02;
 continuity_config.motion.yaw_acceleration_radps2 = 8.0;
 continuity_config.motion.safety_reaction_time_s = 0.2;
 MotionDecisionPipeline continuity_pipeline(continuity_config);
 SwitchablePlanner switchable;
 TurnOnlyValidator turn_only;
 const MotionDecisionResult first_turn = continuity_pipeline.run(
 estimate, perception, map, route, limits, limits, observation, survey,
 estimate.stamp_ns, behavior, switchable, turn_only, controller);
 assert(first_turn.reason.find("recovery:") == 0);
 assert(first_turn.command.yaw_rate_radps > 0.0);
 assert(first_turn.command.yaw_rate_radps <=
 continuity_config.motion.yaw_acceleration_radps2 *
 continuity_config.decision_period_s + 1e-9);
 switchable.available = true;
 const MotionDecisionResult one_frame_plan = continuity_pipeline.run(
 estimate, perception, map, route, limits, limits, observation, survey,
 estimate.stamp_ns + 1, behavior, switchable, turn_only, controller);
 assert(one_frame_plan.reason.find("recovery:") == 0);
 assert(one_frame_plan.command.yaw_rate_radps > 0.0);
 switchable.available = false;
 const MotionDecisionResult dropout = continuity_pipeline.run(
 estimate, perception, map, route, limits, limits, observation, survey,
 estimate.stamp_ns + 2, behavior, switchable, turn_only, controller);
 assert(dropout.reason.find("recovery:") == 0);
 assert(dropout.command.yaw_rate_radps > 0.0);

 // Reachable-speed escape. From the observation crawl every window-limited
 // sample stays within one decision period of the measured twist. A mover
 // that arrives sooner than the crawl can leave its lane therefore leaves
 // only creeps that still meet it, and a stop is a predicted contact too.
 // The straight command at the declared chassis limit is then the only
 // escape; it is published verbatim, and only after the validator accepts
 // the whole rollout as clear.
 MotionDecisionPipeline::Config reach_config;
 reach_config.decision_period_s = 0.02;
 EstimatedState crawling = estimate;
 crawling.forward_mps = reach_config.recovery_speed_mps;
 const double window_step_mps = reach_config.motion.acceleration_mps2 *
 reach_config.decision_period_s;
 {
 MotionDecisionPipeline reach_pipeline(reach_config);
 CrossingValidator crossing;
 const MotionDecisionResult reach = reach_pipeline.run(
 crawling, perception, map, route, limits, limits, observation, survey,
 estimate.stamp_ns, behavior, hold_planner, crossing, controller);
 assert(!reach.command.is_safe_stop());
 assert(reach.constraint == "dynamic_escape");
 assert(reach.trajectory_verdict == TrajectoryVerdict::kAccepted);
 assert(reach.command.forward_mps == -limits.max_forward_mps);
 assert(reach.command.lateral_mps == 0.0);
 }

 // A window-limited sample that is fully clear keeps the escape inside the
 // window: the reachable-speed set is not even validated.
 {
 MotionDecisionPipeline windowed_pipeline(reach_config);
 CrossingValidator crossing;
 crossing.clear_speed_mps = 0.01;
 const MotionDecisionResult windowed = windowed_pipeline.run(
 estimate, perception, map, route, limits, limits, observation, survey,
 estimate.stamp_ns, behavior, hold_planner, crossing, controller);
 assert(windowed.command.forward_mps < 0.0);
 assert(std::abs(windowed.command.forward_mps) <= window_step_mps + 1e-9);
 assert(crossing.fastest_validated_mps <= window_step_mps + 1e-9);
 }

 // The minimum-risk exception admits creeps that still meet the track, so a
 // reachable-speed candidate is consulted but the creep is not displaced
 // unless the candidate is fully clear.
 {
 MotionDecisionPipeline creeping_pipeline(reach_config);
 CrossingValidator crossing;
 crossing.clear_speed_mps = std::numeric_limits<double>::infinity();
 const MotionDecisionResult creeping = creeping_pipeline.run(
 estimate, perception, map, route, limits, limits, observation, survey,
 estimate.stamp_ns, behavior, hold_planner, crossing, controller);
 assert(creeping.trajectory_verdict == TrajectoryVerdict::kMinimumRiskEscape);
 assert(std::abs(creeping.command.forward_mps) <= window_step_mps + 1e-9);
 assert(crossing.fastest_validated_mps >= limits.max_forward_mps);
 }

 // A reachable-speed candidate the validator rejects is never published.
 {
 MotionDecisionPipeline blocked_pipeline(reach_config);
 CrossingValidator crossing;
 crossing.reverse_clears = false;
 const MotionDecisionResult blocked = blocked_pipeline.run(
 crawling, perception, map, route, limits, limits, observation, survey,
 estimate.stamp_ns, behavior, hold_planner, crossing, controller);
 assert(blocked.command.is_safe_stop());
 assert(crossing.fastest_validated_mps >= limits.max_forward_mps);
 }

 // Ordinary recovery keeps the observation crawl: when stopping is safe the
 // reachable-speed set is never offered.
 {
 MotionDecisionPipeline ordinary_pipeline(reach_config);
 CrossingValidator crossing;
 crossing.hold_is_safe = true;
 const MotionDecisionResult ordinary = ordinary_pipeline.run(
 crawling, perception, map, route, limits, limits, observation, survey,
 estimate.stamp_ns, behavior, empty, crossing, controller);
 assert(ordinary.command.is_safe_stop());
 assert(crossing.fastest_validated_mps < limits.max_forward_mps);
 }

 // The behavior policy's own report that holding is unsafe opens the set
 // even when the validator would accept the hold.
 {
 MotionDecisionPipeline conflict_pipeline(reach_config);
 PersistentConflictBehavior conflict;
 CrossingValidator crossing;
 crossing.hold_is_safe = true;
 const MotionDecisionResult conflict_escape = conflict_pipeline.run(
 crawling, perception, map, route, limits, limits, observation, survey,
 estimate.stamp_ns, conflict, hold_planner, crossing, controller);
 assert(conflict_escape.constraint == "dynamic_escape");
 assert(conflict_escape.command.forward_mps == -limits.max_forward_mps);
 }

 // The targets are the declared chassis limits: the wheel-speed bound of a
 // differential base applies, and a holonomic base escapes sideways.
 {
 platform::ChassisLimits wheel_limited = limits;
 wheel_limited.max_wheel_speed_mps = 1.5;
 MotionDecisionPipeline wheel_pipeline(reach_config);
 CrossingValidator crossing;
 const MotionDecisionResult wheel = wheel_pipeline.run(
 crawling, perception, map, route, wheel_limited, wheel_limited,
 observation, survey, estimate.stamp_ns, behavior, hold_planner,
 crossing, controller);
 assert(wheel.command.forward_mps == -wheel_limited.max_wheel_speed_mps);

 platform::ChassisLimits holonomic = limits;
 holonomic.max_lateral_mps = 1.2;
 MotionDecisionPipeline slide_pipeline(reach_config);
 CrossingValidator sliding;
 sliding.reverse_clears = false;
 sliding.lateral_clears = true;
 sliding.slow_motion_is_minimum_risk = false;
 const MotionDecisionResult slide = slide_pipeline.run(
 estimate, perception, map, route, holonomic, holonomic, observation,
 survey, estimate.stamp_ns, behavior, hold_planner, sliding, controller);
 assert(std::abs(slide.command.lateral_mps) == holonomic.max_lateral_mps);
 assert(slide.command.forward_mps == 0.0);
 }

 // A started reachable-speed escape stays preferred until the escape
 // releases. The plant needs several decision periods to execute the
 // command, so a window-limited sample that is momentarily clear must not
 // replace it and publish an opposite, much smaller drive demand.
 {
 MotionDecisionPipeline commit_pipeline(reach_config);
 CrossingValidator crossing;
 const MotionDecisionResult started = commit_pipeline.run(
 crawling, perception, map, route, limits, limits, observation, survey,
 estimate.stamp_ns, behavior, hold_planner, crossing, controller);
 assert(started.command.forward_mps == -limits.max_forward_mps);
 EstimatedState reversing = estimate;
 reversing.forward_mps = -0.5;
 crossing.clear_speed_mps = 0.01;
 crossing.hold_is_safe = true;
 for (int cycle = 1; cycle <= 2; ++cycle) {
 const MotionDecisionResult kept = commit_pipeline.run(
 reversing, perception, map, route, limits, limits, observation,
 survey, estimate.stamp_ns + cycle, behavior, hold_planner, crossing,
 controller);
 assert(kept.command.forward_mps == -limits.max_forward_mps);
 }
 const MotionDecisionResult after_release = commit_pipeline.run(
 reversing, perception, map, route, limits, limits, observation, survey,
 estimate.stamp_ns + 3, behavior, hold_planner, crossing, controller);
 assert(after_release.command.forward_mps < 0.0);
 assert(after_release.command.forward_mps > -limits.max_forward_mps);
 }

 // Closed loop through the real prediction, validator and controller: a
 // robot crawling inside the observation envelope in the lane of a mover
 // that does not yield used to stop in the lane and be hit. Without a mover
 // the crawl envelope is untouched.
 const LaneCrossing free_lane = run_lane_crossing(false);
 assert(free_lane.fastest_mps <= reach_config.recovery_speed_mps + 1e-9);
 const LaneCrossing shared_lane = run_lane_crossing(true);
 if (shared_lane.closest_approach_m <= 0.0) {
 std::fprintf(stderr,
 "lane crossing contact: closest approach %.3f m, fastest %.3f m/s\n",
 shared_lane.closest_approach_m, shared_lane.fastest_mps);
 }
 assert(shared_lane.closest_approach_m > 0.0);

 std::printf("motion pipeline ok\n");
 return 0;
}
