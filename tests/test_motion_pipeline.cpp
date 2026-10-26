#include <cassert>
#include <cmath>
#include <cstdio>

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
 result.reason = "clear";
 return result;
 }
};

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

 // A supervised localization search requests observation motion only when
 // the normal planner has no accepted moving candidate. The fallback crawl
 // remains validator-approved and respects Ackermann curvature.
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

 std::printf("motion pipeline ok\n");
 return 0;
}
