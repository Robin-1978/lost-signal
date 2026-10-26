#pragma once

#include "robot/autonomy/behavior/behavior.hpp"
#include "robot/autonomy/control/controller.hpp"
#include "robot/autonomy/decision/motion_pipeline.hpp"
#include "robot/autonomy/estimation/estimator.hpp"
#include "robot/autonomy/perception/perception.hpp"
#include "robot/autonomy/planning/global_planner.hpp"
#include "robot/autonomy/planning/local_planner.hpp"
#include "robot/autonomy/planning/route_pipeline.hpp"
#include "robot/autonomy/validation/validator.hpp"

namespace lost_signal::robot::autonomy {

// DecisionCyclePipeline owns route-refresh and motion-selection policy. The
// coordinator supplies current ROBOT evidence and module seams, then merely
// publishes the returned result. No SIM state or truth-side type can enter
// this pipeline.
class DecisionCyclePipeline {
 public:
 DecisionCyclePipeline() = default;
 explicit DecisionCyclePipeline(MotionDecisionPipeline::Config config)
 : motion_(config) {}
 void set_decision_period(double period_s) {
 motion_.set_decision_period(period_s);
 }
 void reset() {
 motion_.reset();
 route_.reset();
 local_replan_requested_ = false;
 }
 MotionDecisionResult run(
 IGlobalPlanner& global_planner, ILocalPlanner& local_planner,
 IBehaviorPolicy& behavior, IValidator& validator, IController& controller,
 const EstimatedState& estimate, const PerceptionResult& perception,
 const LocalMap& local_map, Route& active_route, const RoutePoint& goal,
 bool has_goal, const iface::MapSnapshot& map, bool force_route_refresh,
 bool reset_local_plan,
 const platform::ChassisLimits& limits,
 const platform::ChassisLimits& behavior_limits,
 const iface::ObservationFrame& observation, const SurveyContext& survey,
 core::MonotonicNs now_ns, bool forward_blocked = false,
 bool observation_motion_requested = false);

 private:
 RoutePlanningPipeline route_;
 MotionDecisionPipeline motion_;
 bool local_replan_requested_ = false;
};

} // namespace lost_signal::robot::autonomy
