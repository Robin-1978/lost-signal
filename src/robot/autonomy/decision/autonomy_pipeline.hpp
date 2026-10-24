#pragma once

#include "robot/autonomy/behavior/behavior.hpp"
#include "robot/autonomy/control/controller.hpp"
#include "robot/autonomy/decision/motion_pipeline.hpp"
#include "robot/autonomy/estimation/estimator.hpp"
#include "robot/autonomy/perception/perception.hpp"
#include "robot/autonomy/planning/global_planner.hpp"
#include "robot/autonomy/planning/local_planner.hpp"
#include "robot/autonomy/validation/validator.hpp"

namespace lost_signal::robot::autonomy {

// DecisionCyclePipeline owns route-refresh and motion-selection policy. The
// coordinator supplies current ROBOT evidence and module seams, then merely
// publishes the returned result. No SIM state or truth-side type can enter
// this pipeline.
class DecisionCyclePipeline {
 public:
 static MotionDecisionResult run(
 IGlobalPlanner& global_planner, ILocalPlanner& local_planner,
 IBehaviorPolicy& behavior, IValidator& validator, IController& controller,
 const EstimatedState& estimate, const PerceptionResult& perception,
 const LocalMap& local_map, Route& active_route, const RoutePoint& goal,
 bool has_goal, const iface::MapSnapshot& map, bool force_route_refresh,
 const platform::ChassisLimits& limits,
 const platform::ChassisLimits& behavior_limits,
 const iface::ObservationFrame& observation, const SurveyContext& survey,
 core::MonotonicNs now_ns);
};

} // namespace lost_signal::robot::autonomy
