#pragma once

#include <string>

#include "robot/autonomy/behavior/behavior.hpp"
#include "robot/autonomy/control/controller.hpp"
#include "robot/autonomy/estimation/estimator.hpp"
#include "robot/autonomy/mapping/mapper.hpp"
#include "robot/autonomy/perception/perception.hpp"
#include "robot/autonomy/planning/local_planner.hpp"
#include "robot/autonomy/validation/validator.hpp"
#include "robot/iface/command.hpp"

namespace lost_signal::robot::autonomy {

// The motion decision pipeline owns trajectory policy. The coordinator only
// supplies current ROBOT evidence and wires the pipeline's module seams.
struct MotionDecisionResult {
 iface::MotionCommand command;
 BehaviorDecision behavior;
 std::string constraint = "none";
 std::string reason = "no-plan";
};

class MotionDecisionPipeline {
 public:
 static MotionDecisionResult run(
 const EstimatedState& est, const PerceptionResult& perception,
 const LocalMap& local_map, const Route& route,
 const platform::ChassisLimits& limits,
 const platform::ChassisLimits& behavior_limits,
 const iface::ObservationFrame& observation,
 const SurveyContext& survey, core::MonotonicNs now_ns,
 IBehaviorPolicy& behavior,
 ILocalPlanner& local_planner, IValidator& validator,
 IController& controller, bool forward_blocked = false);
};

} // namespace lost_signal::robot::autonomy
