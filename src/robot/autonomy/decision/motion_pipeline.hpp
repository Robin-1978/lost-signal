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
 double prediction_horizon_s = 0.0;
 std::size_t prediction_hypotheses = 0;
};

class MotionDecisionPipeline {
 public:
 struct Config {
 double recovery_speed_mps = 0.25;
 double recovery_yaw_rate_radps = 0.6;
 double recovery_projection_horizon_s = 1.0;
 // Dynamic escape is released only after a stationary rollout retains this
 // extra predicted separation for several consecutive decisions.
 double escape_release_clearance_m = 0.20;
 std::size_t escape_release_cycles = 3;
 };

 MotionDecisionPipeline() = default;
 explicit MotionDecisionPipeline(Config config) : config_(config) {}
 void reset();
 MotionDecisionResult run(
 const EstimatedState& est, const PerceptionResult& perception,
 const LocalMap& local_map, const Route& route,
 const platform::ChassisLimits& limits,
 const platform::ChassisLimits& behavior_limits,
 const iface::ObservationFrame& observation,
 const SurveyContext& survey, core::MonotonicNs now_ns,
 IBehaviorPolicy& behavior,
 ILocalPlanner& local_planner, IValidator& validator,
 IController& controller, bool forward_blocked = false);

 private:
 Config config_;
 bool escape_active_ = false;
 LocalCandidate escape_command_{};
 std::size_t escape_release_count_ = 0;
};

} // namespace lost_signal::robot::autonomy
