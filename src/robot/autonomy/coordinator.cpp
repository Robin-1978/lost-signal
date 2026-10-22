#include "robot/autonomy/coordinator.hpp"

namespace lost_signal::robot::autonomy {

AutonomyCoordinator::AutonomyCoordinator(platform::ChassisLimits limits)
    : limits_(limits),
      estimator_(std::make_unique<DeadReckoningEstimator>()),
      perception_module_(std::make_unique<NullPerception>()),
      mapper_(std::make_unique<NullMapper>()),
      behavior_(std::make_unique<WaitOnIntrusionBehavior>()),
      local_planner_(std::make_unique<HoldPlanner>()),
      validator_(std::make_unique<ClearanceValidator>()),
      controller_(std::make_unique<DirectController>()) {}

core::Status AutonomyCoordinator::configure() {
  estimator_->reset();
  perception_module_->reset();
  mapper_->reset();
  behavior_->reset();
  local_planner_->reset();
  controller_->reset();
  has_observation_ = false;
  last_obs_ = iface::ObservationFrame{};
  perception_ = PerceptionResult{};
  decision_ = BehaviorDecision{};
  state_ = lifecycle::ModuleState::kInactive;
  return core::Status::Ok();
}

core::Status AutonomyCoordinator::activate() {
  if (state_ != lifecycle::ModuleState::kInactive) {
    return core::Status::Fail("autonomy: activate requires Inactive");
  }
  state_ = lifecycle::ModuleState::kActive;
  return core::Status::Ok();
}

core::Status AutonomyCoordinator::deactivate() {
  if (state_ == lifecycle::ModuleState::kActive) {
    state_ = lifecycle::ModuleState::kInactive;
  }
  return core::Status::Ok();
}

core::Status AutonomyCoordinator::cleanup() {
  estimator_->reset();
  perception_module_->reset();
  mapper_->reset();
  behavior_->reset();
  local_planner_->reset();
  controller_->reset();
  has_observation_ = false;
  last_obs_ = iface::ObservationFrame{};
  perception_ = PerceptionResult{};
  decision_ = BehaviorDecision{};
  state_ = lifecycle::ModuleState::kUnconfigured;
  return core::Status::Ok();
}

void AutonomyCoordinator::ingest_observation(
    const iface::ObservationFrame& obs) {
  if (state_ != lifecycle::ModuleState::kActive) return;
  if (!iface::valid_observation_frame(obs)) return;
  // Fail-closed: stale or repeated input never advances state.
  if (has_observation_ && obs.sequence <= last_obs_.sequence) return;
  if (has_observation_ && obs.stamp_ns <= last_obs_.stamp_ns) return;
  estimator_->predict(obs);
  estimator_->correct(obs);
  perception_ = perception_module_->update(obs, estimator_->estimate());
  mapper_->update(obs, estimator_->estimate());
  last_obs_ = obs;
  has_observation_ = true;
}

iface::MotionCommand AutonomyCoordinator::update(
    iface::MapSnapshot /*map*/, core::MonotonicNs now_ns) {
  // Missing observation is unknown evidence -> safe stop, never free space.
  if (state_ != lifecycle::ModuleState::kActive || !has_observation_) {
    return iface::MotionCommand::SafeStop(now_ns);
  }
  // Dynamic decision first: behavior turns tracks + estimate into one
  // constraint. A hold short-circuits sampling but still passes the
  // independent validator + controller, so the evidence chain is uniform.
  decision_ =
      behavior_->decide(estimator_->estimate(), perception_, limits_);
  if (decision_.must_hold) {
    LocalCandidate hold;
    hold.feasible = true;
    ValidationResult verdict =
        validator_->validate(hold, estimator_->estimate(), perception_,
                             last_obs_);
    if (!verdict.accepted) return iface::MotionCommand::SafeStop(now_ns);
    return controller_->track(hold, estimator_->estimate(), now_ns);
  }
  platform::ChassisLimits effective = limits_;
  if (decision_.max_forward_mps < effective.max_forward_mps) {
    effective.max_forward_mps = decision_.max_forward_mps;
  }
  LocalPlan plan = local_planner_->plan(estimator_->estimate(), perception_,
                                        mapper_->local_map(), effective);
  if (!plan.has_plan) return iface::MotionCommand::SafeStop(now_ns);
  ValidationResult verdict =
      validator_->validate(plan.best, estimator_->estimate(), perception_,
                           last_obs_);
  if (!verdict.accepted) return iface::MotionCommand::SafeStop(now_ns);
  return controller_->track(plan.best, estimator_->estimate(), now_ns);
}

const EstimatedState& AutonomyCoordinator::estimate() const {
  return estimator_->estimate();
}

LocalMap AutonomyCoordinator::local_map() const {
  return mapper_->local_map();
}

}  // namespace lost_signal::robot::autonomy
