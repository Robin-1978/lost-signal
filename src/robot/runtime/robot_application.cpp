#include "robot/runtime/robot_application.hpp"

#include <stdexcept>
#include <utility>
#include <algorithm>

namespace sim::robot {

RobotApplication::RobotApplication(RobotApplicationConfig config)
 : channel_(config.runtime_registry.create(
 config.runtime_provider_id, config.channel)),
 conformance_(std::move(config.conformance)) {
 if (!channel_)
 throw std::runtime_error("robot application could not create its LSRI provider");
 // Endpoint descriptors are immutable deployment expectations. Register them
 // before activation so a provider cannot start with an unowned or ambiguous
 // sensor stream. EndpointRegistry has already rejected duplicate ownership.
 for (const auto& endpoint : config.endpoints.endpoints()) {
 if (endpoint.role == runtime::EndpointRole::ObservationSource)
 conformance_.expect(endpoint);
 }
}

RobotApplication::RobotApplication(
 runtime::IRuntimeChannel& channel,
 autonomy::AutonomyCoordinator& coordinator,
 runtime::ProviderConformanceConfig conformance)
 : borrowed_channel_(&channel), conformance_(std::move(conformance)),
 borrowed_coordinator_(&coordinator) {}

RobotApplication::~RobotApplication() {
 if (channel_ && channel_->state() == runtime::LifecycleState::Active)
 channel_->deactivate();
}

runtime::IRuntimeChannel& RobotApplication::active_channel() {
 if (channel_)
 return *channel_;
 if (borrowed_channel_)
 return *borrowed_channel_;
 throw std::logic_error("robot application has no LSRI channel");
}

const runtime::IRuntimeChannel& RobotApplication::active_channel() const {
 if (channel_)
 return *channel_;
 if (borrowed_channel_)
 return *borrowed_channel_;
 throw std::logic_error("robot application has no LSRI channel");
}

autonomy::AutonomyCoordinator& RobotApplication::active_coordinator() {
 return borrowed_coordinator_ ? *borrowed_coordinator_ : coordinator_;
}

const autonomy::AutonomyCoordinator& RobotApplication::active_coordinator() const {
 return borrowed_coordinator_ ? *borrowed_coordinator_ : coordinator_;
}

void RobotApplication::configure() {
 active_channel().configure();
}

void RobotApplication::activate() {
 active_channel().activate();
}

void RobotApplication::deactivate() {
 active_channel().deactivate();
}

runtime::LifecycleState RobotApplication::state() const {
 return active_channel().state();
}

runtime::RuntimeHealth RobotApplication::health() const {
 return active_channel().health();
}

const runtime::IRuntimeChannel& RobotApplication::channel() const {
 return active_channel();
}

runtime::IAutonomyPort& RobotApplication::autonomy_port() {
 return active_channel().autonomy();
}

runtime::IPlatformPort& RobotApplication::platform_port() {
 return active_channel().platform();
}

bool RobotApplication::publish_observation(
 ObservationFrame observation, runtime::TimePoint now) {
 const bool accepted = active_channel().platform().publish_observation(observation);
 if (accepted)
 conformance_.observe(observation, now);
 return accepted;
}

void RobotApplication::check_provider_liveness(runtime::TimePoint now) {
 conformance_.check_liveness(now);
}

RobotCycleOutput RobotApplication::step(const RobotCycleInput& input) {
 if (state() != runtime::LifecycleState::Active)
 throw std::logic_error("robot cycle requires an active application");
 RobotCycleOutput output;
 auto& coordinator = active_coordinator();
 if (input.run_autonomy) {
 const auto autonomy_cycle = coordinator.run_cycle({input.autonomy, input.local_planning});
 output.autonomy = autonomy_cycle.autonomy;
 output.local_planning = autonomy_cycle.local_planning;
 } else if (input.local_planning) {
 output.local_planning = coordinator.plan_local_motion(*input.local_planning);
  const auto& status = output.local_planning->status;
  MotionCapability capability = MotionCapability::Unknown;
  switch (status.mode) {
  case LocalMotionMode::Nominal:
   capability = status.motion_authorized ? MotionCapability::Normal : MotionCapability::Hold;
   break;
  case LocalMotionMode::LocalizationReduced:
   capability = status.motion_authorized ? MotionCapability::Reduced : MotionCapability::Hold;
   break;
  case LocalMotionMode::LocalizationHold:
   capability = MotionCapability::Hold;
   break;
  case LocalMotionMode::MinimumRiskEscape:
   capability = status.motion_authorized ? MotionCapability::Reduced : MotionCapability::Hold;
   break;
  case LocalMotionMode::NoSafeAction:
   capability = MotionCapability::NoSafeAction;
   break;
  }
  executive_.report_motion({capability, status.speed_cap_mps, status.reason,
   status.recovery_condition, status.decision_time_s});
 }
 // A body/actuator intent is only publishable after the ROBOT planner has
 // proved the corresponding local transaction. This prevents a native or
 // simulated adapter from bypassing trajectory validation by supplying a
 // command beside an empty or rejected candidate set.
 if (input.local_planning &&
 (!output.local_planning || !output.local_planning->status.motion_authorized))
 return output;
 if (!input.motion_intent)
 return output;
 if (!(input.dt_s > 0.0))
 throw std::invalid_argument("robot cycle requires a positive timestep");
 auto motion = execute_motion(*input.motion_intent, input.dt_s, input.now, input.valid_until);
 output.motion_command = std::move(motion.motion_command);
 output.actuator_command = std::move(motion.actuator_command);
 return output;
}

RobotCycleOutput RobotApplication::execute_motion(
 const DiffCmd& motion_intent, double dt_s, runtime::TimePoint now,
 runtime::TimePoint valid_until) {
 if (state() != runtime::LifecycleState::Active)
 throw std::logic_error("robot motion requires an active application");
 if (!(dt_s > 0.0))
 throw std::invalid_argument("robot motion requires a positive timestep");
 RobotCycleOutput output;
 auto& coordinator = active_coordinator();
 const auto sequence = coordinator.motion_runtime().command_sequence()++;
 if (coordinator.has_differential_controller() ||
 coordinator.has_ackermann_controller() ||
 coordinator.has_holonomic_controller()) {
 const auto feedback = autonomy_port().read_observation();
 if (!feedback)
 throw std::runtime_error("robot cycle requires actuator feedback");
 output.actuator_command = coordinator.update_chassis_actuators(
 motion_intent, *feedback, dt_s, now, valid_until, sequence).command;
 if (autonomy_port().write_actuators(*output.actuator_command) !=
 runtime::CommandWriteResult::Accepted)
 throw std::runtime_error("robot cycle actuator command was rejected");
 return output;
 }
 output.motion_command = runtime::make_motion_command(
 motion_intent, channel().config().capabilities.chassis, now,
 sequence, std::max(0.0, valid_until.seconds() - now.seconds()),
 "robot.autonomy");
 if (autonomy_port().write_motion(*output.motion_command) !=
 runtime::CommandWriteResult::Accepted)
 throw std::runtime_error("robot cycle motion command was rejected");
 return output;
}

} // namespace sim::robot
