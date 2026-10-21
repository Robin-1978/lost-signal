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

RobotApplication::~RobotApplication() {
 if (channel_ && channel_->state() == runtime::LifecycleState::Active)
 channel_->deactivate();
}

void RobotApplication::configure() {
 channel_->configure();
}

void RobotApplication::activate() {
 channel_->activate();
}

void RobotApplication::deactivate() {
 channel_->deactivate();
}

runtime::LifecycleState RobotApplication::state() const {
 return channel_->state();
}

runtime::RuntimeHealth RobotApplication::health() const {
 return channel_->health();
}

const runtime::IRuntimeChannel& RobotApplication::channel() const {
 return *channel_;
}

runtime::IAutonomyPort& RobotApplication::autonomy_port() {
 return channel_->autonomy();
}

runtime::IPlatformPort& RobotApplication::platform_port() {
 return channel_->platform();
}

bool RobotApplication::publish_observation(
 ObservationFrame observation, runtime::TimePoint now) {
 const bool accepted = channel_->platform().publish_observation(observation);
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
 const auto autonomy_cycle = coordinator_.run_cycle({input.autonomy, input.local_planning});
 output.autonomy = autonomy_cycle.autonomy;
 output.local_planning = autonomy_cycle.local_planning;
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
 const auto sequence = coordinator_.motion_runtime().command_sequence()++;
 if (coordinator_.has_differential_controller() ||
 coordinator_.has_ackermann_controller() ||
 coordinator_.has_holonomic_controller()) {
 const auto feedback = autonomy_port().read_observation();
 if (!feedback)
 throw std::runtime_error("robot cycle requires actuator feedback");
 output.actuator_command = coordinator_.update_chassis_actuators(
 *input.motion_intent, *feedback, input.dt_s, input.now, input.valid_until, sequence).command;
 if (autonomy_port().write_actuators(*output.actuator_command) !=
 runtime::CommandWriteResult::Accepted)
 throw std::runtime_error("robot cycle actuator command was rejected");
 return output;
 }
 output.motion_command = runtime::make_motion_command(
 *input.motion_intent, channel().config().capabilities.chassis, input.now,
 sequence, std::max(0.0, input.valid_until.seconds() - input.now.seconds()),
 "robot.autonomy");
 if (autonomy_port().write_motion(*output.motion_command) !=
 runtime::CommandWriteResult::Accepted)
 throw std::runtime_error("robot cycle motion command was rejected");
 return output;
}

} // namespace sim::robot
