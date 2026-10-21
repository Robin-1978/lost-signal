#include "robot/runtime/robot_application.hpp"

#include <stdexcept>
#include <utility>

namespace sim::robot {

RobotApplication::RobotApplication(RobotApplicationConfig config)
 : channel_(config.runtime_registry.create(
 config.runtime_provider_id, config.channel)) {
 if (!channel_)
 throw std::runtime_error("robot application could not create its LSRI provider");
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

} // namespace sim::robot
