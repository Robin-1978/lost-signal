#pragma once

#include "robot/autonomy/pipeline/autonomy_coordinator.hpp"
#include "robot/executive/task_executive.hpp"
#include "robot/runtime/runtime_channel.hpp"

#include <memory>
#include <string>

namespace sim::robot {

// Native-process composition root. It owns only ROBOT objects and a selected
// LSRI channel. SIM uses the same stack in-process, but supplies a simulated
// provider from its own composition root. No game, renderer or physics type
// can enter this class.
struct RobotApplicationConfig {
 runtime::RuntimeRegistry runtime_registry{runtime::RuntimeRegistry::with_builtins()};
 std::string runtime_provider_id{"channel.in-process"};
 runtime::RuntimeChannelConfig channel;
};

class RobotApplication {
 public:
 explicit RobotApplication(RobotApplicationConfig config);
 ~RobotApplication();

 RobotApplication(const RobotApplication&) = delete;
 RobotApplication& operator=(const RobotApplication&) = delete;

 void configure();
 void activate();
 void deactivate();

 runtime::LifecycleState state() const;
 runtime::RuntimeHealth health() const;
 const runtime::IRuntimeChannel& channel() const;
 runtime::IAutonomyPort& autonomy_port();
 runtime::IPlatformPort& platform_port();

 autonomy::AutonomyCoordinator& coordinator() { return coordinator_; }
 const autonomy::AutonomyCoordinator& coordinator() const { return coordinator_; }
 executive::TaskExecutive& executive() { return executive_; }
 const executive::TaskExecutive& executive() const { return executive_; }

 private:
 std::unique_ptr<runtime::IRuntimeChannel> channel_;
 autonomy::AutonomyCoordinator coordinator_;
 executive::TaskExecutive executive_;
};

} // namespace sim::robot
