#pragma once

#include "robot/autonomy/pipeline/autonomy_coordinator.hpp"
#include "robot/executive/task_executive.hpp"
#include "robot/runtime/runtime_channel.hpp"
#include "robot/runtime/endpoint_registry.hpp"
#include "robot/runtime/provider_conformance.hpp"

#include <memory>
#include <optional>
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
 // Endpoint ownership is deployment data. The same descriptors are used by
 // simulated, replay and native providers; only backend/provider_id changes.
 runtime::EndpointRegistry endpoints;
 runtime::ProviderConformanceConfig conformance;
};

// One provider-neutral native/SIM cycle. Task and perception composition may
// supply a body-motion intent after local planning; the application owns the
// final ROBOT transaction and never accepts provider-native objects.
struct RobotCycleInput {
 autonomy::AutonomyCoordinatorInput autonomy;
 // Optional local-planning transaction. The caller provides only ROBOT
 // contracts; the application runs it before publishing the actuator/body
 // command. Native and simulated composition roots therefore share one
 // autonomy cycle instead of duplicating planner orchestration.
 const autonomy::RobotLocalPlanningInput* local_planning{nullptr};
 std::optional<DiffCmd> motion_intent;
 double dt_s{0.0};
 runtime::TimePoint now;
 runtime::TimePoint valid_until;
};

struct RobotCycleOutput {
 autonomy::AutonomyCoordinatorOutput autonomy;
 std::optional<LocalPlanningOutput> local_planning;
 std::optional<runtime::MotionCommand> motion_command;
 std::optional<runtime::ActuatorCommand> actuator_command;
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

 // Provider adapters should use these helpers instead of bypassing the
 // conformance monitor when publishing frames or checking liveness.
 bool publish_observation(ObservationFrame observation, runtime::TimePoint now);
 void check_provider_liveness(runtime::TimePoint now);
 RobotCycleOutput step(const RobotCycleInput& input);
 const runtime::ProviderConformanceMonitor& conformance() const { return conformance_; }
 runtime::ProviderConformanceMonitor& conformance() { return conformance_; }

 autonomy::AutonomyCoordinator& coordinator() { return coordinator_; }
 const autonomy::AutonomyCoordinator& coordinator() const { return coordinator_; }
 executive::TaskExecutive& executive() { return executive_; }
 const executive::TaskExecutive& executive() const { return executive_; }

 private:
 std::unique_ptr<runtime::IRuntimeChannel> channel_;
 runtime::ProviderConformanceMonitor conformance_;
 autonomy::AutonomyCoordinator coordinator_;
 executive::TaskExecutive executive_;
};

} // namespace sim::robot
