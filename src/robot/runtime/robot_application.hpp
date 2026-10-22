#pragma once

#include "robot/autonomy/pipeline/autonomy_coordinator.hpp"
#include "robot/executive/robot_executive.hpp"
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
 // A composition root may already have run the behavior transaction while
 // assembling this cycle's local-planning facts. In that case ROBOT still
 // owns local planning and validation, but must not update the policy twice
 // in one decision tick.
 bool run_autonomy{true};
 // Optional task observation for the executive.  When present, the same
 // application tick updates task lifecycle before autonomy runs.  Keeping
 // this here makes the ordering explicit without making the executive a
 // second scheduler or asking SIM to call it separately.
 std::optional<executive::TaskObservation> task_observation;
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
 // In-process composition constructor. The SIM adapter owns the provider
 // channel and supplies it here without transferring ownership. ROBOT still
 // owns the cycle and controller transaction; this is the same boundary used
 // by the native composition root, without requiring IPC.
 RobotApplication(runtime::IRuntimeChannel& channel,
  autonomy::AutonomyCoordinator& coordinator,
  runtime::ProviderConformanceConfig conformance = {});
 // Owning composition constructor used by SIM and native deployment roots.
 // The coordinator lifetime is then unambiguously inside ROBOT.
 RobotApplication(runtime::IRuntimeChannel& channel,
  std::unique_ptr<autonomy::AutonomyCoordinator> coordinator,
  runtime::ProviderConformanceConfig conformance = {});
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
 // The single ROBOT cycle entry point for native and in-process composition.
 // Executive and autonomy are ordered here; callers do not schedule either
 // object independently.
 RobotCycleOutput tick(const RobotCycleInput& input);
 // Execute an already selected body-motion intent through the ROBOT
 // controller and LSRI provider. Planning must have produced the intent;
 // this entry point only owns feedback ingestion, actuator conversion,
 // expiry and command publication.
 RobotCycleOutput execute_motion(const DiffCmd& motion_intent, double dt_s,
 runtime::TimePoint now, runtime::TimePoint valid_until);
 const runtime::ProviderConformanceMonitor& conformance() const { return conformance_; }
 runtime::ProviderConformanceMonitor& conformance() { return conformance_; }

 autonomy::AutonomyCoordinator& coordinator() { return active_coordinator(); }
 const autonomy::AutonomyCoordinator& coordinator() const { return active_coordinator(); }
 RobotExecutive& executive() { return executive_; }
 const RobotExecutive& executive() const { return executive_; }

 private:
 std::unique_ptr<runtime::IRuntimeChannel> channel_;
 runtime::IRuntimeChannel* borrowed_channel_{nullptr};
 runtime::ProviderConformanceMonitor conformance_;
 autonomy::AutonomyCoordinator coordinator_;
 std::unique_ptr<autonomy::AutonomyCoordinator> owned_coordinator_;
 autonomy::AutonomyCoordinator* borrowed_coordinator_{nullptr};
 RobotExecutive executive_;

 runtime::IRuntimeChannel& active_channel();
 const runtime::IRuntimeChannel& active_channel() const;
 autonomy::AutonomyCoordinator& active_coordinator();
 const autonomy::AutonomyCoordinator& active_coordinator() const;
};

} // namespace sim::robot
