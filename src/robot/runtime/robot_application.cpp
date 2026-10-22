#include "robot/runtime/robot_application.hpp"

#include <stdexcept>
#include <utility>
#include <algorithm>

namespace sim::robot {

RobotApplication::RobotApplication(RobotApplicationConfig config)
 : channel_(config.runtime_registry.create(
 config.runtime_provider_id, config.channel)),
 conformance_(std::move(config.conformance)),
 coordinator_(std::move(config.high_level_policy)),
 map_provider_(config.map_provider) {
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

RobotApplication::RobotApplication(
 runtime::IRuntimeChannel& channel,
 std::unique_ptr<autonomy::AutonomyCoordinator> coordinator,
 runtime::ProviderConformanceConfig conformance)
 : borrowed_channel_(&channel), conformance_(std::move(conformance)),
 owned_coordinator_(std::move(coordinator)) {
 if (!owned_coordinator_)
 throw std::invalid_argument("robot application requires an autonomy coordinator");
}

RobotApplication::RobotApplication(
 runtime::IRuntimeChannel& channel,
 std::shared_ptr<::sim::IHighLevelPolicy> policy,
 runtime::ProviderConformanceConfig conformance)
 : borrowed_channel_(&channel), conformance_(std::move(conformance)),
 owned_coordinator_(std::make_unique<autonomy::AutonomyCoordinator>(std::move(policy))) {}

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
 if (borrowed_coordinator_)
 return *borrowed_coordinator_;
 if (owned_coordinator_)
 return *owned_coordinator_;
 return coordinator_;
}

const autonomy::AutonomyCoordinator& RobotApplication::active_coordinator() const {
 if (borrowed_coordinator_)
 return *borrowed_coordinator_;
 if (owned_coordinator_)
 return *owned_coordinator_;
 return coordinator_;
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

runtime::IMapProvider& RobotApplication::map_provider() {
 if (!map_provider_)
 throw std::logic_error("robot application has no map provider");
 return *map_provider_;
}

const runtime::IMapProvider& RobotApplication::map_provider() const {
 if (!map_provider_)
 throw std::logic_error("robot application has no map provider");
 return *map_provider_;
}

void RobotApplication::refresh_traversal_map() {
 if (!map_provider_)
 return;
 auto snapshot = map_provider_->current_snapshot();
 if (!snapshot || snapshot.get() == traversal_map_.get())
 return;
 traversal_.bind(std::move(snapshot));
 traversal_map_ = traversal_.ready() ?
 map_provider_->current_snapshot() : nullptr;
}

TerrainSample RobotApplication::estimated_terrain(Vec2 position) {
 refresh_traversal_map();
 return traversal_.sample(position);
}

TraversalContactEstimate RobotApplication::estimated_contact(Vec2 position) {
 refresh_traversal_map();
 return traversal_.contact(position);
}

double RobotApplication::estimated_traversal_cost(Vec2 from, Vec2 to,
 const TerrainCapability& capability) {
 refresh_traversal_map();
 return traversal_.traversal_cost(from, to, capability);
}

void RobotApplication::bind_map_catalog(
 runtime::MapCatalog& catalog, runtime::MapManager& active_map) {
 map_catalog_ = &catalog;
 active_map_manager_ = &active_map;
 map_provider_ = &active_map;
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

RobotCycleOutput RobotApplication::tick(const RobotCycleInput& input) {
 if (state() != runtime::LifecycleState::Active)
 throw std::logic_error("robot cycle requires an active application");
 RobotCycleOutput output;
 if (input.task_observation)
 executive_.update(*input.task_observation);
 if (const auto request = executive_.pending_map_request()) {
 // An active task must first report a hold capability. The request remains
 // queued while the planner is still moving, so a UI command cannot race a
 // live trajectory or silently discard the user's selection.
 const bool task_allows_switch =
 executive_.state() != executive::TaskState::Active ||
 executive_.status().motion_capability == MotionCapability::Hold;
 if (task_allows_switch && map_catalog_ && active_map_manager_) {
 std::string reason;
 if (map_catalog_->select(*active_map_manager_, request->map_id, &reason)) {
 executive_.report_map_selection(request->map_id, true);
 } else {
 executive_.report_map_selection({}, false, reason);
 }
 }
 }
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
