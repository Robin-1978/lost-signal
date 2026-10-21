#include "robot/autonomy/pipeline/autonomy_coordinator.hpp"

#include <stdexcept>
#include <utility>
#include <array>
#include <algorithm>
#include <cmath>

namespace sim::autonomy {

AutonomyCoordinator::AutonomyCoordinator(std::shared_ptr<IHighLevelPolicy> policy)
 : policy_(policy ? std::move(policy) : make_route_baseline_policy()) {
 if (!policy_)
 throw std::invalid_argument("autonomy coordinator requires a policy");
 policy_name_ = policy_->name();
}

void AutonomyCoordinator::reset(std::uint64_t seed) {
 policy_->reset(seed);
 policy_name_ = policy_->name();
 diagnostics_.reset();
 motion_runtime_.reset();
}

AutonomyCoordinatorOutput AutonomyCoordinator::update(
 const AutonomyCoordinatorInput& input) {
 // Policy selection and policy safety are one ROBOT-side transaction.  The
 // caller owns only observation assembly and cannot accidentally publish a
 // raw custom-policy action that bypasses the common safety shield.
 const auto requested = policy_->decide(input.observation);
 AutonomyCoordinatorOutput output;
 output.decision = apply_strategy_safety(requested, input.observation, input.safety);
 output.policy_name = policy_name_;
 return output;
}

void AutonomyCoordinator::configure_route_tracking(double kp, double ki,
 double kd, double output_limit, double integral_limit) {
 route_tracking_ = std::make_unique<RouteTrackingSystem>(
 kp, ki, kd, output_limit, integral_limit);
}

RouteTrackingSystem& AutonomyCoordinator::route_tracking() {
 if (!route_tracking_)
 throw std::logic_error("route tracking has not been configured");
 return *route_tracking_;
}

const RouteTrackingSystem& AutonomyCoordinator::route_tracking() const {
 if (!route_tracking_)
 throw std::logic_error("route tracking has not been configured");
 return *route_tracking_;
}

bool AutonomyCoordinator::update_controlled_stop(
 bool carrying_stop, double peak_wheel_speed_mps,
 const ControlledStopLatch::Config& config) {
 return controlled_stop_latch_.update(carrying_stop, peak_wheel_speed_mps, config);
}

void AutonomyCoordinator::reset_controlled_stop() {
 controlled_stop_latch_.reset();
}

void AutonomyCoordinator::configure_collision_monitor(
 Vec2 world_size, double robot_radius_m,
 std::vector<CollisionBox> static_obstacles) {
 collision_monitor_ = std::make_unique<CollisionMonitor>(
 world_size, robot_radius_m, std::move(static_obstacles));
}

CollisionMonitor& AutonomyCoordinator::collision_monitor() {
 if (!collision_monitor_)
 throw std::logic_error("collision monitor has not been configured");
 return *collision_monitor_;
}

const CollisionMonitor& AutonomyCoordinator::collision_monitor() const {
 if (!collision_monitor_)
 throw std::logic_error("collision monitor has not been configured");
 return *collision_monitor_;
}

void AutonomyCoordinator::reset_chassis_controllers() {
 differential_controller_.reset();
 ackermann_controller_.reset();
 holonomic_controller_.reset();
}

void AutonomyCoordinator::configure_differential_controller(
 DifferentialWheelControllerConfig config, WheelPidGains gains) {
 reset_chassis_controllers();
 differential_controller_ = std::make_unique<DifferentialWheelController>(
 std::move(config), gains);
}

void AutonomyCoordinator::configure_ackermann_controller(
 AckermannActuatorControllerConfig config, WheelPidGains gains) {
 reset_chassis_controllers();
 ackermann_controller_ = std::make_unique<AckermannActuatorController>(
 std::move(config), gains);
}

void AutonomyCoordinator::configure_holonomic_controller(
 HolonomicActuatorControllerConfig config, WheelPidGains gains) {
 reset_chassis_controllers();
 holonomic_controller_ = std::make_unique<HolonomicActuatorController>(
 std::move(config), gains);
}

DifferentialWheelController& AutonomyCoordinator::differential_controller() {
 if (!differential_controller_)
 throw std::logic_error("differential controller has not been configured");
 return *differential_controller_;
}

AckermannActuatorController& AutonomyCoordinator::ackermann_controller() {
 if (!ackermann_controller_)
 throw std::logic_error("Ackermann controller has not been configured");
 return *ackermann_controller_;
}

HolonomicActuatorController& AutonomyCoordinator::holonomic_controller() {
 if (!holonomic_controller_)
 throw std::logic_error("holonomic controller has not been configured");
 return *holonomic_controller_;
}

ChassisControlResult AutonomyCoordinator::update_chassis_actuators(
 const DiffCmd& command, const ObservationFrame& feedback,
 double dt_s, runtime::TimePoint now, runtime::TimePoint valid_until,
 std::uint64_t sequence) {
 if (!feedback.actuator_feedback)
 throw std::invalid_argument("chassis actuator control requires actuator feedback");
 if (!(dt_s > 0.0) || !std::isfinite(dt_s))
 throw std::invalid_argument("chassis actuator control requires a finite positive timestep");

 const auto& channels = *feedback.actuator_feedback;
 const auto require = [&](const std::string& id) -> const ActuatorFeedbackObservation::Channel& {
 const auto* value = channels.find(id);
 if (!value)
 throw std::runtime_error("actuator feedback is missing channel '" + id + "'");
 return *value;
 };
 ChassisControlResult result;
 result.command.header = {
 runtime::kMessageSchemaVersion, now, now, sequence, "base_link", "robot.chassis-controller"};
 result.command.valid_until = valid_until;

 if (differential_controller_) {
 const auto& left = require("left");
 const auto& right = require("right");
 const auto output = differential_controller_->update(
 command, {left.position_rad, right.position_rad, left.velocity_radps,
 right.velocity_radps, true}, dt_s);
 result.command.setpoints = {
 {"left", runtime::ActuatorCommandMode::NormalizedDuty, output.left.normalized_duty},
 {"right", runtime::ActuatorCommandMode::NormalizedDuty, output.right.normalized_duty},
 };
 result.command.header.source_id = "robot.differential-controller";
 return result;
 }

 if (ackermann_controller_) {
 const auto& rear_left = require("rear_left_drive");
 const auto& rear_right = require("rear_right_drive");
 const auto& front_left = require("front_left_steer");
 const auto& front_right = require("front_right_steer");
 const auto output = ackermann_controller_->update(
 command, {rear_left.position_rad, rear_right.position_rad,
 rear_left.velocity_radps, rear_right.velocity_radps,
 front_left.position_rad, front_right.position_rad,
 front_left.velocity_radps, front_right.velocity_radps, true}, dt_s);
 result.command.setpoints = {
 {"rear_left_drive", runtime::ActuatorCommandMode::NormalizedDuty,
 output.rear_left.normalized_duty},
 {"rear_right_drive", runtime::ActuatorCommandMode::NormalizedDuty,
 output.rear_right.normalized_duty},
 {"front_left_steer", runtime::ActuatorCommandMode::NormalizedDuty,
 output.front_left_steer.normalized_duty},
 {"front_right_steer", runtime::ActuatorCommandMode::NormalizedDuty,
 output.front_right_steer.normalized_duty},
 };
 if (ackermann_controller_->config().service_brake_enabled) {
 result.command.setpoints.push_back({
 "rear_left_brake", runtime::ActuatorCommandMode::NormalizedBrake,
 output.rear_left_brake_request});
 result.command.setpoints.push_back({
 "rear_right_brake", runtime::ActuatorCommandMode::NormalizedBrake,
 output.rear_right_brake_request});
 }
 result.command.header.source_id = "robot.ackermann-controller";
 return result;
 }

 if (holonomic_controller_) {
 constexpr std::array<const char*, 4> drive_channels{
 "front_left_drive", "front_right_drive", "rear_left_drive", "rear_right_drive"};
 constexpr std::array<const char*, 4> steer_channels{
 "front_left_steer", "front_right_steer", "rear_left_steer", "rear_right_steer"};
 HolonomicActuatorFeedback input;
 for (std::size_t index = 0; index < drive_channels.size(); ++index) {
 const auto& drive = require(drive_channels[index]);
 input.drive_velocity_radps[index] = drive.velocity_radps;
 // Mecanum has no steer feedback. The controller ignores these fields in
 // that mode, while swerve requires all four channels below.
 if (holonomic_controller_->kind() == HolonomicControlKind::Swerve) {
 const auto& steer = require(steer_channels[index]);
 input.steer_position_rad[index] = steer.position_rad;
 input.steer_velocity_radps[index] = steer.velocity_radps;
 }
 }
 const auto output = holonomic_controller_->update(command, input, dt_s);
 for (std::size_t index = 0; index < drive_channels.size(); ++index) {
 result.command.setpoints.push_back({
 drive_channels[index], runtime::ActuatorCommandMode::NormalizedDuty,
 output.drive_duty[index]});
 }
 if (holonomic_controller_->kind() == HolonomicControlKind::Swerve) {
 for (std::size_t index = 0; index < steer_channels.size(); ++index) {
 result.command.setpoints.push_back({
 steer_channels[index], runtime::ActuatorCommandMode::NormalizedDuty,
 output.steer_duty[index]});
 }
 }
 result.command.header.source_id = "robot.holonomic-controller";
 return result;
 }

 throw std::logic_error("no actuator controller has been configured");
}

void AutonomyCoordinator::configure_global_planner(
 std::shared_ptr<const ::autonomy::IGlobalPlanner> planner) {
 if (!planner)
 throw std::invalid_argument("autonomy coordinator requires a global planner");
 global_planning_ = std::make_unique<::autonomy::GlobalPlanningSystem>(std::move(planner));
}

::autonomy::GlobalPlanningSystem& AutonomyCoordinator::global_planning() {
 if (!global_planning_)
 throw std::logic_error("global planning has not been configured");
 return *global_planning_;
}

const ::autonomy::GlobalPlanningSystem& AutonomyCoordinator::global_planning() const {
 if (!global_planning_)
 throw std::logic_error("global planning has not been configured");
 return *global_planning_;
}

void AutonomyCoordinator::configure_pipeline(
 const GridMap& static_map, int width, int height, double resolution_m,
 std::uint32_t seed, ObservedObstacleLayerConfig obstacle_config,
 const runtime::IMapProvider* map_provider) {
 pipeline_ = std::make_unique<AutonomyPipeline>(
 static_map, width, height, resolution_m, seed, std::move(obstacle_config), map_provider);
}

AutonomyPipeline& AutonomyCoordinator::pipeline() {
 if (!pipeline_)
 throw std::logic_error("autonomy pipeline has not been configured");
 return *pipeline_;
}

const AutonomyPipeline& AutonomyCoordinator::pipeline() const {
 if (!pipeline_)
 throw std::logic_error("autonomy pipeline has not been configured");
 return *pipeline_;
}

void AutonomyCoordinator::reset_pipeline(std::uint32_t seed) {
 pipeline().reset(seed);
}

} // namespace sim::autonomy
