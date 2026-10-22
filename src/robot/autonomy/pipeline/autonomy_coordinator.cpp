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

RobotAutonomyCycleOutput AutonomyCoordinator::run_cycle(
 const RobotAutonomyCycleInput& input) {
 RobotAutonomyCycleOutput output;
 output.autonomy = update(input.autonomy);
 if (input.local_planning)
 output.local_planning = plan_local_motion(*input.local_planning);
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

std::size_t AutonomyCoordinator::route_path_index() const { return route_tracking().path_index(); }
void AutonomyCoordinator::set_route_path_index(std::size_t index) { route_tracking().set_path_index(index); }
bool AutonomyCoordinator::route_reversing() const { return route_tracking().reversing(); }
void AutonomyCoordinator::set_route_reversing(bool reversing) { route_tracking().set_reversing(reversing); }
void AutonomyCoordinator::reset_route_tracking() { route_tracking().reset(); }
sim::GuideOut AutonomyCoordinator::follow_route(
 const std::vector<sim::RoutePoint>& path, const sim::EstimatedState& estimate,
 double lookahead_m, double cruise_mps, double dt_s, const GridMap& planning_map,
 const ChassisCaps& chassis_caps) {
 return route_tracking().follow_route(path, estimate, lookahead_m, cruise_mps,
 dt_s, planning_map, chassis_caps);
}
TravelDirection AutonomyCoordinator::route_travel_direction(
 const std::vector<sim::RoutePoint>& path) const {
 return route_tracking().travel_direction(path);
}
bool AutonomyCoordinator::route_should_replan(
 const std::vector<sim::RoutePoint>& path, const sim::EstimatedState& estimate,
 const GridMap& local_map, const std::vector<DynamicTrack>& dynamic_tracks,
 double dt_s) {
 return route_tracking().should_replan(path, estimate, local_map, dynamic_tracks, dt_s);
}
double AutonomyCoordinator::route_heading_command(double error_rad, double dt_s) {
 return route_tracking().heading_command(error_rad, dt_s);
}
double AutonomyCoordinator::route_pid_p() const { return route_tracking().pid_p(); }
double AutonomyCoordinator::route_pid_i() const { return route_tracking().pid_i(); }
double AutonomyCoordinator::route_pid_d() const { return route_tracking().pid_d(); }

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

LidarSafetyAssessment AutonomyCoordinator::assess_lidar_safety(
 const LidarSafetyObservation& observation,
 const LidarSafetyConfig& config) const {
 if (!collision_monitor_)
 throw std::logic_error("collision monitor has not been configured");
 return collision_monitor_->assess_lidar(observation, config);
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

double AutonomyCoordinator::safe_stop_deceleration_limit(
 const ChassisCaps& chassis, const SafeStopLimits& limits) const {
 return safe_stop_linear_decel_limit(chassis, limits);
}

LocalPlanningOutput AutonomyCoordinator::plan_local_motion(
 const RobotLocalPlanningInput& input) {
 if (!pipeline_)
 throw std::logic_error("autonomy pipeline has not been configured");
 if (!input.request || !input.planning_map || !input.predicted_frames)
 throw std::invalid_argument("robot local planning transaction has missing input");
 return pipeline_->plan_local(*input.request, *input.planning_map,
 *input.predicted_frames, input.terrain_feasible,
 input.physical_collision_map, input.physical_terrain_feasible,
 input.terrain_sample);
}

void AutonomyCoordinator::set_motion_policy(const MotionPolicy& policy) {
 if (!pipeline_)
 throw std::logic_error("autonomy pipeline has not been configured");
 pipeline_->set_motion_policy(policy);
}

GridMap AutonomyCoordinator::compose_local_map(const GridMap& static_costmap,
 double now_s, double inflation_radius_m) const {
 if (!pipeline_)
 throw std::logic_error("autonomy pipeline has not been configured");
 return pipeline_->compose_local_map(static_costmap, now_s, inflation_radius_m);
}

std::vector<Vec2> AutonomyCoordinator::active_observed_obstacles(double now_s) const {
 if (!pipeline_)
 throw std::logic_error("autonomy pipeline has not been configured");
 return pipeline_->observed_obstacles().active_obstacle_centres(now_s);
}

std::vector<DynamicTrack> AutonomyCoordinator::active_dynamic_tracks(
 double time_s, double max_age_s) const {
 if (!pipeline_)
 throw std::logic_error("autonomy pipeline has not been configured");
 return pipeline_->dynamic_obstacles().active_tracks(time_s, max_age_s);
}

PredictionBatch AutonomyCoordinator::predict_dynamic_objects(
 double time_s, double horizon_s, double frame_dt_s, double max_age_s) const {
 if (!pipeline_)
 throw std::logic_error("autonomy pipeline has not been configured");
 return pipeline_->predict_dynamic_objects(time_s, horizon_s, frame_dt_s, max_age_s);
}

LidarPerceptionFrame AutonomyCoordinator::process_lidar(
 const Scan& scan, const Scan& static_reference, const LidarProjectionState& state,
 const LidarProjectionConfig& config, double track_max_age_s) {
 if (!pipeline_)
 throw std::logic_error("autonomy pipeline has not been configured");
 return pipeline_->process_lidar(scan, static_reference, state, config, track_max_age_s);
}

std::vector<Vec2> AutonomyCoordinator::latest_lidar_obstacle_points(
 double time_s, double forward_speed_mps, double maximum_range_m,
 std::size_t maximum_points) const {
 std::vector<Vec2> points;
 if (!pipeline_ || !pipeline_->latest_raw_lidar() || maximum_points == 0)
 return points;
 const auto& scan = *pipeline_->latest_raw_lidar();
 const auto& projection = pipeline_->latest_lidar_config();
 const double age_s = std::clamp(
 time_s - scan.header.stamp_s, 0.0, 0.35);
 const double shift_forward_m =
 -std::max(0.0, forward_speed_mps) * age_s;
 const std::size_t stride = std::max<std::size_t>(
 1, (static_cast<std::size_t>(std::max(0, scan.n_rays)) +
 maximum_points - 1) / maximum_points);
 points.reserve(std::min<std::size_t>(maximum_points,
 scan.ranges.size() / stride + 1));
 for (int ray = 0; ray < scan.n_rays; ray += static_cast<int>(stride)) {
 const auto index = static_cast<std::size_t>(ray);
 if (!scan.hit[index] || !std::isfinite(scan.ranges[index]))
 continue;
 const double range_m = scan.ranges[index];
 if (!(range_m > 0.05) || range_m > std::min(maximum_range_m, scan.max_range))
 continue;
 // Optional height evidence removes terrain and self returns from the
 // horizontal obstacle guard. A planar scan without height still keeps the
 // return, which is the conservative choice for a physical safety field.
 if (!scan.ray_gravity_heights_m.empty() &&
 index < scan.ray_gravity_heights_m.size() &&
 std::isfinite(scan.ray_gravity_heights_m[index]) &&
 scan.ray_gravity_heights_m[index] < -0.20) {
 continue;
 }
 const double angle = projection.yaw_in_base_rad +
 scan.angle_min_rad + scan.angle_increment_rad * static_cast<double>(ray);
 points.push_back({
 projection.position_in_base.x + range_m * std::cos(angle) +
 shift_forward_m,
 projection.position_in_base.y + range_m * std::sin(angle),
 });
 }
 return points;
}

LidarSafetyAssessment AutonomyCoordinator::assess_latest_lidar_safety(
 double time_s, double forward_speed_mps,
 const LidarSafetyConfig& config) const {
 // No scan or no configured safety monitor is an unknown sensor state. Keep
 // this query total so a composition root can fail closed before its first
 // provider frame instead of turning startup into an exception path.
 if (!pipeline_ || !collision_monitor_) {
 LidarSafetyAssessment unavailable;
 unavailable.stop_required = true;
 unavailable.measurement_valid = false;
 unavailable.forward_clearance_m = -1.0;
 return unavailable;
 }
 const auto& scan = pipeline_->latest_raw_lidar();
 if (!scan)
 return {};
 const auto& projection = pipeline_->latest_lidar_config();
 LidarSafetyObservation observation;
 observation.scan = &*scan;
 observation.position_in_base = projection.position_in_base;
 observation.yaw_in_base_rad = projection.yaw_in_base_rad;
 observation.forward_speed_mps = std::max(0.0, forward_speed_mps);
 observation.measurement_valid = std::isfinite(scan->header.stamp_s);
 const double scan_period_s = std::max(0.02, scan->scan_period_s);
 observation.measurement_fresh =
 observation.measurement_valid &&
 time_s >= scan->header.stamp_s - 1e-6 &&
 time_s - scan->header.stamp_s <= std::max(0.35, 2.0 * scan_period_s);
 return collision_monitor_->assess_lidar(observation, config);
}

KnownMapLocalizationResult AutonomyCoordinator::localize_known_map(
 const Scan& scan, Vec2 odom_position, double odom_heading_rad,
 const runtime::MapSnapshot& map, double timestamp_s) const {
 if (!pipeline_)
 throw std::logic_error("autonomy pipeline has not been configured");
 return pipeline_->localize_known_map(scan, odom_position, odom_heading_rad, map, timestamp_s);
}

autonomy::LocalizationResult AutonomyCoordinator::localize_known_map_global(
 const Scan& scan, const runtime::MapSnapshot& map, double timestamp_s) const {
 if (!pipeline_)
 throw std::logic_error("autonomy pipeline has not been configured");
 return pipeline_->localize_known_map_global(scan, map, timestamp_s);
}

CrossingBehaviorDecision AutonomyCoordinator::assess_crossing(
 const std::vector<RoutePoint>& route, std::size_t path_index, Vec2 ego_position,
 double ego_speed_mps, double ego_heading_rad,
 const std::vector<std::vector<DynamicObjectState>>& predicted_frames,
 double frame_dt_s, const CrossingSafetyEnvelope& safety,
 const CrossingExecutionCaps& execution, double decision_dt_s,
 double evaluation_stamp_s, CrossingPathFeasibility path_feasible) {
 if (!pipeline_)
 throw std::logic_error("autonomy pipeline has not been configured");
 return pipeline_->assess_crossing(route, path_index, ego_position, ego_speed_mps,
 ego_heading_rad, predicted_frames, frame_dt_s, safety, execution, decision_dt_s,
 evaluation_stamp_s, std::move(path_feasible));
}

GovernorOutput AutonomyCoordinator::update_entry_governor(const GovernorInput& input) {
 if (!pipeline_)
 throw std::logic_error("autonomy pipeline has not been configured");
 return pipeline_->update_entry_governor(input);
}

EntryWindowDecision AutonomyCoordinator::assess_entry_window(const EntryWindowRequest& request) {
 if (!pipeline_)
 throw std::logic_error("autonomy pipeline has not been configured");
 return pipeline_->assess_entry_window(request);
}

bool AutonomyCoordinator::entry_governor_holding() const {
 if (!pipeline_)
 throw std::logic_error("autonomy pipeline has not been configured");
 return pipeline_->entry_governor_holding();
}

void AutonomyCoordinator::report_crossing_execution(
 bool committed, const TrajectoryValidation& validation,
 std::uint64_t trajectory_sequence, double stamp_s) {
 if (!pipeline_)
 throw std::logic_error("autonomy pipeline has not been configured");
 pipeline_->report_crossing_execution(committed, validation, trajectory_sequence, stamp_s);
}

const CrossingExecutionFeedback& AutonomyCoordinator::crossing_execution_feedback() const {
 if (!pipeline_)
 throw std::logic_error("autonomy pipeline has not been configured");
 return pipeline_->crossing_execution_feedback();
}

TrajectoryTrackingReport AutonomyCoordinator::observe_trajectory(
 double time_s, Vec2 position, double heading_rad, double forward_mps) const {
 if (!pipeline_)
 throw std::logic_error("autonomy pipeline has not been configured");
 return pipeline_->observe_trajectory(time_s, position, heading_rad, forward_mps);
}

std::uint64_t AutonomyCoordinator::accept_trajectory(
 double generated_at_s, const ExecutableTrajectory& trajectory,
 const TrajectoryTrackingLimits& limits) {
 if (!pipeline_)
 throw std::logic_error("autonomy pipeline has not been configured");
 return pipeline_->accept_trajectory(generated_at_s, trajectory, limits);
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
