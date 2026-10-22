#include "robot/runtime/robot_application.hpp"

#include <iostream>
#include <utility>
#include <vector>

#define CHECK(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

int main() {
 sim::robot::RobotApplicationConfig config;
 config.channel.instance_id = "test-robot-process";
 config.channel.capabilities.chassis.command_space = sim::ChassisCommandSpace::YawRate;
 config.channel.capabilities.chassis.max_yaw_rate_radps = 2.0;
 config.channel.capabilities.max_forward_speed_mps = 1.0;
 config.endpoints.register_endpoint({
 "encoder/base", "sim.encoder", "sim.encoder", "base_link", "sim-clock",
 sim::runtime::EndpointRole::ObservationSource,
 sim::runtime::EndpointKind::Encoder, sim::runtime::ProviderBackend::Simulated,
 50.0});
 sim::robot::RobotApplication application(std::move(config));

 CHECK(application.state() == sim::runtime::LifecycleState::Unconfigured);
 application.configure();
 CHECK(application.state() == sim::runtime::LifecycleState::Inactive);
 application.activate();
 CHECK(application.state() == sim::runtime::LifecycleState::Active);
 CHECK(application.health().level == sim::runtime::HealthLevel::Healthy);
 sim::ObservationFrame observation;
 observation.time_s = 0.0;
 observation.encoders = sim::EncoderObservation{};
 observation.encoders->header = {0.0, 1, "base_link", 0.0,
 sim::kMeasurementSchemaVersion, "sim.encoder"};
 CHECK(application.publish_observation(observation, sim::runtime::TimePoint::from_seconds(0.0)));
 CHECK(application.autonomy_port().read_observation().has_value());
 application.check_provider_liveness(sim::runtime::TimePoint::from_seconds(0.01));
 CHECK(application.conformance().report(sim::runtime::TimePoint::from_seconds(0.01)).failures.empty());

 // The native composition root owns the same local-planning transaction used
 // by SIM. This check intentionally supplies only provider-neutral ROBOT
 // contracts; no scenario or physics type is needed to execute the cycle.
 sim::GridMap planning_map(20, 20, 0.5);
 application.configure_pipeline(planning_map, 20, 20, 0.5, 17);
executive::TaskPlan task;
task.request_id = "tick-task";
task.goals.push_back({"goal", {3.0, 1.0}, 0.2, true, false, 0.0, 0.5});
CHECK(application.executive().submit(std::move(task)));
 sim::LocalPlanningRequest request;
 request.command_space = sim::LocalCommandSpace::DifferentialTwist;
 request.position = {1.0, 1.0};
 request.target = {3.0, 1.0};
 request.max_speed_mps = 0.5;
 request.max_yaw_rate_radps = 1.0;
 request.max_accel_mps2 = 0.5;
 request.max_decel_mps2 = 0.5;
 request.max_yaw_accel_radps2 = 1.0;
 request.prediction_horizon_s = 0.5;
 request.step_s = 0.1;
 request.robot_radius_m = 0.2;
 std::vector<std::vector<sim::DynamicObjectState>> predicted;
 sim::autonomy::RobotLocalPlanningInput local_input;
 local_input.request = &request;
 local_input.planning_map = &planning_map;
 local_input.predicted_frames = &predicted;
 sim::robot::RobotCycleInput cycle_input;
 cycle_input.local_planning = &local_input;
 cycle_input.autonomy.safety.fallback_target = request.target;
 cycle_input.autonomy.safety.map_size = {10.0, 10.0};
 cycle_input.autonomy.safety.subgoal_is_safe = [](sim::Vec2) { return true; };
cycle_input.task_observation = executive::TaskObservation{
 {1.0, 1.0}, 0.0, true, 0.02, true, 1.0, 0.0};
 cycle_input.dt_s = 0.02;
 cycle_input.now = sim::runtime::TimePoint::from_seconds(0.02);
 cycle_input.valid_until = sim::runtime::TimePoint::from_seconds(0.1);
 const auto cycle = application.tick(cycle_input);
 CHECK(cycle.local_planning.has_value());
CHECK(application.executive().status().task_state == executive::TaskState::Active);

 application.deactivate();
 CHECK(application.state() == sim::runtime::LifecycleState::Inactive);
 return 0;
}
