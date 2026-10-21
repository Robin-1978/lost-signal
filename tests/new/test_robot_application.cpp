#include "robot/runtime/robot_application.hpp"

#include <iostream>
#include <utility>

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
 application.deactivate();
 CHECK(application.state() == sim::runtime::LifecycleState::Inactive);
 return 0;
}
