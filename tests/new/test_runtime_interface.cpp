#include "robot/runtime/replay_driver.hpp"
#include "robot/runtime/endpoint_registry.hpp"
#include "robot/runtime/runtime_channel.hpp"

#include <atomic>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#define CHECK(condition)                                                                                     \
 do {                                                                                                        \
  if (!(condition)) {                                                                                        \
   std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n";                           \
   return 1;                                                                                                 \
  }                                                                                                          \
 } while (false)

namespace {

sim::runtime::RuntimeChannelConfig
config(sim::ChassisCommandSpace space = sim::ChassisCommandSpace::YawRate) {
 sim::runtime::RuntimeChannelConfig value;
 value.instance_id = "test-robot";
 value.observation_queue_capacity = 2;
 value.maximum_command_ttl_s = 0.2;
 value.capabilities.chassis.command_space = space;
 value.capabilities.chassis.max_yaw_rate_radps = 2.0;
 value.capabilities.chassis.geometry.steering_wheel_base_m = 0.45;
 value.capabilities.chassis.geometry.max_steer_rad = 0.6;
 value.capabilities.max_forward_speed_mps = 3.0;
 value.capabilities.max_lateral_speed_mps = space == sim::ChassisCommandSpace::HolonomicTwist ? 2.0 : 0.0;
 return value;
}

sim::ObservationFrame encoder_frame(double time_s, std::uint64_t sequence,
                                    std::string source = "test.encoder") {
 sim::ObservationFrame frame;
 frame.time_s = time_s;
 frame.encoders.emplace();
 frame.encoders->linear_velocity_mps = 0.5;
 frame.encoders->header = {time_s, sequence, "base_link", 0.0, sim::kMeasurementSchemaVersion, std::move(source)};
 return frame;
}

sim::ObservationFrame imu_frame(double time_s, std::uint64_t sequence,
 std::string source = "test.imu") {
 sim::ObservationFrame frame;
 frame.time_s = time_s;
 frame.imu.emplace();
 frame.imu->yaw_rate_radps = 0.1;
 frame.imu->header = {time_s, sequence, "imu_link", 0.0, sim::kMeasurementSchemaVersion, std::move(source)};
 return frame;
}

sim::ObservationFrame actuator_feedback_frame(double time_s,
 std::uint64_t sequence) {
 sim::ObservationFrame frame;
 frame.time_s = time_s;
 frame.actuator_feedback.emplace();
 frame.actuator_feedback->channels = {
 {"left", 1.25, 3.5},
 {"right", 1.50, 4.0},
 };
 frame.actuator_feedback->header = {
 time_s, sequence, "base_link", 0.0,
 sim::kMeasurementSchemaVersion, "test.actuator-feedback"};
 return frame;
}

} // namespace

int main() {
 using namespace sim::runtime;

 CHECK(kApiVersion == 2);
 CHECK(kMessageSchemaVersion == 2);
 CHECK(TimePoint::from_seconds(1.25).nanoseconds == 1'250'000'000);
 CHECK(std::abs(TimePoint::from_seconds(1.25).seconds() - 1.25) < 1e-12);

 const auto& builtins = RuntimeRegistry::builtins();
 CHECK(builtins.supports("channel.in-process"));
 CHECK(builtins.provider_ids().size() == 1);
 bool threw = false;
 try {
  (void)builtins.create("missing", config());
 } catch (const std::runtime_error&) {
  threw = true;
 }
 CHECK(threw);

 auto invalid_config = config(sim::ChassisCommandSpace::HolonomicTwist);
 invalid_config.capabilities.max_lateral_speed_mps = 0.0;
 auto invalid_channel = make_in_process_channel(invalid_config);
 threw = false;
 try {
 invalid_channel->configure();
 } catch (const std::invalid_argument&) {
 threw = true;
 }
 CHECK(threw);

 RuntimeRegistry custom = RuntimeRegistry::with_builtins();
 custom.register_channel("channel.fixture", make_in_process_channel);
 CHECK(custom.supports("channel.fixture"));
 threw = false;
 try {
  custom.register_channel("channel.fixture", make_in_process_channel);
 } catch (const std::runtime_error&) {
  threw = true;
 }
 CHECK(threw);

 auto channel = custom.create("channel.fixture", config());
 CHECK(channel->descriptor().api_version == 2);
 CHECK(channel->state() == LifecycleState::Unconfigured);
 const auto before_active =
   make_motion_command({}, config().capabilities.chassis, TimePoint::from_seconds(0.0), 0, 0.1);
 CHECK(channel->autonomy().write_motion(before_active) == CommandWriteResult::Inactive);
 channel->configure();
 CHECK(channel->state() == LifecycleState::Inactive);
 channel->activate();
 CHECK(channel->state() == LifecycleState::Active);

 CHECK(channel->platform().publish_observation(encoder_frame(0.0, 0)));
 const auto received = channel->autonomy().read_observation();
 CHECK(received && received->encoders);
 CHECK(received->encoders->header.source_id == "test.encoder");
 CHECK(!channel->platform().publish_observation(encoder_frame(0.1, 0)));
 CHECK(channel->health().rejected_observations == 1);

 sim::DiffCmd desired{1.25, 0.4};
 const auto command = make_motion_command(desired, config().capabilities.chassis,
                                          TimePoint::from_seconds(0.0), 1, 0.1, "test.autonomy");
 CHECK(valid_motion_command(command, config()));
 auto stale_on_arrival = command;
 stale_on_arrival.header.receive_time = TimePoint::from_seconds(0.11);
 CHECK(!valid_motion_command(stale_on_arrival, config()));
 auto excessive_speed = command;
 excessive_speed.forward_mps = 3.1;
 CHECK(!valid_motion_command(excessive_speed, config()));
 CHECK(channel->autonomy().write_motion(command) == CommandWriteResult::Accepted);
 const auto delivered = channel->platform().read_motion(TimePoint::from_seconds(0.05));
 CHECK(!delivered.safe_stop);
 CHECK(std::abs(to_diff_command(delivered).target_v - 1.25) < 1e-12);
 CHECK(std::abs(to_diff_command(delivered).target_w - 0.4) < 1e-12);
 CHECK(channel->autonomy().write_motion(command) == CommandWriteResult::OutOfOrder);
 const auto stopped = channel->platform().read_motion(TimePoint::from_seconds(0.11));
 CHECK(stopped.safe_stop);
 CHECK(to_diff_command(stopped).target_v == 0.0);
 CHECK(channel->health().watchdog_stops == 1);

 const auto steering_config = config(sim::ChassisCommandSpace::SteeringAngle);
 const sim::DiffCmd steering_input{1.0, 0.45};
 const auto steering = make_motion_command(steering_input, steering_config.capabilities.chassis,
                                           TimePoint::from_seconds(2.0), 0, 0.1);
 CHECK(steering.yaw_rate_radps == 0.0);
 CHECK(steering.steering_angle_rad == 0.45);
 CHECK(to_diff_command(steering).target_w == 0.45);

 const auto holonomic_config = config(sim::ChassisCommandSpace::HolonomicTwist);
 sim::DiffCmd holonomic_input{1.0, 0.3};
 holonomic_input.target_lateral_v = -0.75;
 const auto holonomic = make_motion_command(holonomic_input, holonomic_config.capabilities.chassis,
                                            TimePoint::from_seconds(2.0), 0, 0.1);
 CHECK(holonomic.lateral_mps == -0.75);
 CHECK(to_diff_command(holonomic).target_lateral_v == -0.75);

 auto queue_channel = make_in_process_channel(config());
 queue_channel->configure();
 queue_channel->activate();
 CHECK(queue_channel->platform().publish_observation(encoder_frame(0.0, 0, "queue.encoder")));
 CHECK(queue_channel->platform().publish_observation(encoder_frame(0.1, 1, "queue.encoder")));
 CHECK(queue_channel->platform().publish_observation(encoder_frame(0.2, 2, "queue.encoder")));
 CHECK(queue_channel->health().dropped_observations == 1);
 const auto oldest_remaining = queue_channel->autonomy().read_observation();
 CHECK(oldest_remaining && std::abs(oldest_remaining->time_s - 0.1) < 1e-12);

 // A real platform commonly has independent sensor I/O threads. The
 // in-process provider is the reference transport for that execution mode:
 // multiple producers may publish concurrently without corrupting per-source
 // ordering or the bounded mailbox.
 auto concurrent_config = config();
 concurrent_config.observation_queue_capacity = 512;
 auto concurrent_channel = make_in_process_channel(concurrent_config);
 concurrent_channel->configure();
 concurrent_channel->activate();
 std::atomic<bool> start_producers{false};
 std::atomic<bool> producers_ok{true};
 const auto publish_sensor = [&](const std::string& source, double time_offset_s, bool publish_imu) {
 while (!start_producers.load(std::memory_order_acquire))
 std::this_thread::yield();
 for (std::uint64_t sequence = 0; sequence < 128; ++sequence) {
 const double time_s = time_offset_s + static_cast<double>(sequence) * 0.01;
 auto observation = publish_imu ? imu_frame(time_s, sequence, source)
 : encoder_frame(time_s, sequence, source);
 if (!concurrent_channel->platform().publish_observation(
 std::move(observation))) {
 producers_ok.store(false, std::memory_order_release);
 }
 }
 };
 std::thread encoder_producer(publish_sensor, "concurrent.encoder", 10.0, false);
 std::thread imu_producer(publish_sensor, "concurrent.imu", 20.0, true);
 start_producers.store(true, std::memory_order_release);
 encoder_producer.join();
 imu_producer.join();
 CHECK(producers_ok.load(std::memory_order_acquire));
 std::size_t concurrent_frames = 0;
 while (concurrent_channel->autonomy().read_observation())
 ++concurrent_frames;
 CHECK(concurrent_frames == 256);
 CHECK(concurrent_channel->health().dropped_observations == 0);
 CHECK(concurrent_channel->health().rejected_observations == 0);

 std::vector<sim::ObservationFrame> replay_frames;
 replay_frames.push_back(encoder_frame(0.0, 0, "replay.encoder"));
 replay_frames.push_back(encoder_frame(0.5, 1, "replay.encoder"));
 ReplayDriver replay(std::move(replay_frames));
 auto replay_channel = make_in_process_channel(config());
 replay_channel->configure();
 replay_channel->activate();
 CHECK(replay.publish_until(TimePoint::from_seconds(0.25), replay_channel->platform()) == 1);
 CHECK(!replay.finished());
 CHECK(replay_channel->autonomy().read_observation().has_value());
 CHECK(replay.publish_until(TimePoint::from_seconds(0.5), replay_channel->platform()) == 1);
 CHECK(replay.finished());
 replay.reset();
 CHECK(!replay.finished());

 // Device-granular provider ownership permits mixed reality without allowing
 // two providers to command the same endpoint.
 EndpointRegistry endpoints;
 endpoints.register_endpoint({
 "lidar/front", "provider.native-lidar", "lidar.front", "lidar_link",
 "host.monotonic", EndpointRole::ObservationSource, EndpointKind::Lidar,
 ProviderBackend::Native, 10.0});
 endpoints.register_endpoint({
 "imu/base", "provider.sim-imu", "imu.base", "imu_link",
 "simulation.fixed", EndpointRole::ObservationSource, EndpointKind::Imu,
 ProviderBackend::Simulated, 100.0});
 endpoints.register_endpoint({
 "drive/left", "provider.sim-actuator", "drive.left", "left_wheel_link",
 "simulation.fixed", EndpointRole::CommandSink, EndpointKind::Actuator,
 ProviderBackend::Simulated, 50.0});
 CHECK(endpoints.endpoints().size() == 3);
 CHECK(endpoints.endpoint("lidar/front", EndpointRole::ObservationSource).backend ==
 ProviderBackend::Native);
 threw = false;
 try {
 endpoints.register_endpoint({
 "drive/left", "provider.native-motor", "drive.left.native", "left_wheel_link",
 "host.monotonic", EndpointRole::CommandSink, EndpointKind::Actuator,
 ProviderBackend::Native, 50.0});
 } catch (const std::runtime_error&) {
 threw = true;
 }
 CHECK(threw);

 // A deployable controller may own the low-level loop and send actuator
 // commands to either a simulated plant or a native driver.
 auto actuator_config = config();
 actuator_config.capabilities.command_level = PlatformCommandLevel::Actuator;
 actuator_config.capabilities.actuator_channels = {
 {"left", ActuatorCommandMode::NormalizedDuty, -1.0, 1.0, 0.0},
 {"right", ActuatorCommandMode::NormalizedDuty, -1.0, 1.0, 0.0},
 };
 auto actuator_channel = make_in_process_channel(actuator_config);
 actuator_channel->configure();
 actuator_channel->activate();
 CHECK(actuator_channel->platform().publish_observation(
 actuator_feedback_frame(1.0, 0)));
 const auto actuator_feedback =
 actuator_channel->autonomy().read_observation();
 CHECK(actuator_feedback && actuator_feedback->actuator_feedback);
 const auto* left_feedback =
 actuator_feedback->actuator_feedback->find("left");
 CHECK(left_feedback);
 CHECK(left_feedback->position_rad == 1.25);
 CHECK(left_feedback->velocity_radps == 3.5);
 auto duplicate_feedback = actuator_feedback_frame(1.1, 1);
 duplicate_feedback.actuator_feedback->channels[1].channel_id = "left";
 CHECK(!actuator_channel->platform().publish_observation(
 std::move(duplicate_feedback)));
 auto nonfinite_feedback = actuator_feedback_frame(1.1, 1);
 nonfinite_feedback.actuator_feedback->channels[0].velocity_radps =
 std::numeric_limits<double>::quiet_NaN();
 CHECK(!actuator_channel->platform().publish_observation(
 std::move(nonfinite_feedback)));
 ActuatorCommand actuator_command;
 actuator_command.header = {
 kMessageSchemaVersion, TimePoint::from_seconds(1.0),
 TimePoint::from_seconds(1.0), 7, "base_link", "controller.ro1"};
 actuator_command.valid_until = TimePoint::from_seconds(1.1);
 actuator_command.setpoints = {
 {"left", ActuatorCommandMode::NormalizedDuty, 0.4},
 {"right", ActuatorCommandMode::NormalizedDuty, 0.5},
 };
 CHECK(valid_actuator_command(actuator_command, actuator_config));
 CHECK(actuator_channel->autonomy().write_actuators(actuator_command) ==
 CommandWriteResult::Accepted);
 const auto actuator_delivery =
 actuator_channel->platform().read_actuators(TimePoint::from_seconds(1.05));
 CHECK(!actuator_delivery.safe_stop);
 CHECK(actuator_delivery.setpoints[1].value == 0.5);
 const auto actuator_stop =
 actuator_channel->platform().read_actuators(TimePoint::from_seconds(1.11));
 CHECK(actuator_stop.safe_stop);
 CHECK(actuator_stop.setpoints[0].value == 0.0);
 auto wrong_mode = actuator_command;
 wrong_mode.header.sequence = 8;
 wrong_mode.setpoints[0].mode = ActuatorCommandMode::EffortNm;
 CHECK(!valid_actuator_command(wrong_mode, actuator_config));
 CHECK(actuator_channel->autonomy().write_motion(command) == CommandWriteResult::Invalid);

 channel->deactivate();
 CHECK(channel->state() == LifecycleState::Inactive);
 CHECK(channel->platform().read_motion(TimePoint::from_seconds(1.0)).safe_stop);
 return 0;
}
