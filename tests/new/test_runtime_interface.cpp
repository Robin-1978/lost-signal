#include "runtime/replay_driver.hpp"
#include "runtime/runtime_channel.hpp"

#include <atomic>
#include <cmath>
#include <iostream>
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
 frame.encoders->header = {time_s, sequence, "base_link", 0.0, 1, std::move(source)};
 return frame;
}

sim::ObservationFrame imu_frame(double time_s, std::uint64_t sequence,
 std::string source = "test.imu") {
 sim::ObservationFrame frame;
 frame.time_s = time_s;
 frame.imu.emplace();
 frame.imu->yaw_rate_radps = 0.1;
 frame.imu->header = {time_s, sequence, "imu_link", 0.0, 1, std::move(source)};
 return frame;
}

} // namespace

int main() {
 using namespace sim::runtime;

 CHECK(kApiVersion == 1);
 CHECK(kMessageSchemaVersion == 1);
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
 CHECK(channel->descriptor().api_version == 1);
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

 channel->deactivate();
 CHECK(channel->state() == LifecycleState::Inactive);
 CHECK(channel->platform().read_motion(TimePoint::from_seconds(1.0)).safe_stop);
 return 0;
}
