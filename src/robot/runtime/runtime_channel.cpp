#include "runtime_channel.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <utility>

namespace sim::runtime {
namespace {

constexpr double kNanosecondsPerSecond = 1'000'000'000.0;

void set_reason(std::string* reason, std::string value) {
 if (reason) {
  *reason = std::move(value);
 }
}

bool finite_nonnegative(double value) {
 return std::isfinite(value) && value >= 0.0;
}

bool valid_platform_capabilities(const PlatformCapabilities& capabilities) {
 if (!std::isfinite(capabilities.max_forward_speed_mps) || capabilities.max_forward_speed_mps <= 0.0 ||
 !finite_nonnegative(capabilities.max_lateral_speed_mps)) {
 return false;
 }
 bool chassis_valid = false;
 switch (capabilities.chassis.command_space) {
 case ChassisCommandSpace::YawRate:
 chassis_valid = capabilities.max_lateral_speed_mps == 0.0 &&
 std::isfinite(capabilities.chassis.max_yaw_rate_radps) &&
 capabilities.chassis.max_yaw_rate_radps > 0.0;
 break;
 case ChassisCommandSpace::SteeringAngle:
 chassis_valid = capabilities.max_lateral_speed_mps == 0.0 &&
 std::isfinite(capabilities.chassis.geometry.steering_wheel_base_m) &&
 capabilities.chassis.geometry.steering_wheel_base_m > 0.0 &&
 std::isfinite(capabilities.chassis.geometry.max_steer_rad) &&
 capabilities.chassis.geometry.max_steer_rad > 0.0;
 break;
 case ChassisCommandSpace::HolonomicTwist:
 chassis_valid = capabilities.max_lateral_speed_mps > 0.0 &&
 std::isfinite(capabilities.chassis.max_yaw_rate_radps) &&
 capabilities.chassis.max_yaw_rate_radps > 0.0;
 break;
 }
 if (!chassis_valid)
 return false;
 if (capabilities.command_level == PlatformCommandLevel::BodyMotion)
 return capabilities.actuator_channels.empty();
 if (capabilities.actuator_channels.empty())
 return false;
 std::map<std::string, bool> channel_ids;
 for (const auto& channel : capabilities.actuator_channels) {
 if (channel.channel_id.empty() || !std::isfinite(channel.minimum) ||
 !std::isfinite(channel.maximum) || !std::isfinite(channel.safe_value) ||
 channel.minimum > channel.maximum || channel.safe_value < channel.minimum ||
 channel.safe_value > channel.maximum ||
 !channel_ids.emplace(channel.channel_id, true).second) {
 return false;
 }
 }
 return true;
}

bool valid_measurement_header(const MeasurementHeader& header, double frame_time_s, std::string* reason) {
 if (header.schema_version != kMessageSchemaVersion) {
  set_reason(reason, "unsupported measurement schema version");
  return false;
 }
 if (!finite_nonnegative(header.stamp_s) || !finite_nonnegative(header.latency_s) ||
     std::abs(header.stamp_s - frame_time_s) > 1e-9) {
  set_reason(reason, "measurement time does not match its observation frame");
  return false;
 }
 if (header.frame_id.empty() || header.source_id.empty()) {
  set_reason(reason, "measurement frame and source must be named");
  return false;
 }
 return true;
}

std::vector<const MeasurementHeader*> measurement_headers(const ObservationFrame& observation) {
 std::vector<const MeasurementHeader*> headers;
 if (observation.lidar)
  headers.push_back(&observation.lidar->header);
 if (observation.lidar_point_cloud)
 headers.push_back(&observation.lidar_point_cloud->header);
 if (observation.lidar_static_reference)
  headers.push_back(&observation.lidar_static_reference->header);
 if (observation.encoders)
  headers.push_back(&observation.encoders->header);
 if (observation.actuator_feedback)
 headers.push_back(&observation.actuator_feedback->header);
 if (observation.imu)
  headers.push_back(&observation.imu->header);
 if (observation.absolute_pose)
  headers.push_back(&observation.absolute_pose->header);
 return headers;
}

struct SharedState {
 explicit SharedState(RuntimeChannelConfig value) : config(std::move(value)) {}

 RuntimeChannelConfig config;
 LifecycleState lifecycle{LifecycleState::Unconfigured};
 RuntimeHealth health;
 std::deque<ObservationFrame> observations;
 std::map<std::string, std::uint64_t> observation_sequences;
 std::optional<MotionCommand> latest_command;
 std::optional<std::uint64_t> last_command_sequence;
 bool watchdog_active{false};
 std::optional<ActuatorCommand> latest_actuator_command;
 std::optional<std::uint64_t> last_actuator_sequence;
 bool actuator_watchdog_active{false};
 mutable std::mutex mutex;
};

class InProcessAutonomyPort final : public IAutonomyPort {
 public:
 explicit InProcessAutonomyPort(std::shared_ptr<SharedState> state) : state_(std::move(state)) {}

 std::optional<ObservationFrame> read_observation() override {
  std::lock_guard lock(state_->mutex);
  if (state_->lifecycle != LifecycleState::Active || state_->observations.empty()) {
   return std::nullopt;
  }
  ObservationFrame output = std::move(state_->observations.front());
  state_->observations.pop_front();
  return output;
 }

 CommandWriteResult write_motion(const MotionCommand& command) override {
  std::lock_guard lock(state_->mutex);
  if (state_->lifecycle != LifecycleState::Active) {
   ++state_->health.rejected_commands;
   state_->health.level = HealthLevel::Degraded;
   state_->health.message = "command rejected while runtime channel is inactive";
   return CommandWriteResult::Inactive;
  }
  std::string reason;
  if (!valid_motion_command(command, state_->config, &reason)) {
   ++state_->health.rejected_commands;
   state_->health.level = HealthLevel::Degraded;
   state_->health.message = std::move(reason);
   return CommandWriteResult::Invalid;
  }
  if (state_->last_command_sequence && command.header.sequence <= *state_->last_command_sequence) {
   ++state_->health.rejected_commands;
   state_->health.level = HealthLevel::Degraded;
   state_->health.message = "motion command sequence is not monotonic";
   return CommandWriteResult::OutOfOrder;
  }
  state_->last_command_sequence = command.header.sequence;
  state_->latest_command = command;
  state_->watchdog_active = false;
  state_->health.level = HealthLevel::Healthy;
  state_->health.message = "active";
  return CommandWriteResult::Accepted;
 }

 CommandWriteResult write_actuators(const ActuatorCommand& command) override {
 std::lock_guard lock(state_->mutex);
 if (state_->lifecycle != LifecycleState::Active) {
 ++state_->health.rejected_commands;
 state_->health.level = HealthLevel::Degraded;
 state_->health.message = "actuator command rejected while runtime channel is inactive";
 return CommandWriteResult::Inactive;
 }
 std::string reason;
 if (!valid_actuator_command(command, state_->config, &reason)) {
 ++state_->health.rejected_commands;
 state_->health.level = HealthLevel::Degraded;
 state_->health.message = std::move(reason);
 return CommandWriteResult::Invalid;
 }
 if (state_->last_actuator_sequence &&
 command.header.sequence <= *state_->last_actuator_sequence) {
 ++state_->health.rejected_commands;
 state_->health.level = HealthLevel::Degraded;
 state_->health.message = "actuator command sequence is not monotonic";
 return CommandWriteResult::OutOfOrder;
 }
 state_->last_actuator_sequence = command.header.sequence;
 state_->latest_actuator_command = command;
 state_->actuator_watchdog_active = false;
 state_->health.level = HealthLevel::Healthy;
 state_->health.message = "active";
 return CommandWriteResult::Accepted;
 }

 private:
 std::shared_ptr<SharedState> state_;
};

class InProcessPlatformPort final : public IPlatformPort {
 public:
 explicit InProcessPlatformPort(std::shared_ptr<SharedState> state) : state_(std::move(state)) {}

 bool publish_observation(ObservationFrame observation) override {
  std::lock_guard lock(state_->mutex);
  if (state_->lifecycle != LifecycleState::Active) {
   ++state_->health.rejected_observations;
   state_->health.level = HealthLevel::Degraded;
   state_->health.message = "observation rejected while runtime channel is inactive";
   return false;
  }
  std::string reason;
  if (!valid_observation_frame(observation, &reason)) {
   ++state_->health.rejected_observations;
   state_->health.level = HealthLevel::Degraded;
   state_->health.message = std::move(reason);
   return false;
  }
  std::map<std::string, std::uint64_t> frame_sequences;
  for (const auto* header : measurement_headers(observation)) {
   const auto duplicate = frame_sequences.find(header->source_id);
   if (duplicate != frame_sequences.end()) {
    if (duplicate->second != header->sequence) {
     ++state_->health.rejected_observations;
     state_->health.level = HealthLevel::Degraded;
     state_->health.message = "one observation frame contains conflicting source sequences";
     return false;
    }
    continue;
   }
   const auto previous = state_->observation_sequences.find(header->source_id);
   if (previous != state_->observation_sequences.end() && header->sequence <= previous->second) {
    ++state_->health.rejected_observations;
    state_->health.level = HealthLevel::Degraded;
    state_->health.message = "observation sequence is not monotonic for source '" + header->source_id + "'";
    return false;
   }
   frame_sequences.emplace(header->source_id, header->sequence);
  }
  for (const auto& [source, sequence] : frame_sequences) {
   state_->observation_sequences[source] = sequence;
  }
  if (state_->observations.size() == state_->config.observation_queue_capacity) {
   state_->observations.pop_front();
   ++state_->health.dropped_observations;
   state_->health.level = HealthLevel::Degraded;
   state_->health.message = "oldest observation dropped because the runtime queue is full";
  }
  state_->observations.push_back(std::move(observation));
  return true;
 }

 MotionCommand read_motion(TimePoint now) override {
  std::lock_guard lock(state_->mutex);
  const bool active = state_->lifecycle == LifecycleState::Active;
  const bool fresh = active && state_->latest_command && now <= state_->latest_command->valid_until;
  if (fresh) {
   state_->watchdog_active = false;
   return *state_->latest_command;
  }
  if (!state_->watchdog_active) {
   state_->watchdog_active = true;
   ++state_->health.watchdog_stops;
   if (active) {
    state_->health.level = HealthLevel::Degraded;
    state_->health.message = "motion watchdog issued a safe stop";
   }
  }
  const std::uint64_t sequence = state_->last_command_sequence ? *state_->last_command_sequence + 1 : 0;
  return make_safe_stop(state_->config.capabilities.chassis, now, sequence);
 }

 ActuatorCommand read_actuators(TimePoint now) override {
 std::lock_guard lock(state_->mutex);
 const bool active = state_->lifecycle == LifecycleState::Active;
 const bool fresh = active && state_->latest_actuator_command &&
 now <= state_->latest_actuator_command->valid_until;
 if (fresh) {
 state_->actuator_watchdog_active = false;
 return *state_->latest_actuator_command;
 }
 if (!state_->actuator_watchdog_active) {
 state_->actuator_watchdog_active = true;
 ++state_->health.watchdog_stops;
 if (active) {
 state_->health.level = HealthLevel::Degraded;
 state_->health.message = "actuator watchdog issued a safe command";
 }
 }
 const std::uint64_t sequence = state_->last_actuator_sequence
 ? *state_->last_actuator_sequence + 1 : 0;
 return make_actuator_safe_stop(state_->config.capabilities, now, sequence);
 }

 private:
 std::shared_ptr<SharedState> state_;
};

class InProcessRuntimeChannel final : public IRuntimeChannel {
 public:
 explicit InProcessRuntimeChannel(RuntimeChannelConfig config)
     : state_(std::make_shared<SharedState>(std::move(config))), autonomy_(state_), platform_(state_) {
  descriptor_.id = "channel.in-process";
  descriptor_.kind = "runtime-channel";
 }

 const PluginDescriptor& descriptor() const override { return descriptor_; }
 const RuntimeChannelConfig& config() const override { return state_->config; }
 LifecycleState state() const override {
  std::lock_guard lock(state_->mutex);
  return state_->lifecycle;
 }
 RuntimeHealth health() const override {
  std::lock_guard lock(state_->mutex);
  return state_->health;
 }

 void configure() override {
  std::lock_guard lock(state_->mutex);
  if (state_->lifecycle != LifecycleState::Unconfigured) {
   throw std::logic_error("runtime channel can only be configured once");
  }
 if (state_->config.instance_id.empty() || state_->config.observation_queue_capacity == 0 ||
 !std::isfinite(state_->config.maximum_command_ttl_s) || state_->config.maximum_command_ttl_s <= 0.0 ||
 !valid_platform_capabilities(state_->config.capabilities)) {
   throw std::invalid_argument("invalid runtime channel configuration");
  }
  state_->lifecycle = LifecycleState::Inactive;
  state_->health = {};
  state_->health.message = "configured";
 }

 void activate() override {
  std::lock_guard lock(state_->mutex);
  if (state_->lifecycle != LifecycleState::Inactive) {
   throw std::logic_error("runtime channel must be inactive before activation");
  }
  state_->lifecycle = LifecycleState::Active;
  state_->health.level = HealthLevel::Healthy;
  state_->health.message = "active";
 }

 void deactivate() override {
  std::lock_guard lock(state_->mutex);
  if (state_->lifecycle == LifecycleState::Unconfigured) {
   throw std::logic_error("unconfigured runtime channel cannot be deactivated");
  }
  state_->lifecycle = LifecycleState::Inactive;
  state_->latest_command.reset();
 state_->latest_actuator_command.reset();
  state_->observations.clear();
  state_->health.message = "inactive";
 }

 IAutonomyPort& autonomy() override { return autonomy_; }
 IPlatformPort& platform() override { return platform_; }

 private:
 PluginDescriptor descriptor_;
 std::shared_ptr<SharedState> state_;
 InProcessAutonomyPort autonomy_;
 InProcessPlatformPort platform_;
};

} // namespace

TimePoint TimePoint::from_seconds(double seconds) {
 if (!finite_nonnegative(seconds) ||
     seconds > static_cast<double>(std::numeric_limits<std::int64_t>::max()) / kNanosecondsPerSecond) {
  throw std::invalid_argument("runtime time must be finite, non-negative, and representable");
 }
 return {static_cast<std::int64_t>(std::llround(seconds * kNanosecondsPerSecond))};
}

double TimePoint::seconds() const {
 return static_cast<double>(nanoseconds) / kNanosecondsPerSecond;
}

const char* lifecycle_state_name(LifecycleState state) {
 switch (state) {
 case LifecycleState::Unconfigured:
  return "unconfigured";
 case LifecycleState::Inactive:
  return "inactive";
 case LifecycleState::Active:
  return "active";
 }
 return "unknown";
}

const char* health_level_name(HealthLevel level) {
 switch (level) {
 case HealthLevel::Healthy:
  return "healthy";
 case HealthLevel::Degraded:
  return "degraded";
 case HealthLevel::Fault:
  return "fault";
 }
 return "unknown";
}

bool valid_message_header(const MessageHeader& header) {
 return header.schema_version == kMessageSchemaVersion && header.sample_time.nanoseconds >= 0 &&
        header.receive_time >= header.sample_time && !header.frame_id.empty() && !header.source_id.empty();
}

bool valid_observation_frame(const ObservationFrame& observation, std::string* reason) {
 if (!finite_nonnegative(observation.time_s)) {
  set_reason(reason, "observation time must be finite and non-negative");
  return false;
 }
 const auto headers = measurement_headers(observation);
 if (headers.empty()) {
  set_reason(reason, "observation frame contains no measurements");
  return false;
 }
 for (const auto* header : headers) {
  if (!valid_measurement_header(*header, observation.time_s, reason)) {
   return false;
  }
 }
 if (observation.lidar) {
  const auto& scan = *observation.lidar;
  const auto rays = static_cast<std::size_t>(std::max(0, scan.n_rays));
  if (scan.n_rays <= 0 || !finite_nonnegative(scan.max_range) || scan.max_range <= 0.0 ||
      scan.ranges.size() != rays || scan.hit.size() != rays ||
      (!scan.intensities.empty() && scan.intensities.size() != rays)) {
   set_reason(reason, "lidar payload dimensions or range are invalid");
   return false;
  }
 }
 if (observation.lidar_point_cloud) {
 const auto& cloud = *observation.lidar_point_cloud;
 if (cloud.pattern != LidarPatternKind::MultiBeamSpinning || cloud.rings == 0 ||
 cloud.azimuth_samples == 0 || !finite_nonnegative(cloud.max_range_m) ||
 cloud.max_range_m <= 0.0 || !finite_nonnegative(cloud.scan_period_s) ||
 cloud.scan_period_s <= 0.0 ||
 cloud.points.size() > static_cast<std::size_t>(cloud.rings) * cloud.azimuth_samples) {
 set_reason(reason, "lidar point cloud dimensions or timing are invalid");
 return false;
 }
 for (const auto& point : cloud.points) {
 if (!std::isfinite(point.x_m) || !std::isfinite(point.y_m) ||
 !std::isfinite(point.z_m) || !finite_nonnegative(point.intensity) ||
 point.intensity > 1.0 || !std::isfinite(point.time_offset_s) ||
 point.time_offset_s > 1e-9 ||
 point.time_offset_s < -cloud.scan_period_s - 1e-9 ||
 point.ring >= cloud.rings) {
 set_reason(reason, "lidar point cloud contains an invalid return");
 return false;
 }
 }
 }
 if (observation.lidar_static_reference && !observation.lidar) {
  set_reason(reason, "static lidar reference requires a lidar measurement");
  return false;
 }
 if (observation.imu) {
 const auto& imu = *observation.imu;
 if (!std::isfinite(imu.roll_rate_radps) || !std::isfinite(imu.pitch_rate_radps) ||
 !std::isfinite(imu.yaw_rate_radps) || !std::isfinite(imu.forward_acceleration_mps2) ||
 !std::isfinite(imu.left_acceleration_mps2) || !std::isfinite(imu.up_acceleration_mps2) ||
 !finite_nonnegative(imu.noise_sigma_radps) ||
 !finite_nonnegative(imu.acceleration_noise_sigma_mps2)) {
 set_reason(reason, "imu payload contains a non-finite value or invalid noise sigma");
 return false;
 }
 }
 if (observation.actuator_feedback) {
 const auto& feedback = *observation.actuator_feedback;
 if (feedback.channels.empty()) {
 set_reason(reason, "actuator feedback contains no channels");
 return false;
 }
 std::map<std::string, bool> channel_ids;
 for (const auto& channel : feedback.channels) {
 if (channel.channel_id.empty() || !std::isfinite(channel.position_rad) ||
 !std::isfinite(channel.velocity_radps) ||
 !channel_ids.emplace(channel.channel_id, true).second) {
 set_reason(reason, "actuator feedback contains an invalid or duplicate channel");
 return false;
 }
 }
 }
 return true;
}

bool valid_motion_command(const MotionCommand& command, const RuntimeChannelConfig& config,
                          std::string* reason) {
 if (config.capabilities.command_level != PlatformCommandLevel::BodyMotion) {
 set_reason(reason, "platform does not accept body-motion commands");
 return false;
 }
 if (!valid_message_header(command.header)) {
  set_reason(reason, "motion command header is invalid");
  return false;
 }
 if (command.header.frame_id != "base_link" || command.valid_until < command.header.receive_time) {
  set_reason(reason, "motion command frame or validity interval is invalid");
  return false;
 }
 const double ttl_s =
   (command.valid_until.nanoseconds - command.header.sample_time.nanoseconds) / kNanosecondsPerSecond;
 if (ttl_s > config.maximum_command_ttl_s + 1e-12) {
  set_reason(reason, "motion command validity exceeds the platform watchdog limit");
  return false;
 }
 if (!std::isfinite(command.forward_mps) || !std::isfinite(command.lateral_mps) ||
     !std::isfinite(command.yaw_rate_radps) || !std::isfinite(command.steering_angle_rad)) {
  set_reason(reason, "motion command contains a non-finite value");
  return false;
 }
 if (std::abs(command.forward_mps) > config.capabilities.max_forward_speed_mps + 1e-9 ||
     std::abs(command.lateral_mps) > config.capabilities.max_lateral_speed_mps + 1e-9) {
  set_reason(reason, "motion command exceeds declared linear capabilities");
  return false;
 }
 if (command.command_space != config.capabilities.chassis.command_space) {
  set_reason(reason, "motion command space does not match the platform capability contract");
  return false;
 }
 switch (command.command_space) {
 case ChassisCommandSpace::YawRate:
  if (std::abs(command.lateral_mps) > 1e-12 || std::abs(command.steering_angle_rad) > 1e-12 ||
      std::abs(command.yaw_rate_radps) > config.capabilities.chassis.max_yaw_rate_radps + 1e-9) {
   set_reason(reason, "yaw-rate command is incompatible with platform capabilities");
   return false;
  }
  break;
 case ChassisCommandSpace::SteeringAngle:
  if (std::abs(command.lateral_mps) > 1e-12 || std::abs(command.yaw_rate_radps) > 1e-12 ||
      std::abs(command.steering_angle_rad) > config.capabilities.chassis.geometry.max_steer_rad + 1e-9) {
   set_reason(reason, "steering command is incompatible with platform capabilities");
   return false;
  }
  break;
 case ChassisCommandSpace::HolonomicTwist:
  if (std::abs(command.steering_angle_rad) > 1e-12 ||
      std::abs(command.yaw_rate_radps) > config.capabilities.chassis.max_yaw_rate_radps + 1e-9) {
   set_reason(reason, "holonomic command is incompatible with platform capabilities");
   return false;
  }
  break;
 }
 if (command.safe_stop &&
     (std::abs(command.forward_mps) > 1e-12 || std::abs(command.lateral_mps) > 1e-12 ||
      std::abs(command.yaw_rate_radps) > 1e-12 || std::abs(command.steering_angle_rad) > 1e-12)) {
  set_reason(reason, "safe-stop command must contain zero motion");
  return false;
 }
 return true;
}

bool valid_actuator_command(const ActuatorCommand& command,
 const RuntimeChannelConfig& config, std::string* reason) {
 if (config.capabilities.command_level != PlatformCommandLevel::Actuator) {
 set_reason(reason, "platform does not accept actuator commands");
 return false;
 }
 if (!valid_message_header(command.header)) {
 set_reason(reason, "actuator command header is invalid");
 return false;
 }
 if (command.header.frame_id != "base_link" ||
 command.valid_until < command.header.receive_time) {
 set_reason(reason, "actuator command frame or validity interval is invalid");
 return false;
 }
 const double ttl_s = (command.valid_until.nanoseconds -
 command.header.sample_time.nanoseconds) / kNanosecondsPerSecond;
 if (ttl_s > config.maximum_command_ttl_s + 1e-12) {
 set_reason(reason, "actuator command validity exceeds the platform watchdog limit");
 return false;
 }
 if (command.setpoints.size() != config.capabilities.actuator_channels.size()) {
 set_reason(reason, "actuator command must set every declared channel exactly once");
 return false;
 }
 std::map<std::string, bool> seen;
 for (const auto& setpoint : command.setpoints) {
 const auto capability = std::find_if(
 config.capabilities.actuator_channels.begin(),
 config.capabilities.actuator_channels.end(),
 [&](const auto& value) { return value.channel_id == setpoint.channel_id; });
 if (capability == config.capabilities.actuator_channels.end() ||
 !seen.emplace(setpoint.channel_id, true).second ||
 capability->mode != setpoint.mode || !std::isfinite(setpoint.value) ||
 setpoint.value < capability->minimum || setpoint.value > capability->maximum) {
 set_reason(reason, "actuator setpoint violates its declared channel capability");
 return false;
 }
 if (command.safe_stop && std::abs(setpoint.value - capability->safe_value) > 1e-12) {
 set_reason(reason, "safe actuator command must use every channel's safe value");
 return false;
 }
 }
 return true;
}

MotionCommand make_motion_command(const DiffCmd& command, const ChassisCaps& capabilities, TimePoint now,
                                  std::uint64_t sequence, double valid_for_s, std::string source_id) {
 if (!finite_nonnegative(valid_for_s)) {
  throw std::invalid_argument("motion command validity must be finite and non-negative");
 }
 MotionCommand output;
 output.header.sample_time = now;
 output.header.receive_time = now;
 output.header.sequence = sequence;
 output.header.frame_id = "base_link";
 output.header.source_id = std::move(source_id);
 output.valid_until = TimePoint::from_seconds(now.seconds() + valid_for_s);
 output.command_space = capabilities.command_space;
 output.forward_mps = command.target_v;
 if (capabilities.command_space == ChassisCommandSpace::SteeringAngle) {
  output.steering_angle_rad = command.target_w;
 } else {
  output.yaw_rate_radps = command.target_w;
 }
 if (capabilities.command_space == ChassisCommandSpace::HolonomicTwist) {
  output.lateral_mps = command.target_lateral_v;
 }
 return output;
}

MotionCommand make_safe_stop(const ChassisCaps& capabilities, TimePoint now, std::uint64_t sequence,
                             std::string source_id) {
 MotionCommand output = make_motion_command({}, capabilities, now, sequence, 0.0, std::move(source_id));
 output.safe_stop = true;
 return output;
}

ActuatorCommand make_actuator_safe_stop(const PlatformCapabilities& capabilities,
 TimePoint now, std::uint64_t sequence, std::string source_id) {
 ActuatorCommand output;
 output.header.sample_time = now;
 output.header.receive_time = now;
 output.header.sequence = sequence;
 output.header.frame_id = "base_link";
 output.header.source_id = std::move(source_id);
 output.valid_until = now;
 output.safe_stop = true;
 output.setpoints.reserve(capabilities.actuator_channels.size());
 for (const auto& channel : capabilities.actuator_channels) {
 output.setpoints.push_back({channel.channel_id, channel.mode, channel.safe_value});
 }
 return output;
}

DiffCmd to_diff_command(const MotionCommand& command) {
 DiffCmd output;
 output.target_v = command.forward_mps;
 output.target_lateral_v =
   command.command_space == ChassisCommandSpace::HolonomicTwist ? command.lateral_mps : 0.0;
 output.target_w = command.command_space == ChassisCommandSpace::SteeringAngle ? command.steering_angle_rad
                                                                               : command.yaw_rate_radps;
 return output;
}

const RuntimeRegistry& RuntimeRegistry::builtins() {
 static const RuntimeRegistry registry = with_builtins();
 return registry;
}

RuntimeRegistry RuntimeRegistry::with_builtins() {
 RuntimeRegistry registry;
 registry.register_channel("channel.in-process", make_in_process_channel);
 return registry;
}

void RuntimeRegistry::register_channel(std::string id, RuntimeChannelFactory factory) {
 if (id.empty() || !factory) {
  throw std::invalid_argument("runtime provider id and factory must be non-empty");
 }
 if (!factories_.emplace(std::move(id), std::move(factory)).second) {
  throw std::runtime_error("runtime provider is already registered");
 }
}

bool RuntimeRegistry::supports(const std::string& id) const {
 return factories_.contains(id);
}

std::vector<std::string> RuntimeRegistry::provider_ids() const {
 std::vector<std::string> result;
 result.reserve(factories_.size());
 for (const auto& [id, factory] : factories_) {
  (void)factory;
  result.push_back(id);
 }
 return result;
}

std::unique_ptr<IRuntimeChannel> RuntimeRegistry::create(const std::string& id,
                                                         const RuntimeChannelConfig& config) const {
 const auto found = factories_.find(id);
 if (found == factories_.end()) {
  throw std::runtime_error("unsupported runtime provider '" + id + "'");
 }
 auto channel = found->second(config);
 if (!channel || channel->descriptor().api_version != kApiVersion ||
     channel->descriptor().message_schema_version != kMessageSchemaVersion) {
  throw std::runtime_error("runtime provider '" + id + "' has an incompatible API version");
 }
 return channel;
}

std::unique_ptr<IRuntimeChannel> make_in_process_channel(const RuntimeChannelConfig& config) {
 return std::make_unique<InProcessRuntimeChannel>(config);
}

} // namespace sim::runtime
