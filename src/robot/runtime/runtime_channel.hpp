#pragma once

#include "robot/platform/chassis_caps.hpp"
#include "robot/platform/differential_kinematics.hpp"
#include "robot/runtime/observation.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace sim::runtime {

inline constexpr std::uint32_t kApiVersion = 2;
inline constexpr std::uint32_t kMessageSchemaVersion = kMeasurementSchemaVersion;

// Transport-neutral monotonic time. Wire adapters may copy nanoseconds
// directly; simulation converts its fixed-step seconds at the boundary.
struct TimePoint {
 std::int64_t nanoseconds{0};

 static TimePoint from_seconds(double seconds);
 double seconds() const;

 friend bool operator==(TimePoint left, TimePoint right) { return left.nanoseconds == right.nanoseconds; }
 friend bool operator!=(TimePoint left, TimePoint right) { return !(left == right); }
 friend bool operator<(TimePoint left, TimePoint right) { return left.nanoseconds < right.nanoseconds; }
 friend bool operator<=(TimePoint left, TimePoint right) { return !(right < left); }
 friend bool operator>(TimePoint left, TimePoint right) { return right < left; }
 friend bool operator>=(TimePoint left, TimePoint right) { return !(left < right); }
};

enum class LifecycleState { Unconfigured, Inactive, Active };
enum class HealthLevel { Healthy, Degraded, Fault };
enum class CommandWriteResult { Accepted, Inactive, Invalid, OutOfOrder };
enum class PlatformCommandLevel { BodyMotion, Actuator };
enum class ActuatorCommandMode {
 NormalizedDuty,
 NormalizedBrake,
 EffortNm,
 VelocityRadps,
 PositionRad
};

const char* lifecycle_state_name(LifecycleState state);
const char* health_level_name(HealthLevel level);

struct MessageHeader {
 std::uint32_t schema_version{kMessageSchemaVersion};
 TimePoint sample_time;
 TimePoint receive_time;
 std::uint64_t sequence{0};
 std::string frame_id;
 std::string source_id;
};

struct PluginDescriptor {
 std::string id;
 std::string kind;
 std::uint32_t api_version{kApiVersion};
 std::uint32_t message_schema_version{kMessageSchemaVersion};
};

struct PlatformCapabilities {
 ChassisCaps chassis;
 double max_forward_speed_mps{0.0};
 double max_lateral_speed_mps{0.0};
 PlatformCommandLevel command_level{PlatformCommandLevel::BodyMotion};

 struct ActuatorChannel {
 std::string channel_id;
 ActuatorCommandMode mode{ActuatorCommandMode::NormalizedDuty};
 double minimum{-1.0};
 double maximum{1.0};
 double safe_value{0.0};
 };
 std::vector<ActuatorChannel> actuator_channels;
};

struct RuntimeHealth {
 HealthLevel level{HealthLevel::Healthy};
 std::string message{"ready"};
 std::uint64_t dropped_observations{0};
 std::uint64_t rejected_observations{0};
 std::uint64_t rejected_commands{0};
 std::uint64_t watchdog_stops{0};
};

// One explicit message covers every shipped base without overloading a field:
// yaw-rate bases use yaw_rate_radps, Ackermann uses steering_angle_rad, and
// holonomic bases additionally use lateral_mps.
struct MotionCommand {
 MessageHeader header;
 TimePoint valid_until;
 ChassisCommandSpace command_space{ChassisCommandSpace::YawRate};
 double forward_mps{0.0};
 double lateral_mps{0.0};
 double yaw_rate_radps{0.0};
 double steering_angle_rad{0.0};
 bool safe_stop{false};
};

// Low-level LSRI command used when the deployable ROBOT stack owns the
// chassis controller. A platform declares one unambiguous mode per channel;
// SIM and native providers consume the same setpoints.
struct ActuatorCommand {
 struct Setpoint {
 std::string channel_id;
 ActuatorCommandMode mode{ActuatorCommandMode::NormalizedDuty};
 double value{0.0};
 };

 MessageHeader header;
 TimePoint valid_until;
 std::vector<Setpoint> setpoints;
 bool safe_stop{false};
};

struct RuntimeChannelConfig {
 std::string instance_id{"robot"};
 PlatformCapabilities capabilities;
 std::size_t observation_queue_capacity{64};
 double maximum_command_ttl_s{0.25};
};

bool valid_message_header(const MessageHeader& header);
bool valid_observation_frame(const ObservationFrame& observation, std::string* reason = nullptr);
bool valid_motion_command(const MotionCommand& command, const RuntimeChannelConfig& config,
                          std::string* reason = nullptr);
bool valid_actuator_command(const ActuatorCommand& command, const RuntimeChannelConfig& config,
 std::string* reason = nullptr);

MotionCommand make_motion_command(const DiffCmd& command, const ChassisCaps& capabilities, TimePoint now,
                                  std::uint64_t sequence, double valid_for_s,
                                  std::string source_id = "autonomy");
MotionCommand make_safe_stop(const ChassisCaps& capabilities, TimePoint now, std::uint64_t sequence,
                             std::string source_id = "runtime.watchdog");
ActuatorCommand make_actuator_safe_stop(const PlatformCapabilities& capabilities, TimePoint now,
 std::uint64_t sequence, std::string source_id = "runtime.watchdog");
DiffCmd to_diff_command(const MotionCommand& command);

// The autonomy side never sees simulator truth or device transports.
class IAutonomyPort {
 public:
 virtual ~IAutonomyPort() = default;
 virtual std::optional<ObservationFrame> read_observation() = 0;
 virtual CommandWriteResult write_motion(const MotionCommand& command) = 0;
 virtual CommandWriteResult write_actuators(const ActuatorCommand& command) = 0;
};

// A Bullet adapter, log replay, native hardware driver, or ROS bridge owns
// this side. It publishes measured observations and consumes commands.
class IPlatformPort {
 public:
 virtual ~IPlatformPort() = default;
 virtual bool publish_observation(ObservationFrame observation) = 0;
 virtual MotionCommand read_motion(TimePoint now) = 0;
 virtual ActuatorCommand read_actuators(TimePoint now) = 0;
};

class IRuntimeChannel {
 public:
 virtual ~IRuntimeChannel() = default;
 virtual const PluginDescriptor& descriptor() const = 0;
 virtual const RuntimeChannelConfig& config() const = 0;
 virtual LifecycleState state() const = 0;
 virtual RuntimeHealth health() const = 0;
 virtual void configure() = 0;
 virtual void activate() = 0;
 virtual void deactivate() = 0;
 virtual IAutonomyPort& autonomy() = 0;
 virtual IPlatformPort& platform() = 0;
};

using RuntimeChannelFactory = std::function<std::unique_ptr<IRuntimeChannel>(const RuntimeChannelConfig&)>;

// Explicit registration keeps static builds deterministic and small. Dynamic
// libraries and ROS belong in optional bridge packages that register a factory
// or connect to the same two ports.
class RuntimeRegistry {
 public:
 static const RuntimeRegistry& builtins();
 static RuntimeRegistry with_builtins();

 void register_channel(std::string id, RuntimeChannelFactory factory);
 bool supports(const std::string& id) const;
 std::vector<std::string> provider_ids() const;
 std::unique_ptr<IRuntimeChannel> create(const std::string& id, const RuntimeChannelConfig& config) const;

 private:
 std::map<std::string, RuntimeChannelFactory> factories_;
};

std::unique_ptr<IRuntimeChannel> make_in_process_channel(const RuntimeChannelConfig& config);

} // namespace sim::runtime
