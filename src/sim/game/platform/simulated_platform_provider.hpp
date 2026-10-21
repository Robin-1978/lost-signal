#pragma once

#include "robot/runtime/runtime_channel.hpp"
#include "sim/hardware/vehicle/motor_drive.hpp"

#include <cstdint>
#include <string_view>

namespace sim::platform {

// SIM-only LSRI provider adapter. It is the sole place where provider-native
// MuJoCo joint feedback and actuator setpoints are translated to/from the
// transport-neutral ROBOT contract. No autonomy decision belongs here.
class SimulatedPlatformProviderAdapter {
 public:
 static ObservationFrame make_actuator_feedback(
 const ChassisActuatorFeedback& feedback, std::string_view drive_type,
 double sample_time_s, std::uint64_t sequence);

 static bool publish_actuator_feedback(
 runtime::IRuntimeChannel& channel, const ChassisActuatorFeedback& feedback,
 std::string_view drive_type, double sample_time_s, std::uint64_t sequence);

 static ChassisActuatorDutyCommand consume_actuator_command(
 const runtime::ActuatorCommand& command);
};

} // namespace sim::platform
