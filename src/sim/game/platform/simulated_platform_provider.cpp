#include "sim/game/platform/simulated_platform_provider.hpp"

#include <optional>
#include <stdexcept>

namespace sim::platform {

ObservationFrame SimulatedPlatformProviderAdapter::make_actuator_feedback(
 const ChassisActuatorFeedback& feedback, std::string_view drive_type,
 double sample_time_s, std::uint64_t sequence) {
 ObservationFrame frame;
 frame.time_s = sample_time_s;
 frame.actuator_feedback.emplace();
 const auto average_track_side = [&](const char* front, const char* middle,
 const char* rear) -> std::optional<ChassisActuatorFeedback::JointSample> {
 const auto* first = feedback.find(front);
 const auto* second = feedback.find(middle);
 const auto* third = feedback.find(rear);
 if (!first || !second || !third)
 return std::nullopt;
 return ChassisActuatorFeedback::JointSample{
 "", (first->position_rad + second->position_rad + third->position_rad) / 3.0,
 (first->velocity_radps + second->velocity_radps + third->velocity_radps) / 3.0};
 };
 if (drive_type == "tracked" || drive_type == "differential") {
 const auto tracked_left = drive_type == "tracked"
 ? average_track_side("front_left_drive", "middle_left_drive", "rear_left_drive")
 : std::nullopt;
 const auto tracked_right = drive_type == "tracked"
 ? average_track_side("front_right_drive", "middle_right_drive", "rear_right_drive")
 : std::nullopt;
 const auto* left = tracked_left ? &*tracked_left : feedback.find("left");
 const auto* right = tracked_right ? &*tracked_right : feedback.find("right");
 if (!left || !right)
 throw std::runtime_error("differential provider did not expose both drive sides");
 frame.actuator_feedback->channels = {
 {"left", left->position_rad, left->velocity_radps},
 {"right", right->position_rad, right->velocity_radps},
 };
 for (const auto& joint : feedback.joints) {
 if (joint.channel == "left" || joint.channel == "right")
 continue;
 frame.actuator_feedback->channels.push_back(
 {joint.channel, joint.position_rad, joint.velocity_radps});
 }
 } else {
 for (const auto& joint : feedback.joints)
 frame.actuator_feedback->channels.push_back(
 {joint.channel, joint.position_rad, joint.velocity_radps});
 }
 frame.actuator_feedback->header = {
 sample_time_s, sequence, "base_link", 0.0,
 kMeasurementSchemaVersion, "sim.actuator-feedback"};
 return frame;
}

bool SimulatedPlatformProviderAdapter::publish_actuator_feedback(
 runtime::IRuntimeChannel& channel, const ChassisActuatorFeedback& feedback,
 std::string_view drive_type, double sample_time_s, std::uint64_t sequence) {
 auto frame = make_actuator_feedback(feedback, drive_type, sample_time_s, sequence);
 return channel.platform().publish_observation(std::move(frame));
}

ChassisActuatorDutyCommand SimulatedPlatformProviderAdapter::consume_actuator_command(
 const runtime::ActuatorCommand& command) {
 ChassisActuatorDutyCommand output;
 output.setpoints.reserve(command.setpoints.size());
 for (const auto& setpoint : command.setpoints) {
 if (setpoint.mode != runtime::ActuatorCommandMode::NormalizedDuty &&
 setpoint.mode != runtime::ActuatorCommandMode::NormalizedBrake)
 throw std::runtime_error("SIM actuator provider received unsupported command mode");
 output.setpoints.push_back({setpoint.channel_id, setpoint.value});
 }
 return output;
}

} // namespace sim::platform
