#include "robot/runtime/manipulator_runtime.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace sim::robot {

ManipulatorRuntime::ManipulatorRuntime(
 runtime::IRuntimeChannel& channel, ManipulatorRuntimeConfig config)
 : channel_(channel), config_(std::move(config)), controller_(config_.controller) {
 if (config_.channels.size() != 4)
 throw std::invalid_argument("manipulator runtime requires four actuator channels");
 if (!valid_manipulator_geometry(config_.geometry))
 throw std::invalid_argument("manipulator runtime received invalid geometry");
 target_joints_ = config_.initial_joints;
}

void ManipulatorRuntime::reset() {
 initialized_ = false;
}

ManipulatorCycleOutput ManipulatorRuntime::tick(const ManipulatorCycleInput& input) {
 if (!(input.dt_s > 0.0) || !std::isfinite(input.dt_s))
 throw std::invalid_argument("manipulator runtime requires a finite positive timestep");
 const auto feedback_frame = channel_.autonomy().read_observation();
 if (!feedback_frame || !feedback_frame->actuator_feedback)
 throw std::runtime_error("manipulator runtime requires actuator feedback");

 ManipulatorJointFeedback feedback;
 for (std::size_t index = 0; index < config_.channels.size(); ++index) {
 const auto* joint = feedback_frame->actuator_feedback->find(config_.channels[index]);
 if (!joint)
 throw std::runtime_error("manipulator feedback is missing channel '" +
 config_.channels[index] + "'");
 feedback.position_rad[index] = joint->position_rad;
 feedback.velocity_radps[index] = joint->velocity_radps;
 }
 feedback.valid = true;
 if (!initialized_) {
 controller_.reset(feedback.position_rad);
 initialized_ = true;
 }

 ManipulatorCycleOutput output;
 output.target_pose = input.target_pose;
 const auto solution = manipulator_inverse_kinematics(
 config_.geometry, input.target_pose, target_joints_);
 if (solution) {
 target_joints_ = solution->joints;
 output.target_reachable = true;
 }

 const auto now = input.now;
 output.command.header = {
 runtime::kMessageSchemaVersion, now, now, input.sequence,
 config_.frame_id, config_.source_id};
 output.command.valid_until = input.valid_until;
 output.command.setpoints.reserve(config_.channels.size());
 for (std::size_t index = 0; index < config_.channels.size(); ++index) {
 output.command.setpoints.push_back({
 config_.channels[index], runtime::ActuatorCommandMode::PositionRad,
 target_joints_.rad[index]});
 }
 if (channel_.autonomy().write_actuators(output.command) !=
 runtime::CommandWriteResult::Accepted)
 throw std::runtime_error("manipulator runtime actuator command was rejected");

 // The provider may clamp or expire a setpoint. Compute effort from the
 // delivered LSRI command, never from a simulator-side copy of the target.
 const auto delivered = channel_.platform().read_actuators(input.now);
 std::array<double, 4> delivered_target{};
 for (std::size_t index = 0; index < config_.channels.size(); ++index) {
 const auto setpoint = std::find_if(
 delivered.setpoints.begin(), delivered.setpoints.end(), [&](const auto& value) {
 return value.channel_id == config_.channels[index];
 });
 if (setpoint == delivered.setpoints.end() ||
 setpoint->mode != runtime::ActuatorCommandMode::PositionRad)
 throw std::runtime_error("delivered manipulator command is incomplete");
 delivered_target[index] = setpoint->value;
 }
 output.control = controller_.update(delivered_target, feedback, input.dt_s);
 return output;
}

} // namespace sim::robot
