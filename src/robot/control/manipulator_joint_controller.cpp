#include "robot/control/manipulator_joint_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sim {

ManipulatorJointController::ManipulatorJointController(
 ManipulatorJointControllerConfig config) : config_(config) {
 for (std::size_t i = 0; i < config_.kp.size(); ++i) {
 if (!(std::isfinite(config_.kp[i]) && config_.kp[i] > 0.0 &&
 std::isfinite(config_.kd[i]) && config_.kd[i] >= 0.0 &&
 std::isfinite(config_.maximum_effort_Nm[i]) && config_.maximum_effort_Nm[i] > 0.0 &&
 std::isfinite(config_.maximum_velocity_radps[i]) && config_.maximum_velocity_radps[i] > 0.0))
 throw std::invalid_argument("invalid manipulator joint controller configuration");
 }
 for (const double moment : config_.gravity_moment_Nm) {
 if (!(std::isfinite(moment) && moment >= 0.0))
 throw std::invalid_argument("invalid manipulator gravity model");
 }
}

void ManipulatorJointController::reset(
 const std::array<double, 4>& measured_position_rad) {
 reference_rad_ = measured_position_rad;
 initialized_ = true;
}

ManipulatorJointControl ManipulatorJointController::update(
 const std::array<double, 4>& requested_position_rad,
 const ManipulatorJointFeedback& feedback, double dt_s) {
 if (!feedback.valid || !(std::isfinite(dt_s) && dt_s > 0.0))
 throw std::invalid_argument("manipulator controller requires valid feedback and dt");
 if (!initialized_)
 reset(feedback.position_rad);
 ManipulatorJointControl output;
 const double shoulder = feedback.position_rad[1];
 const double elbow_sum = shoulder + feedback.position_rad[2];
 const double tool_pitch = elbow_sum + feedback.position_rad[3];
 const std::array<double, 4> gravity_effort_Nm{
 0.0,
 -(config_.gravity_moment_Nm[0] * std::sin(shoulder) +
 config_.gravity_moment_Nm[1] * std::sin(elbow_sum) +
 config_.gravity_moment_Nm[2] * std::sin(tool_pitch)),
 -(config_.gravity_moment_Nm[1] * std::sin(elbow_sum) +
 config_.gravity_moment_Nm[2] * std::sin(tool_pitch)),
 -config_.gravity_moment_Nm[2] * std::sin(tool_pitch),
 };
 for (std::size_t i = 0; i < reference_rad_.size(); ++i) {
 const double maximum_step = config_.maximum_velocity_radps[i] * dt_s;
 reference_rad_[i] += std::clamp(requested_position_rad[i] - reference_rad_[i],
 -maximum_step, maximum_step);
 output.target_rad[i] = reference_rad_[i];
 const double raw = config_.kp[i] * (reference_rad_[i] - feedback.position_rad[i]) -
 config_.kd[i] * feedback.velocity_radps[i] + gravity_effort_Nm[i];
 output.effort_Nm[i] = std::clamp(raw, -config_.maximum_effort_Nm[i],
 config_.maximum_effort_Nm[i]);
 output.saturated[i] = std::abs(raw) > config_.maximum_effort_Nm[i] + 1e-12;
 }
 return output;
}

} // namespace sim
