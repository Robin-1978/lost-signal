#pragma once

#include "robot/control/differential_wheel_controller.hpp"
#include "robot/platform/holonomic_kinematics.hpp"
#include "robot/platform/differential_kinematics.hpp"

#include <array>

namespace sim {
enum class HolonomicControlKind { Swerve, Mecanum };

struct HolonomicActuatorControllerConfig {
 HolonomicControlKind kind{HolonomicControlKind::Swerve};
 double wheel_base_m{0.56};
 double track_width_m{0.56};
 double wheel_radius_m{0.1};
 double max_wheel_speed_mps{2.4};
 double duty_limit{1.0};
 double integral_limit{2.0};
 double steer_kp{2.5};
 double steer_velocity_gain_s{0.35};
 double max_steer_rate_radps{2.0};
};
struct HolonomicActuatorFeedback {
 std::array<double, 4> drive_velocity_radps{};
 std::array<double, 4> steer_position_rad{};
 std::array<double, 4> steer_velocity_radps{};
};
struct HolonomicActuatorControlOutput {
 std::array<double, 4> drive_duty{};
 std::array<double, 4> steer_duty{};
 std::array<double, 4> target_speed_mps{};
 std::array<double, 4> target_steer_rad{};
};

class HolonomicActuatorController {
 public:
 HolonomicActuatorController(HolonomicActuatorControllerConfig config, WheelPidGains drive_gains);
 void reset();
 HolonomicActuatorControlOutput update(const DiffCmd& command, const HolonomicActuatorFeedback& feedback,
                                       double dt_s);

 private:
 HolonomicActuatorControllerConfig config_;
 std::array<PID, 4> drive_pid_;
};
} // namespace sim
