#include "robot/control/ackermann_actuator_controller.hpp"
#include "robot/runtime/runtime_channel.hpp"

#include <cmath>
#include <iostream>

#define CHECK(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

int main() {
 constexpr double base_m = 0.62;
 constexpr double track_m = 0.58;
 constexpr double steer_rad = 0.35;
 const auto steering = sim::ackermann_steering_angles(
 steer_rad, base_m, track_m);
 CHECK(steering.left_rad > steering.right_rad);
 CHECK(steering.right_rad > 0.0);
 const double left_icr_m = base_m / std::tan(steering.left_rad) + track_m * 0.5;
 const double right_icr_m = base_m / std::tan(steering.right_rad) - track_m * 0.5;
 CHECK(std::abs(left_icr_m - right_icr_m) < 1e-12);

 const auto rear = sim::ackermann_rear_wheel_speeds(
 1.0, steer_rad, base_m, track_m);
 CHECK(rear.left_mps < rear.right_mps);
 CHECK(std::abs((rear.left_mps + rear.right_mps) * 0.5 - 1.0) < 1e-12);
 const auto reverse = sim::ackermann_rear_wheel_speeds(
 -1.0, steer_rad, base_m, track_m);
 CHECK(reverse.left_mps < 0.0 && reverse.right_mps < 0.0);

 sim::AckermannActuatorController controller({
 base_m, track_m, 0.1, 2.4, 0.55, 2.0, 2048,
 1.0, 2.0, 1.0, 1.2, 1.0 / 0.35, 0.35,
 }, {0.4, 0.08, 0.002});
 const auto first = controller.update({1.0, steer_rad}, {}, 0.02);
 CHECK(first.rear_left.target_mps < first.rear_right.target_mps);
 CHECK(first.rear_left.normalized_duty > 0.0);
 CHECK(first.rear_right.normalized_duty > 0.0);
 CHECK(first.front_left_steer.target_rad > first.front_right_steer.target_rad);
 CHECK(first.front_left_steer.normalized_duty >
 first.front_right_steer.normalized_duty);

 // LSRI actuator expiry is independent of the in-process simulator. A real
 // provider receives exactly the same four safe zero setpoints after TTL.
 sim::runtime::RuntimeChannelConfig config;
 config.capabilities.command_level = sim::runtime::PlatformCommandLevel::Actuator;
 config.capabilities.max_forward_speed_mps = 2.4;
 config.capabilities.max_lateral_speed_mps = 0.0;
 config.capabilities.chassis.command_space =
 sim::ChassisCommandSpace::SteeringAngle;
 config.capabilities.chassis.geometry.steering_wheel_base_m = base_m;
 config.capabilities.chassis.geometry.max_steer_rad = 0.55;
 config.capabilities.actuator_channels = {
 {"rear_left_drive", sim::runtime::ActuatorCommandMode::NormalizedDuty, -1.0, 1.0, 0.0},
 {"rear_right_drive", sim::runtime::ActuatorCommandMode::NormalizedDuty, -1.0, 1.0, 0.0},
 {"front_left_steer", sim::runtime::ActuatorCommandMode::NormalizedDuty, -1.0, 1.0, 0.0},
 {"front_right_steer", sim::runtime::ActuatorCommandMode::NormalizedDuty, -1.0, 1.0, 0.0},
 };
 auto channel = sim::runtime::make_in_process_channel(config);
 channel->configure();
 channel->activate();
 sim::runtime::ActuatorCommand command;
 command.header.sample_time = sim::runtime::TimePoint::from_seconds(1.0);
 command.header.receive_time = command.header.sample_time;
 command.header.sequence = 1;
 command.header.frame_id = "base_link";
 command.header.source_id = "controller.ro2";
 command.valid_until = sim::runtime::TimePoint::from_seconds(1.05);
 command.setpoints = {
 {"rear_left_drive", sim::runtime::ActuatorCommandMode::NormalizedDuty, 0.4},
 {"rear_right_drive", sim::runtime::ActuatorCommandMode::NormalizedDuty, 0.4},
 {"front_left_steer", sim::runtime::ActuatorCommandMode::NormalizedDuty, 0.2},
 {"front_right_steer", sim::runtime::ActuatorCommandMode::NormalizedDuty, 0.2},
 };
 CHECK(channel->autonomy().write_actuators(command) ==
 sim::runtime::CommandWriteResult::Accepted);
 const auto expired = channel->platform().read_actuators(
 sim::runtime::TimePoint::from_seconds(1.10));
 CHECK(expired.safe_stop);
 CHECK(expired.setpoints.size() == 4);
 for (const auto& setpoint : expired.setpoints)
 CHECK(setpoint.value == 0.0);
 CHECK(channel->health().watchdog_stops == 1);
 return 0;
}
