#include "robot/control/holonomic_actuator_controller.hpp"
#include "robot/core/math.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#define CHECK(c)                                                                                             \
 do {                                                                                                        \
  if (!(c)) {                                                                                                \
   std::cerr << __FILE__ << ':' << __LINE__ << ": " #c "\n";                                                 \
   return 1;                                                                                                 \
  }                                                                                                          \
 } while (false)

int main() {
 const sim::WheelPidGains gains{0.5, 0.0, 0.0};
 sim::HolonomicActuatorFeedback feedback;
 sim::HolonomicActuatorController mecanum(
   {sim::HolonomicControlKind::Mecanum, .6, .5, .1, 2.0, 1.0, 1.0, 3.0}, gains);
 auto lateral = mecanum.update({0.0, 0.0, 1.0}, feedback, .02);
 CHECK(lateral.target_speed_mps[0] < 0.0);
 CHECK(lateral.target_speed_mps[1] > 0.0);
 CHECK(lateral.target_speed_mps[2] > 0.0);
 CHECK(lateral.target_speed_mps[3] < 0.0);
 auto yaw = mecanum.update({0.0, 1.0, 0.0}, feedback, .02);
 CHECK(yaw.target_speed_mps[0] < 0.0 && yaw.target_speed_mps[1] > 0.0);
 CHECK(yaw.target_speed_mps[2] < 0.0 && yaw.target_speed_mps[3] > 0.0);

 sim::HolonomicActuatorController swerve({sim::HolonomicControlKind::Swerve, .6, .5, .1, 2.0, 1.0, 1.0, 3.0},
                                         gains);
 feedback.steer_position_rad.fill(3.10);
 const auto reverse = swerve.update({1.0, 0.0, 0.0}, feedback, .02);
 // A module already near pi reverses its wheel and asks only for the short
 // steering correction instead of rotating the mechanism through pi.
 for (std::size_t i = 0; i < 4; ++i) {
  CHECK(reverse.target_speed_mps[i] < 0.0);
  CHECK(std::abs(reverse.target_steer_rad[i] - sim::kPi) < 0.1);
  CHECK(std::abs(reverse.steer_duty[i]) < 0.2);
  CHECK(std::abs(reverse.drive_duty[i]) <= 1.0);
 }

 // During a turn-to-straight transition, drive effort must wait for each
 // module to align. Otherwise the four wheels pull in different lateral
 // directions and excite a visible rear-body oscillation.
 feedback = {};
 feedback.steer_position_rad.fill(1.4);
 const auto misaligned = swerve.update({1.0, 0.0, 0.0}, feedback, .02);
 for (std::size_t i = 0; i < 4; ++i) {
 CHECK(misaligned.target_speed_mps[i] > 0.0);
 CHECK(misaligned.target_speed_mps[i] < 0.2);
 }
 feedback.steer_velocity_radps.fill(-1.5);
 const auto aligning = swerve.update({1.0, 0.0, 0.0}, feedback, .02);
 for (std::size_t i = 0; i < 4; ++i)
 CHECK(std::abs(aligning.steer_duty[i]) < std::abs(misaligned.steer_duty[i]));

 sim::HolonomicActuatorController reversing(
 {sim::HolonomicControlKind::Mecanum, .6, .5, .1, 2.0, 1.0, 2.0, 3.0},
 {0.2, 1.0, 0.0});
 sim::HolonomicActuatorControlOutput forward_windup;
 for (int step = 0; step < 100; ++step)
 forward_windup = reversing.update({1.0, 0.0, 0.0}, {}, 0.02);
 CHECK(forward_windup.drive_duty[0] > 0.5);
 const auto reverse_after_windup = reversing.update({-1.0, 0.0, 0.0}, {}, 0.02);
 for (double duty : reverse_after_windup.drive_duty)
 CHECK(duty < 0.0);

 std::array<double, 4> wheel{1.0, 1.0, 1.0, 1.0};
 std::array<double, 4> steer{};
 double forward = 0.0, left = 0.0, turn = 0.0;
 sim::swerve_forward(wheel, steer, .6, .5, forward, left, turn);
 CHECK(std::abs(forward - 1.0) < 1e-12);
 CHECK(std::abs(left) < 1e-12);
 CHECK(std::abs(turn) < 1e-12);

 bool rejected_non_finite = false;
 try {
 feedback.drive_velocity_radps[0] = std::numeric_limits<double>::quiet_NaN();
 (void)swerve.update({1.0, 0.0, 0.0}, feedback, .02);
 } catch (const std::invalid_argument&) {
 rejected_non_finite = true;
 }
 CHECK(rejected_non_finite);
 std::cout << "holonomic actuator controller green\n";
 return 0;
}
