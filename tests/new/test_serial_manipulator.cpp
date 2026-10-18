#include "robot/control/manipulator_joint_controller.hpp"
#include "robot/platform/serial_manipulator.hpp"

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
 const sim::ManipulatorGeometry geometry{
 0.42, 0.30, 0.26, 0.12,
 {-2.8, -1.45, -2.35, -1.75},
 {2.8, 1.45, 2.35, 1.75},
 };
 CHECK(sim::valid_manipulator_geometry(geometry));

 const sim::ManipulatorJointPosition expected{{0.48, 0.42, -0.72, 0.25}};
 const auto target = sim::manipulator_forward_kinematics(geometry, expected);
 const auto solved = sim::manipulator_inverse_kinematics(geometry, target, expected);
 CHECK(solved.has_value());
 CHECK(solved->residual_m < 1e-9);
 const auto reached = sim::manipulator_forward_kinematics(geometry, solved->joints);
 CHECK(std::hypot(std::hypot(reached.x_m - target.x_m, reached.y_m - target.y_m),
 reached.z_m - target.z_m) < 1e-9);
 CHECK(std::abs(reached.tool_pitch_rad - target.tool_pitch_rad) < 1e-9);

 CHECK(!sim::manipulator_inverse_kinematics(
 geometry, {1.5, 0.0, 1.5, 0.0}, expected).has_value());

 sim::ManipulatorJointController controller({
 {30.0, 35.0, 28.0, 18.0},
 {2.5, 3.0, 2.2, 1.5},
 {4.0, 8.0, 6.0, 3.0},
 {1.0, 0.8, 1.1, 1.3},
 });
 sim::ManipulatorJointFeedback feedback{{0.0, 0.0, 0.0, 0.0},
 {0.0, 0.0, 0.0, 0.0}, true};
 const auto first = controller.update({1.0, -1.0, 1.0, -1.0}, feedback, 0.02);
 CHECK(std::abs(first.target_rad[0] - 0.02) < 1e-12);
 CHECK(std::abs(first.target_rad[1] + 0.016) < 1e-12);
 CHECK(std::abs(first.effort_Nm[0]) <= 4.0);
 CHECK(std::abs(first.effort_Nm[1]) <= 8.0);

 return 0;
}
