#pragma once

#include <array>
#include <optional>

namespace sim {

// Compact yaw + planar 3R arm used by the first mobile manipulator. Angles
// are SI radians. Shoulder, elbow and wrist pitch are measured from +Z toward
// the arm's radial +X direction; base yaw is measured around +Z.
struct ManipulatorGeometry {
 double shoulder_height_m{0.0};
 double upper_arm_length_m{0.0};
 double forearm_length_m{0.0};
 double tool_length_m{0.0};
 std::array<double, 4> lower_rad{};
 std::array<double, 4> upper_rad{};
};

struct ManipulatorJointPosition {
 std::array<double, 4> rad{}; // base yaw, shoulder, elbow, wrist
};

struct EndEffectorPose {
 double x_m{0.0};
 double y_m{0.0};
 double z_m{0.0};
 double tool_pitch_rad{0.0};
};

struct ManipulatorIkSolution {
 ManipulatorJointPosition joints;
 double residual_m{0.0};
};

EndEffectorPose manipulator_forward_kinematics(
 const ManipulatorGeometry& geometry, const ManipulatorJointPosition& joints);

// Returns the valid elbow branch nearest seed. An unreachable target or a
// solution outside any authored joint limit is rejected rather than clamped.
std::optional<ManipulatorIkSolution> manipulator_inverse_kinematics(
 const ManipulatorGeometry& geometry, const EndEffectorPose& target,
 const ManipulatorJointPosition& seed);

bool valid_manipulator_geometry(const ManipulatorGeometry& geometry);

} // namespace sim
