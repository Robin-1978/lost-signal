#include "robot/platform/serial_manipulator.hpp"

#include "robot/core/math.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace sim {
namespace {

double angular_distance(double a, double b) {
 return std::abs(wrap_angle(a - b));
}

bool within_limits(const ManipulatorGeometry& geometry,
 const ManipulatorJointPosition& joints) {
 for (std::size_t i = 0; i < joints.rad.size(); ++i) {
 if (joints.rad[i] < geometry.lower_rad[i] - 1e-9 ||
 joints.rad[i] > geometry.upper_rad[i] + 1e-9)
 return false;
 }
 return true;
}

} // namespace

bool valid_manipulator_geometry(const ManipulatorGeometry& geometry) {
 if (!(std::isfinite(geometry.shoulder_height_m) && geometry.shoulder_height_m >= 0.0 &&
 std::isfinite(geometry.upper_arm_length_m) && geometry.upper_arm_length_m > 0.0 &&
 std::isfinite(geometry.forearm_length_m) && geometry.forearm_length_m > 0.0 &&
 std::isfinite(geometry.tool_length_m) && geometry.tool_length_m >= 0.0))
 return false;
 for (std::size_t i = 0; i < geometry.lower_rad.size(); ++i) {
 if (!std::isfinite(geometry.lower_rad[i]) || !std::isfinite(geometry.upper_rad[i]) ||
 geometry.lower_rad[i] >= geometry.upper_rad[i])
 return false;
 }
 return true;
}

EndEffectorPose manipulator_forward_kinematics(
 const ManipulatorGeometry& geometry, const ManipulatorJointPosition& joints) {
 const double yaw = joints.rad[0];
 const double shoulder = joints.rad[1];
 const double elbow_sum = shoulder + joints.rad[2];
 const double pitch = elbow_sum + joints.rad[3];
 const double radial = geometry.upper_arm_length_m * std::sin(shoulder) +
 geometry.forearm_length_m * std::sin(elbow_sum) +
 geometry.tool_length_m * std::sin(pitch);
 const double z = geometry.shoulder_height_m +
 geometry.upper_arm_length_m * std::cos(shoulder) +
 geometry.forearm_length_m * std::cos(elbow_sum) +
 geometry.tool_length_m * std::cos(pitch);
 return {radial * std::cos(yaw), radial * std::sin(yaw), z, pitch};
}

std::optional<ManipulatorIkSolution> manipulator_inverse_kinematics(
 const ManipulatorGeometry& geometry, const EndEffectorPose& target,
 const ManipulatorJointPosition& seed) {
 if (!valid_manipulator_geometry(geometry) || !std::isfinite(target.x_m) ||
 !std::isfinite(target.y_m) || !std::isfinite(target.z_m) ||
 !std::isfinite(target.tool_pitch_rad))
 return std::nullopt;

 const double radial = std::hypot(target.x_m, target.y_m);
 double yaw = radial > 1e-9 ? std::atan2(target.y_m, target.x_m) : seed.rad[0];
 yaw = wrap_angle(yaw);
 const double wrist_radial = radial - geometry.tool_length_m * std::sin(target.tool_pitch_rad);
 const double wrist_z = target.z_m - geometry.shoulder_height_m -
 geometry.tool_length_m * std::cos(target.tool_pitch_rad);
 const double l1 = geometry.upper_arm_length_m;
 const double l2 = geometry.forearm_length_m;
 const double cosine_elbow = (wrist_radial * wrist_radial + wrist_z * wrist_z -
 l1 * l1 - l2 * l2) / (2.0 * l1 * l2);
 if (cosine_elbow < -1.0 - 1e-9 || cosine_elbow > 1.0 + 1e-9)
 return std::nullopt;

 const double bounded_cosine = std::clamp(cosine_elbow, -1.0, 1.0);
 std::optional<ManipulatorIkSolution> best;
 double best_seed_distance = std::numeric_limits<double>::infinity();
 for (const double sign : {1.0, -1.0}) {
 const double elbow = sign * std::acos(bounded_cosine);
 const double shoulder = std::atan2(wrist_radial, wrist_z) -
 std::atan2(l2 * std::sin(elbow), l1 + l2 * std::cos(elbow));
 const double wrist = target.tool_pitch_rad - shoulder - elbow;
 ManipulatorJointPosition candidate{{yaw, shoulder, elbow, wrist}};
 if (!within_limits(geometry, candidate))
 continue;
 const auto reached = manipulator_forward_kinematics(geometry, candidate);
 const double residual = std::hypot(std::hypot(reached.x_m - target.x_m,
 reached.y_m - target.y_m), reached.z_m - target.z_m);
 const double seed_distance = angular_distance(yaw, seed.rad[0]) +
 angular_distance(shoulder, seed.rad[1]) + angular_distance(elbow, seed.rad[2]) +
 angular_distance(wrist, seed.rad[3]);
 if (seed_distance < best_seed_distance) {
 best_seed_distance = seed_distance;
 best = ManipulatorIkSolution{candidate, residual};
 }
 }
 return best;
}

} // namespace sim
