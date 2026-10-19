#pragma once
namespace sim {
// Transport-neutral planar base state shared by autonomy, runtime providers,
// and vehicle implementations. It contains no simulation state or physics API.
struct DiffState {
 double x{0}, y{0}, theta{0}, v{0}, w{0};
 // Body-left velocity. An offset differential axle produces the rigid-body
 // value -axle_x*w at base_link; swerve and mecanum command it directly.
 double lateral_v{0};
};
struct DiffCmd {
 // Internal compatibility carrier. target_w is yaw rate for YawRate and
 // HolonomicTwist chassis, and equivalent front-wheel steering angle for a
 // SteeringAngle chassis. LSRI keeps these meanings in separate named fields.
 double target_v{0}, target_w{0};
 double target_lateral_v{0};
 // A controlled braking transition invalidates integral state accumulated
 // for the previous maneuver. The target may remain non-zero while a
 // stability-aware ramp approaches rest, so this is distinct from LSRI's
 // zero-valued fail-safe command.
 bool controlled_stop{false};
};
// Pure differential-drive kinematics. Real and simulated platform providers
// use the same conversion, while acceleration, contact, motor, and pose
// integration belong to their respective platform implementations.
class DifferentialKinematics {
 public:
 static void twist_to_wheels(double forward_mps, double yaw_rate_radps,
 double track_width_m, double& left_mps, double& right_mps);
 static void wheels_to_twist(double left_mps, double right_mps,
 double track_width_m, double& forward_mps, double& yaw_rate_radps);
};
} // namespace sim
