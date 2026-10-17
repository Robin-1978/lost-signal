#pragma once
namespace sim {
// Differential chassis: twin-wheel drive, reverse and in-place rotation, with no minimum turn radius.
// Excess yaw rate can cause slip and amplify odometry error; motor_drive applies the traction limit.
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
};
struct DiffParams {
 double v_max{2.0}, v_min{-1.0}, w_max{2.0};
 double accel_max{3.0}, alpha_max{8.0};
 double wheel_base{0.5};
};
class DiffDrive {
 public:
 explicit DiffDrive(DiffParams p = {}) : p_(p) {}
 DiffState step(const DiffState& s, const DiffCmd& cmd, double dt);
 const DiffParams& params() const { return p_; }
 // Stable twist-to-wheel boundary for motor and tire upgrades.
 static void twistToWheels(double v, double w, double base, double& vl, double& vr);
 static void wheelsToTwist(double vl, double vr, double base, double& v, double& w);

 private:
 DiffParams p_;
};
} // namespace sim
