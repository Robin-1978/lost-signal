#pragma once

#include "diff_drive.hpp"
#include "pid.hpp"

namespace sim {

struct MotorDriveParams {
 double wheel_base_m{0.5};
 double wheel_radius_m{0.1};
 double max_wheel_speed_mps{2.4};
 double max_wheel_accel_mps2{4.0};
 int encoder_counts_per_revolution{2048};
 double speed_pid_kp{1.8};
 double speed_pid_ki{0.35};
 double speed_pid_kd{0.01};
};

struct WheelMotorTelemetry {
 double target_mps{0.0};
 double measured_mps{0.0};
 double actual_mps{0.0};
 double pwm{0.0};
 double pid_p{0.0};
 double pid_i{0.0};
 double pid_d{0.0};
};

struct MotorDriveTelemetry {
 WheelMotorTelemetry left;
 WheelMotorTelemetry right;
};

// A compact differential-drive actuator model. Navigation supplies a body
// twist, encoders quantize wheel feedback, two speed PIDs produce normalized
// PWM duty cycles, and PWM accelerates each wheel before kinematics are applied.
class DifferentialMotorDrive {
public:
 explicit DifferentialMotorDrive(MotorDriveParams params);

 void reset();
 DiffState step(const DiffState& state, const DiffCmd& command, double dt);
 const MotorDriveParams& params() const { return params_; }
 const MotorDriveTelemetry& telemetry() const { return telemetry_; }

private:
 double encoder_measure(double wheel_mps, double dt) const;

 MotorDriveParams params_;
 PID left_pid_;
 PID right_pid_;
 MotorDriveTelemetry telemetry_;
};

} // namespace sim
