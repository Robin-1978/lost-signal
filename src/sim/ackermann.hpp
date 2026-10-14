#pragma once

#include "diff_drive.hpp"
#include "motor_drive.hpp"
#include "pid.hpp"

namespace sim {

// Ackermann actuator parameters. Everything comes from robot JSON once
// ackermann content exists; there are no per-vehicle hand-tuned gains because
// both loops are derived (drive like a wheel, servo by response spec).
struct AckermannParams {
 double wheel_base_m{2.0};
 double wheel_radius_m{0.1};
 double max_steer_rad{0.6};
 double max_steer_rate_radps{2.5};
 double servo_response_s{0.4};
 double max_drive_speed_mps{2.4};
 double max_drive_accel_mps2{4.0};
 double stall_force_N{54.0};
 double free_speed_mps{3.0};
 double mass_kg{24.0};
 double pwm_limit{1.0};
 double integral_limit{2.0};
 int encoder_counts_per_revolution{2048};
};

// Ackermann chassis: one drive motor (speed PID -> PWM -> DC force, same
// physics as a differential wheel) plus one steering servo (position P loop ->
// rate-limited first-order angle). Outputs differ from differential on purpose:
// drive PWM + steer PWM/angle instead of two wheel PWMs. Bicycle kinematics
// (w = v * tan(delta) / L) means no spinning at standstill.
class AckermannDrive : public ChassisModel {
 public:
 AckermannDrive(AckermannParams params, WheelPidGains drive_gains, double servo_kp);

 void reset();
 DiffState step(const DiffState& state, const DiffCmd& command, double dt,
                const GroundContact& ground) override;
 const ChassisTelemetry& telemetry() const override { return telemetry_; }
 double steer_angle() const { return delta_rad_; }

 private:
 double encoder_measure(double wheel_mps, double dt) const;

 AckermannParams params_;
 PID drive_pid_;
 PID servo_pid_;
 double delta_rad_{0.0};
 double servo_rate_state_{0.0};
 ChassisTelemetry telemetry_;
};

} // namespace sim
