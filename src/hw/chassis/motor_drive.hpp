#pragma once

#include "caps.hpp"
#include "hw/dynamics/contact.hpp"

#include "diff_drive.hpp"
#include "autonomy/control/pid.hpp"

#include <array>
#include <cstddef>
#include <vector>

namespace sim {

// Motor, contact and inertia parameters. All values come from robot/tire JSON;
// PID gains are never configured, they are derived from these physics by
// tune_wheel_pid() so DIY assemblies stay drivable without hand tuning.
struct MotorDriveParams {
 double wheel_base_m{0.5};
 double wheel_radius_m{0.1};
 double max_wheel_speed_mps{2.4};
 double max_wheel_accel_mps2{6.0};
 double stall_force_N{54.0};
 double free_speed_mps{3.0};
 double mass_kg{24.0};
 double yaw_inertia_kgm2{2.4};
 int encoder_counts_per_revolution{2048};
 double pwm_limit{1.0};
 double integral_limit{2.0};
 double support_width_m{0.5};
 double support_length_m{0.5};
 double max_yaw_rate_radps{2.0};
 struct PassiveCaster {
 double x_m{0.0};
 double y_m{0.0};
 };
 std::vector<PassiveCaster> passive_casters;
};

// Per-step ground contact for the driven wheels. The mission combines the tire
// tread mu with the current surface material mu before stepping. The tire also
// caps usable acceleration (grip-limited traction), keeping tire choice
// physically meaningful beyond braking distance.
struct GroundContact {
 double mu_eff{0.7};
 double rolling_resist{0.01};
 double accel_limit_mps2{6.0};
};

struct WheelMotorTelemetry {
 double target_mps{0.0};
 double measured_mps{0.0};
 double actual_mps{0.0};
 double pwm{0.0};
 double pid_p{0.0};
 double pid_i{0.0};
 double pid_d{0.0};
 bool slipping{false};
 // Signed force delivered by this wheel/track to the chassis after motor
 // curve, rolling resistance and the tire-ground traction clamp.
 double longitudinal_force_N{0.0};
};

struct MotorDriveTelemetry {
 WheelMotorTelemetry left;
 WheelMotorTelemetry right;
};

// Steering servo telemetry (ackermann). A servo is position-controlled: the
// loop output is PWM, the state is the angle, and the rate limit is physics.
struct ServoTelemetry {
 double target_rad{0.0};
 double angle_rad{0.0};
 double pwm{0.0};
 double pid_p{0.0};
 double pid_i{0.0};
 double pid_d{0.0};
};

// Unified chassis telemetry. Each chassis fills only its real actuators:
// twin-drive uses left/right, Ackermann uses drive+servo, and holonomic
// chassis use four modules. Inactive channels stay zero.
struct ChassisTelemetry {
 bool differential{true};
 bool holonomic{false};
 WheelMotorTelemetry left;
 WheelMotorTelemetry right;
 WheelMotorTelemetry drive;
 ServoTelemetry steer;
 bool steering_servo_present{false};
 // Steering actuator output in the robot tire-slot mount order. The chassis
 // owns these angles; assembly code binds index -> mount id and renderers only
 // consume the resulting mount transforms. Ackermann fills FL/FR, swerve
 // fills FL/FR/RL/RR, and fixed-wheel chassis expose zero entries.
 std::array<double, 4> wheel_steer_rad{};
 std::size_t wheel_steer_count{0};
 // Passive swivel angles are a chassis output just like powered steering.
 // World, TF and render code only bind channel order to authored mounts.
 std::array<double, 4> passive_caster_steer_rad{};
 std::size_t passive_caster_steer_count{0};
 // Extra moment not already produced by applying wheel forces at their
 // physical contact points. Tracked chassis use it for scrub resistance;
 // Ackermann uses it for lateral tire response. Differential leaves it zero.
 double extra_yaw_torque_Nm{0.0};
 double desired_yaw_rate_radps{0.0};
 double yaw_rate_tracking_time_s{0.0};
 // Four-module drive output for swerve/mecanum: FL, FR, RL, RR.
 std::array<WheelMotorTelemetry, 4> modules;
 double body_forward_force_N{0.0};
 double body_lateral_force_N{0.0};
 double measured_forward_mps{0.0};
 double measured_lateral_mps{0.0};
 double measured_yaw_radps{0.0};
 bool traction_saturated{false};
};

// Closed-loop wheel-speed tuning from first principles. The linearized wheel
// plant dV/dt = (K_lin*u - c*V)/m_w is dominated by inertia, so a conservative
// PI is placed by bandwidth: Kp = m_w*wc/K_lin, Ki = Kp*wc/30 (slow integrator
// Ti = 30/wc kills steady-state error without windup fights), Kd = Kp*0.005
// (5 ms derivative time stays clear of encoder quantization noise).
// wc = 6 rad/s targets a ~0.5 s settle, well below the 50 Hz actuator rate.
struct WheelPidGains {
 double kp{0.0};
 double ki{0.0};
 double kd{0.0};
};

WheelPidGains tune_wheel_pid(double wheel_mass_kg, double stall_force_N, double free_speed_mps,
                             double linearize_at_mps);

// A chassis turns a body twist command into motion. Differential drive and
// ackermann are the implementations; ro1.json drive.type selects one. Unknown
// types must be rejected by the factory in game code, never stubbed here.
class ChassisModel {
 public:
 virtual ~ChassisModel() = default;
 virtual DiffState step(const DiffState& state, const DiffCmd& command, double dt,
                        const GroundContact& ground) = 0;
 virtual const ChassisTelemetry& telemetry() const = 0;
 virtual ChassisCaps caps() const = 0;
};

// Differential drive with per-wheel DC force, Coulomb/viscous resistance,
// slip-capped traction and body inertia:
//
//   F = PWM * F_stall * max(0, 1 - |v_w| / v_free)   (high rpm -> low torque)
//   F_res = (rr_tire + rr_ground) * m * g / 2 * tanh(v_w / 0.05) + c_visc * v_w
//   |F| <= mu_eff * m * g / 2, excess flags slipping (odometry + braking pay)
//   F_sum -> a = F_sum / m, torque = (F_r - F_l) * base / 2 -> alpha = torque / I
//
// Navigation supplies a body twist, encoders quantize wheel feedback, the two
// auto-tuned speed PIDs produce PWM duty cycles, forces accelerate the body.
class DifferentialMotorDrive : public ChassisModel {
 public:
 DifferentialMotorDrive(MotorDriveParams params, WheelPidGains gains);

 void reset();
 DiffState step(const DiffState& state, const DiffCmd& command, double dt,
                const GroundContact& ground) override;
 const MotorDriveParams& params() const { return params_; }
 const ChassisTelemetry& telemetry() const override { return telemetry_; }
 ChassisCaps caps() const override {
 ChassisCaps output;
 output.geometry.support_width_m = params_.support_width_m;
 output.geometry.support_length_m = params_.support_length_m;
 output.max_yaw_rate_radps = params_.max_yaw_rate_radps;
 return output;
 }

 private:
 double encoder_measure(double wheel_mps, double dt) const;

 MotorDriveParams params_;
 PID left_pid_;
 PID right_pid_;
 ChassisTelemetry telemetry_;
};

} // namespace sim
