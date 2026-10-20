#pragma once

#include "robot/control/pid.hpp"
#include "robot/platform/differential_kinematics.hpp"

namespace sim {

struct WheelPidGains {
    double kp{0.0};
    double ki{0.0};
    double kd{0.0};
};

struct WheelControllerDesign {
    double bandwidth_radps{6.0};
    double integral_time_s{5.0};
    double derivative_time_s{0.005};
};

WheelPidGains tune_wheel_pid(double wheel_mass_kg, double stall_force_N,
                             double free_speed_mps, double linearize_at_mps,
                             const WheelControllerDesign& design = {});

struct DifferentialWheelControllerConfig {
    double track_width_m{0.5};
    double wheel_radius_m{0.1};
    double max_wheel_speed_mps{2.4};
    int encoder_counts_per_revolution{2048};
    double duty_limit{1.0};
    double integral_limit{2.0};
 // Identified duty needed to overcome static drivetrain/contact resistance.
 // Applied only for a non-zero wheel-speed request; it never holds a stopped
 // wheel and therefore cannot hide a missing parking brake.
 double static_friction_feedforward_duty{0.0};
 // Parameters for converting an SI longitudinal acceleration request into
 // finite common wheel duty. A zero total mass disables this feedforward.
 double total_mass_kg{0.0};
 double stall_force_per_unit_N{0.0};
 double free_speed_mps{0.0};
 double driven_unit_count{2.0};
};

struct DifferentialWheelFeedback {
    double left_position_rad{0.0};
    double right_position_rad{0.0};
    double left_velocity_radps{0.0};
    double right_velocity_radps{0.0};
    bool positions_valid{false};
};

struct WheelControlTelemetry {
    double target_mps{0.0};
    double measured_mps{0.0};
    double actual_mps{0.0};
    double normalized_duty{0.0};
    double pid_p{0.0};
    double pid_i{0.0};
    double pid_d{0.0};
 double static_feedforward{0.0};
 double acceleration_feedforward{0.0};
};

struct DifferentialWheelControlOutput {
    WheelControlTelemetry left;
    WheelControlTelemetry right;
    double measured_forward_mps{0.0};
    double measured_yaw_radps{0.0};
};

// Deployable differential wheel-speed controller. It consumes only body-motion
// intent and encoder joint measurements; motor curves, contact, and rigid-body
// integration remain platform-provider responsibilities.
class DifferentialWheelController {
 public:
    DifferentialWheelController(DifferentialWheelControllerConfig config,
                                WheelPidGains gains);

    void reset();
    DifferentialWheelControlOutput update(const DiffCmd& command,
                                          const DifferentialWheelFeedback& feedback,
                                          double dt_s);
    const DifferentialWheelControllerConfig& config() const { return config_; }

 private:
    double encoder_measure(double position_rad, double& previous_count,
                           double dt_s) const;

    DifferentialWheelControllerConfig config_;
    PID left_pid_;
    PID right_pid_;
    bool encoder_initialized_{false};
    double previous_left_count_{0.0};
    double previous_right_count_{0.0};
 bool target_initialized_{false};
 double previous_left_target_mps_{0.0};
 double previous_right_target_mps_{0.0};
 bool controlled_stop_active_{false};
};

} // namespace sim
