#pragma once

#include "robot/control/differential_wheel_controller.hpp"
#include "robot/control/pid.hpp"
#include "robot/platform/ackermann_kinematics.hpp"

namespace sim {

struct AckermannActuatorControllerConfig {
 double wheel_base_m{0.62};
 double track_width_m{0.58};
 double wheel_radius_m{0.1};
 double max_wheel_speed_mps{2.4};
 double max_center_steer_rad{0.55};
 double max_steer_rate_radps{2.0};
 int encoder_counts_per_revolution{2048};
 double drive_duty_limit{1.0};
 double drive_integral_limit{2.0};
 double steering_duty_limit{1.0};
 double steering_integral_limit{1.2};
 double steering_position_gain_radps_per_rad{2.5};
 double steering_velocity_gain_s{0.35};
};

struct AckermannActuatorFeedback {
 double rear_left_position_rad{0.0};
 double rear_right_position_rad{0.0};
 double rear_left_velocity_radps{0.0};
 double rear_right_velocity_radps{0.0};
 double front_left_steer_position_rad{0.0};
 double front_right_steer_position_rad{0.0};
 double front_left_steer_velocity_radps{0.0};
 double front_right_steer_velocity_radps{0.0};
 bool positions_valid{false};
};

struct SteeringControlTelemetry {
 double target_rad{0.0};
 double measured_rad{0.0};
 double normalized_duty{0.0};
};

struct AckermannActuatorControlOutput {
 WheelControlTelemetry rear_left;
 WheelControlTelemetry rear_right;
 SteeringControlTelemetry front_left_steer;
 SteeringControlTelemetry front_right_steer;
};

// Deployable Ackermann actuator controller. It owns linkage geometry,
// encoder quantization, rear-wheel speed loops and steering position loops;
// it consumes no simulator state or contact truth.
class AckermannActuatorController {
 public:
 AckermannActuatorController(AckermannActuatorControllerConfig config,
 WheelPidGains drive_gains);

 void reset();
 AckermannActuatorControlOutput update(const DiffCmd& command,
 const AckermannActuatorFeedback& feedback, double dt_s);
 const AckermannActuatorControllerConfig& config() const { return config_; }

 private:
 double encoder_speed(double position_rad, double& previous_count,
 double dt_s) const;

 AckermannActuatorControllerConfig config_;
 PID rear_left_pid_;
 PID rear_right_pid_;
 bool encoder_initialized_{false};
 double previous_rear_left_count_{0.0};
 double previous_rear_right_count_{0.0};
};

} // namespace sim
