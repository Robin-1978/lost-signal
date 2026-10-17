#pragma once

#include "sim/hardware/vehicle/motor_drive.hpp"
#include "robot/platform/holonomic_kinematics.hpp"

#include <array>

namespace sim {

enum class HolonomicKind { Swerve, Mecanum };

struct HolonomicParams {
 HolonomicKind kind{HolonomicKind::Swerve};
 double wheel_base_m{0.6};
 double track_width_m{0.55};
 double max_wheel_speed_mps{2.4};
 double max_accel_mps2{4.0};
 double max_yaw_accel_radps2{6.0};
 double mass_kg{26.0};
 double yaw_inertia_kgm2{2.8};
 double max_yaw_rate_radps{2.0};
 double wheel_radius_m{0.1};
 double stall_force_N{54.0};
 double free_speed_mps{3.0};
 double pwm_limit{1.0};
 double steering_stall_torque_Nm{8.0};
};

class HolonomicDrive final : public ChassisModel {
 public:
 explicit HolonomicDrive(HolonomicParams params);
 void set_body_dynamics(const ChassisBodyDynamics& dynamics) override;
 void set_actuator_feedback(const ChassisActuatorFeedback& feedback) override { feedback_ = feedback; }
 DiffState step(const DiffState& state, const DiffCmd& command, double dt,
 const GroundContact& ground) override;
 bool accepts_actuator_duty() const override { return true; }
 DiffState step_actuator_duty(const DiffState& state, const DiffCmd& command,
                              const ChassisActuatorDutyCommand& control, double dt,
 const GroundContact& ground) override;
 const ChassisTelemetry& telemetry() const override { return telemetry_; }
 ChassisCaps caps() const override {
 ChassisCaps output;
 output.command_space = ChassisCommandSpace::HolonomicTwist;
 output.geometry.support_width_m = params_.track_width_m;
 output.geometry.support_length_m = params_.wheel_base_m;
 output.max_yaw_rate_radps = params_.max_yaw_rate_radps;
 output.drive_force_units = 4.0;
 return output;
 }

 private:
 HolonomicParams params_;
 ChassisTelemetry telemetry_;
 ChassisActuatorFeedback feedback_;
};

} // namespace sim
