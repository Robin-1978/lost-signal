#pragma once

#include "motor_drive.hpp"

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
};

struct HolonomicWheelCommand {
 double speed_mps{0.0};
 double steer_rad{0.0};
};

std::array<HolonomicWheelCommand, 4> swerve_inverse(double forward_mps, double left_mps,
 double yaw_radps, double wheel_base_m, double track_width_m,
 const std::array<double, 4>& previous_steer_rad = {});
std::array<double, 4> mecanum_inverse(double forward_mps, double left_mps, double yaw_radps,
 double wheel_base_m, double track_width_m);
void mecanum_forward(const std::array<double, 4>& wheel_mps, double wheel_base_m,
 double track_width_m, double& forward_mps, double& left_mps, double& yaw_radps);

class HolonomicDrive final : public ChassisModel {
 public:
 explicit HolonomicDrive(HolonomicParams params);
 DiffState step(const DiffState& state, const DiffCmd& command, double dt,
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
};

} // namespace sim
