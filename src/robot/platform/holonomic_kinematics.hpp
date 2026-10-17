#pragma once

#include <array>

namespace sim {

struct SwerveModuleCommand {
 double speed_mps{0.0};
 double steer_rad{0.0};
};

std::array<SwerveModuleCommand, 4> swerve_inverse(double forward_mps, double left_mps, double yaw_radps,
                                                  double wheel_base_m, double track_width_m,
                                                  const std::array<double, 4>& previous_steer_rad = {});
void swerve_forward(const std::array<double, 4>& wheel_mps, const std::array<double, 4>& steer_rad,
                    double wheel_base_m, double track_width_m, double& forward_mps, double& left_mps,
                    double& yaw_radps);
std::array<double, 4> mecanum_inverse(double forward_mps, double left_mps, double yaw_radps,
                                      double wheel_base_m, double track_width_m);
void mecanum_forward(const std::array<double, 4>& wheel_mps, double wheel_base_m, double track_width_m,
                     double& forward_mps, double& left_mps, double& yaw_radps);

} // namespace sim
