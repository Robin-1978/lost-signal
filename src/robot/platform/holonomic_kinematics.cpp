#include "robot/platform/holonomic_kinematics.hpp"

#include <algorithm>
#include <cmath>

#include "robot/core/geometry.hpp"

namespace lost_signal::robot::platform {

namespace {
constexpr std::array<double, 4> kXSign{1.0, 1.0, -1.0, -1.0};
constexpr std::array<double, 4> kYSign{1.0, -1.0, 1.0, -1.0};
}  // namespace

std::array<SwerveModuleCommand, 4> swerve_inverse(
    double forward_mps, double left_mps, double yaw_radps, double wheel_base_m,
    double track_width_m, const std::array<double, 4>& previous) {
  std::array<SwerveModuleCommand, 4> output;
  for (std::size_t i = 0; i < output.size(); ++i) {
    const double x = kXSign[i] * wheel_base_m * 0.5;
    const double y = kYSign[i] * track_width_m * 0.5;
    const double vx = forward_mps - yaw_radps * y;
    const double vy = left_mps + yaw_radps * x;
    double speed = std::hypot(vx, vy);
    double steer = speed > 1e-9 ? std::atan2(vy, vx) : previous[i];
    if (std::abs(core::wrap_angle(steer - previous[i])) > core::kPi * 0.5) {
      steer = core::wrap_angle(steer + core::kPi);
      speed = -speed;
    }
    output[i] = {speed, steer};
  }
  return output;
}

void swerve_forward(const std::array<double, 4>& wheel,
                    const std::array<double, 4>& steer, double wheel_base_m,
                    double track_width_m, double& forward, double& left,
                    double& yaw) {
  forward = 0.0;
  left = 0.0;
  yaw = 0.0;
  double denominator = 0.0;
  for (std::size_t i = 0; i < wheel.size(); ++i) {
    const double x = kXSign[i] * wheel_base_m * 0.5;
    const double y = kYSign[i] * track_width_m * 0.5;
    const double vx = wheel[i] * std::cos(steer[i]);
    const double vy = wheel[i] * std::sin(steer[i]);
    forward += vx;
    left += vy;
    yaw += -y * vx + x * vy;
    denominator += x * x + y * y;
  }
  forward *= 0.25;
  left *= 0.25;
  yaw /= std::max(0.01, denominator);
}

std::array<double, 4> mecanum_inverse(double forward, double left, double yaw,
                                      double wheel_base_m,
                                      double track_width_m) {
  const double radius = (wheel_base_m + track_width_m) * 0.5;
  return {forward - left - radius * yaw, forward + left + radius * yaw,
          forward + left - radius * yaw, forward - left + radius * yaw};
}

void mecanum_forward(const std::array<double, 4>& wheel, double wheel_base_m,
                     double track_width_m, double& forward, double& left,
                     double& yaw) {
  const double radius = std::max(0.05, (wheel_base_m + track_width_m) * 0.5);
  forward = (wheel[0] + wheel[1] + wheel[2] + wheel[3]) * 0.25;
  left = (-wheel[0] + wheel[1] + wheel[2] - wheel[3]) * 0.25;
  yaw = (-wheel[0] + wheel[1] - wheel[2] + wheel[3]) / (4.0 * radius);
}

}  // namespace lost_signal::robot::platform
