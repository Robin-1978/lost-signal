#include "robot/platform/ackermann_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace lost_signal::robot::platform {

namespace {

void require_geometry(double wheel_base_m, double track_width_m) {
  if (!std::isfinite(wheel_base_m) || !std::isfinite(track_width_m) ||
      wheel_base_m <= 0.0 || track_width_m <= 0.0) {
    throw std::invalid_argument(
        "Ackermann geometry must be finite and positive");
  }
}

}  // namespace

AckermannSteeringAngles ackermann_steering_angles(double center_angle_rad,
                                                 double wheel_base_m,
                                                 double track_width_m) {
  require_geometry(wheel_base_m, track_width_m);
  if (!std::isfinite(center_angle_rad))
    throw std::invalid_argument("Ackermann steering angle must be finite");
  if (std::abs(center_angle_rad) < 1e-12)
    return {center_angle_rad, center_angle_rad};

  const double radius_m = wheel_base_m / std::tan(std::abs(center_angle_rad));
  if (radius_m <= track_width_m * 0.5)
    throw std::invalid_argument(
        "Ackermann steering angle places the turn centre inside the rear "
        "track");
  const double inner_rad =
      std::atan2(wheel_base_m, radius_m - track_width_m * 0.5);
  const double outer_rad =
      std::atan2(wheel_base_m, radius_m + track_width_m * 0.5);
  if (center_angle_rad > 0.0) return {inner_rad, outer_rad};
  return {-outer_rad, -inner_rad};
}

AckermannRearWheelSpeeds ackermann_rear_wheel_speeds(double center_speed_mps,
                                                     double center_angle_rad,
                                                     double wheel_base_m,
                                                     double track_width_m) {
  require_geometry(wheel_base_m, track_width_m);
  if (!std::isfinite(center_speed_mps) || !std::isfinite(center_angle_rad))
    throw std::invalid_argument("Ackermann command must be finite");
  const double half_track_curvature =
      track_width_m * std::tan(center_angle_rad) / (2.0 * wheel_base_m);
  return {
      center_speed_mps * (1.0 - half_track_curvature),
      center_speed_mps * (1.0 + half_track_curvature),
  };
}

}  // namespace lost_signal::robot::platform
