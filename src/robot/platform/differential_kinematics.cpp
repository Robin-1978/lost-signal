#include "robot/platform/differential_kinematics.hpp"

#include <stdexcept>

namespace lost_signal::robot::platform {
void DifferentialKinematics::twist_to_wheels(double forward_mps, double yaw_rate_radps,
 double track_width_m, double& left_mps, double& right_mps) {
 if (track_width_m <= 0.0)
 throw std::invalid_argument("differential track width must be positive");
 left_mps = forward_mps - yaw_rate_radps * track_width_m / 2.0;
 right_mps = forward_mps + yaw_rate_radps * track_width_m / 2.0;
}

void DifferentialKinematics::wheels_to_twist(double left_mps, double right_mps,
 double track_width_m, double& forward_mps, double& yaw_rate_radps) {
 if (track_width_m <= 0.0)
 throw std::invalid_argument("differential track width must be positive");
 forward_mps = (left_mps + right_mps) / 2.0;
 yaw_rate_radps = (right_mps - left_mps) / track_width_m;
}
} // namespace lost_signal::robot::platform
