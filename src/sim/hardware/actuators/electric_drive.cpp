#include "sim/hardware/actuators/electric_drive.hpp"

#include <algorithm>
#include <cmath>

namespace sim::hardware {

double dc_motor_force_N(double normalized_duty, double wheel_mps,
 double stall_force_N, double free_speed_mps) {
 const double duty = std::clamp(normalized_duty, -1.0, 1.0);
 if (std::abs(duty) <= 1e-12)
 return 0.0;
 const bool motoring = duty * wheel_mps > 0.0;
 const double curve = motoring
 ? std::max(0.0, 1.0 - std::abs(wheel_mps) / std::max(0.05, free_speed_mps))
 : 1.0;
 return duty * std::max(0.0, stall_force_N) * curve;
}

} // namespace sim::hardware
