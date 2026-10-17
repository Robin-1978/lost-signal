#pragma once

#include <algorithm>
#include <cmath>

namespace sim {

// Exact discrete update for a rate command passing through a first-order
// actuator response. Hardware controllers and predictive rollouts share this
// primitive so a configured response time has one mathematical meaning.
inline double first_order_rate_step(double current_rate, double demanded_rate,
 double response_s, double dt_s) {
 const double blend = response_s > 0.0
 ? 1.0 - std::exp(-std::max(0.0, dt_s) / response_s)
 : 1.0;
 return current_rate + (demanded_rate - current_rate) * blend;
}

inline double position_servo_pwm(double target, double position,
 double max_rate, double response_s) {
 const double kp = 1.0 / std::max(0.05, response_s * std::max(0.05, max_rate));
 return std::clamp(kp * (target - position), -1.0, 1.0);
}

} // namespace sim
