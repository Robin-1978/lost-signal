#pragma once

namespace sim::hardware {

// Current-limited four-quadrant DC/H-bridge teaching model. Available
// motoring force falls toward the no-load speed. An opposing command retains
// the declared peak electrical-braking authority; zero duty is true coast.
double dc_motor_force_N(double normalized_duty, double wheel_mps,
 double stall_force_N, double free_speed_mps);

} // namespace sim::hardware
