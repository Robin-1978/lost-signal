#pragma once

// SIM-owned drive-motor model (World internals, never ROBOT input).
// Current-limited four-quadrant DC/H-bridge teaching model, ported verbatim
// from the main-stack electric_drive: available motoring force falls toward
// the no-load speed; an opposing command retains the declared peak
// electrical-braking authority; zero duty is true coast.

namespace lost_signal::robot::platform {
struct DriveProfile;  // ROBOT hardware contract; SIM reads, never writes.
}  // namespace lost_signal::robot::platform

namespace lost_signal::sim::world {

double dc_motor_force_N(double normalized_duty, double wheel_mps,
                        double stall_force_N, double free_speed_mps);

struct MotorParams;  // defined in world.hpp; converted here from ROBOT data.

// SIM-side conversion from the ROBOT hardware contract: one set of numbers
// drives sim and native loops. Geometry + motor numbers carry over; brake
// capacity stays SIM-side (no ROBOT brake channel exists).
MotorParams motor_params_from_profile(
    const robot::platform::DriveProfile& profile);

}  // namespace lost_signal::sim::world
