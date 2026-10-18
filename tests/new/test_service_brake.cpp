#include "sim/hardware/actuators/service_brake.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

#define CHECK(expr) do { if (!(expr)) { std::cerr << "CHECK failed: " #expr << '\n'; return 1; } } while (0)

int main() {
 sim::hardware::ServiceBrakePlant brake({8.0, 0.10, 0.20, 0.5});

 // The actuator exposes a rate-limited passive torque capacity. Direction is
 // resolved from joint velocity by the physics provider at substep cadence.
 CHECK(std::abs(brake.step(2.0, 0.02) - 1.6) < 1e-9);
 CHECK(std::abs(brake.telemetry().applied - 0.2) < 1e-9);
 for (int i = 0; i < 10; ++i)
 (void)brake.step(1.0, 0.02);
 CHECK(std::abs(brake.telemetry().available_torque_Nm - 8.0) < 1e-9);

 // Release follows its independently authored response time.
 const double before_release = brake.telemetry().applied;
 (void)brake.step(0.0, 0.02);
 CHECK(std::abs(brake.telemetry().applied - (before_release - 0.1)) < 1e-9);
 return EXIT_SUCCESS;
}
