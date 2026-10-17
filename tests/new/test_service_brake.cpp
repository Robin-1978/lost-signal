#include "sim/hardware/actuators/service_brake.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

#define CHECK(expr) do { if (!(expr)) { std::cerr << "CHECK failed: " #expr << '\n'; return 1; } } while (0)

int main() {
 sim::hardware::ServiceBrakePlant brake({8.0, 0.10, 0.20, 0.5});

 // Apply is rate limited, saturated, and always opposes signed rotation.
 CHECK(std::abs(brake.step(2.0, 4.0, 0.02) + 1.6) < 1e-9);
 CHECK(std::abs(brake.telemetry().applied - 0.2) < 1e-9);
 for (int i = 0; i < 10; ++i)
 (void)brake.step(1.0, 4.0, 0.02);
 CHECK(std::abs(brake.telemetry().delivered_torque_Nm + 8.0) < 1e-9);
 CHECK(std::abs(brake.step(1.0, -4.0, 0.02) - 8.0) < 1e-9);

 // Low-speed smoothing cannot create a finite kick through zero.
 const double near_zero = brake.step(1.0, 0.05, 0.02);
 CHECK(near_zero < 0.0);
 CHECK(std::abs(near_zero) < 1.0);
 CHECK(brake.step(1.0, 0.0, 0.02) == 0.0);

 // Release follows its independently authored response time.
 const double before_release = brake.telemetry().applied;
 (void)brake.step(0.0, 1.0, 0.02);
 CHECK(std::abs(brake.telemetry().applied - (before_release - 0.1)) < 1e-9);
 return EXIT_SUCCESS;
}
