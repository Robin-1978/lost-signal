#include "robot/autonomy/diagnostics/chassis_diagnostics.hpp"

#include <iostream>

#define CHECK(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

int main() {
 sim::autonomy::ChassisDiagnosticsSystem diagnostics;
 diagnostics.reset();

 // The first sensed sample only seeds the derivative history.
 CHECK(!diagnostics.update_slip(true, 0.0, 0.0, 0.02));
 for (int step = 0; step < 30; ++step)
 diagnostics.update_slip(true, 3.0, 0.0, 0.02);
 CHECK(diagnostics.slip_detected());
 CHECK(diagnostics.slip_episodes() == 1);

 // Reset clears all temporal state owned by the ROBOT diagnostics module.
 diagnostics.reset();
 CHECK(!diagnostics.slip_detected());
 CHECK(diagnostics.slip_episodes() == 0);

 diagnostics.update_braking(0.0, {0.0, 0.0}, 1.0, 1.0);
 diagnostics.update_braking(0.1, {0.1, 0.0}, 1.0, 1.0);
 const auto& started = diagnostics.update_braking(0.2, {0.2, 0.0}, 0.9, 0.0);
 CHECK(started.active);
 diagnostics.update_braking(0.3, {0.28, 0.0}, 0.6, 0.0);
 const auto& stopped = diagnostics.update_braking(0.4, {0.32, 0.0}, 0.02, 0.0);
 CHECK(!stopped.active);
 CHECK(stopped.valid);

 std::cout << "chassis diagnostics ok\n";
 return 0;
}
