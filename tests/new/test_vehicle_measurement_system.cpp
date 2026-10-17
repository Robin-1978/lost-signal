#include "sim/game/mission/systems/vehicle_measurement_system.hpp"

#include <cmath>
#include <iostream>

#define CHECK(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

int main() {
 mission::VehicleMeasurementSystem instrument;
 instrument.reset();

 instrument.update(0.0, {0.0, 0.0}, 1.0, 1.0);
 instrument.update(0.1, {0.1, 0.0}, 1.0, 1.0);
 const auto& started = instrument.update(0.2, {0.2, 0.0}, 0.9, 0.0);
 CHECK(started.active);
 CHECK(!started.valid);
 CHECK(std::abs(started.start_speed_mps - 0.9) < 1e-12);

 instrument.update(0.3, {0.28, 0.0}, 0.6, 0.0);
 const auto& stopped = instrument.update(0.4, {0.32, 0.0}, 0.02, 0.0);
 CHECK(!stopped.active);
 CHECK(stopped.valid);
 CHECK(std::abs(stopped.elapsed_s - 0.2) < 1e-12);
 CHECK(std::abs(stopped.distance_m - 0.12) < 1e-12);

 // A renewed command cancels an incomplete run instead of publishing a
 // shorter, fictitious stopping distance.
 instrument.update(1.0, {1.0, 0.0}, 1.0, 1.0);
 instrument.update(1.1, {1.1, 0.0}, 0.9, 0.0);
 const auto& cancelled = instrument.update(1.2, {1.2, 0.0}, 0.8, 0.5);
 CHECK(!cancelled.active);
 CHECK(!cancelled.valid);

 instrument.reset();
 CHECK(!instrument.braking().active);
 CHECK(!instrument.braking().valid);
 return 0;
}
