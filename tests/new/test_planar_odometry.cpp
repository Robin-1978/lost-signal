// Body-frame planar odometry contract. The oracle is the rigid transform of
// forward/left displacement into map axes; it does not reproduce the
// implementation's update expression through another helper.

#include "robot/autonomy/localization/dead_reckon.hpp"

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
 sim::Rng rng(17);
 sim::DeadReckon odometry;

 // At zero heading, +x body motion is +x map and +y body motion is +y map.
 odometry.reset(2.0, -1.0, 0.0);
 odometry.update(0.8, -0.3, 0.0, 2.0, rng, 0.0);
 CHECK(std::abs(odometry.p.x - 3.6) < 1e-12);
 CHECK(std::abs(odometry.p.y + 1.6) < 1e-12);

 // At +90 degrees, body-left points toward map -x. This is the component
 // that a scalar differential-drive odometry contract cannot represent.
 odometry.reset(0.0, 0.0, sim::kPi * 0.5);
 odometry.update(0.0, 1.0, 0.0, 1.0, rng, 0.0);
 CHECK(std::abs(odometry.p.x + 1.0) < 1e-12);
 CHECK(std::abs(odometry.p.y) < 1e-12);

 // Heading propagation rotates the same body twist into the new map frame.
 odometry.reset(0.0, 0.0, 0.0);
 odometry.update(0.0, 1.0, sim::kPi * 0.5, 1.0, rng, 0.0);
 CHECK(std::abs(odometry.th - sim::kPi * 0.5) < 1e-12);
 CHECK(std::abs(odometry.p.x + 1.0) < 1e-12);
 CHECK(std::abs(odometry.p.y) < 1e-12);

 std::cerr << "planar odometry green\n";
 return 0;
}
