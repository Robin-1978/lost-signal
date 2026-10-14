// Stage-1 boundary pins for the three chassis kinematic pairs (the agreed FF
// route): round-trips at extremes plus the documented non-inverses. The happy
// paths already live in test_chassis.cpp; this file pins the edges the FF
// mapping stands on: linearity everywhere for twins, the Ackermann 0.5 m/s
// floor (policy, pins the intentional non-inverse), tracked reuse.

#include "hw/chassis/diff_drive.hpp"
#include "hw/chassis/ackermann.hpp"

#include <cmath>
#include <iostream>

#define CHECK(condition)                                                                                     \
 do {                                                                                                        \
  if (!(condition)) {                                                                                        \
   std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n";                           \
   return 1;                                                                                                 \
  }                                                                                                          \
 } while (false)

int main() {
 // Differential pair: exact mutual inverse incl. extremes and reverse.
 {
  const double bases[2] = {0.52, 0.58};
  const double cases[4][2] = {{2.4, 2.0}, {-1.0, -2.0}, {0.0, 0.0}, {3.0, -5.0}};
  for (const double base : bases) {
   for (const auto& twist : cases) {
    double left = 0.0, right = 0.0;
    sim::DiffDrive::twistToWheels(twist[0], twist[1], base, left, right);
    double v = 0.0, w = 0.0;
    sim::DiffDrive::wheelsToTwist(left, right, base, v, w);
    CHECK(std::abs(v - twist[0]) < 1e-12);
    CHECK(std::abs(w - twist[1]) < 1e-12);
   }
  }
 }
 // Tracked reuses the differential pair (no dedicated pair exists): same
 // linearity at a tracked base. Scrub lives in dynamics, not kinematics.
 {
  double left = 0.0, right = 0.0;
  sim::DiffDrive::twistToWheels(1.0, 1.5, 0.58, left, right);
  double v = 0.0, w = 0.0;
  sim::DiffDrive::wheelsToTwist(left, right, 0.58, v, w);
  CHECK(std::abs(v - 1.0) < 1e-12 && std::abs(w - 1.5) < 1e-12);
 }
 // Ackermann floor: below 0.5 m/s the pair is intentionally not inverse
 // (control policy: a car cannot turn standing still). Pin the boundary.
 {
  const auto slow = sim::steer_for_twist(0.4, 0.2, 0.62, 0.55);
  CHECK(slow.feasible);
  const double w_back = sim::yaw_for_steer(0.2, slow.delta_rad, 0.62);
  CHECK(std::abs(w_back - 0.4) > 1e-6);
  // Above the floor the pair is inverse.
  const auto cruise = sim::steer_for_twist(0.4, 1.0, 0.62, 0.55);
  CHECK(cruise.feasible);
  CHECK(std::abs(sim::yaw_for_steer(1.0, cruise.delta_rad, 0.62) - 0.4) < 1e-9);
  // Standstill demand freezes geometry instead of dividing by zero.
  const auto still = sim::steer_for_twist(0.4, 0.0, 0.62, 0.55);
  CHECK(still.feasible);
  CHECK(std::isfinite(still.delta_rad));
 }
 std::cerr << "inverse consistency green\n";
 return 0;
}
