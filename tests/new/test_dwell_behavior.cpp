#include "algo/dynobs.hpp"

#include <cmath>
#include <iostream>

#define CHECK(condition)                                                                                     \
 do {                                                                                                        \
  if (!(condition)) {                                                                                        \
   std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n";                           \
   return 1;                                                                                                 \
  }                                                                                                          \
 } while (false)

// Dwell behavior contract: a wander loop with timed stops. Pure time
// function: same t gives bit-identical state, velocity is exactly zero while
// dwelling, empty waits degenerate to plain wander.
int main() {
 const sim::Dwell route{{{0.0, 0.0}, {4.0, 0.0}}, {2.0, 0.0}, 2.0, 0.25};
 // Cycle: wait 2s at (0,0), travel 2s to (4,0), wait 0s, travel 2s back = 6s.
 auto at = dwell_state(route, 1.0);
 CHECK(at.p.x == 0.0 && at.p.y == 0.0);
 CHECK(at.v.x == 0.0 && at.v.y == 0.0);
 at = dwell_state(route, 3.0);
 CHECK(std::abs(at.p.x - 2.0) < 1e-9 && at.p.y == 0.0);
 CHECK(std::abs(at.v.x - 2.0) < 1e-9 && at.v.y == 0.0);
 at = dwell_state(route, 5.0);
 CHECK(std::abs(at.p.x - 2.0) < 1e-9 && at.p.y == 0.0);
 CHECK(std::abs(at.v.x + 2.0) < 1e-9 && at.v.y == 0.0);
 // Periodicity: t and t + 6s coincide exactly.
 for (const double t : {0.0, 0.7, 2.5, 4.2, 5.9}) {
  const auto a = dwell_state(route, t);
  const auto b = dwell_state(route, t + 6.0);
  CHECK(std::abs(a.p.x - b.p.x) < 1e-9 && std::abs(a.p.y - b.p.y) < 1e-9 &&
        std::abs(a.v.x - b.v.x) < 1e-9 && std::abs(a.v.y - b.v.y) < 1e-9);
 }
 // Empty waits == wander on the same loop.
 const sim::Dwell plain{{{0.0, 0.0}, {4.0, 0.0}}, {}, 2.0, 0.25};
 const sim::Wander loop{{{0.0, 0.0}, {4.0, 0.0}}, 2.0, 0.25};
 for (const double t : {0.0, 1.3, 3.1, 7.7}) {
  const auto a = dwell_state(plain, t);
  const auto b = sim::wander_state(loop, t);
  CHECK(std::abs(a.p.x - b.p.x) < 1e-9 && std::abs(a.p.y - b.p.y) < 1e-9);
 }
 std::cout << "dwell contract holds\n";
 return 0;
}
