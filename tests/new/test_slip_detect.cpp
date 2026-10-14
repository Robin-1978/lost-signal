#include "algo/slip_detect.hpp"

#include <iostream>

#define CHECK(condition)                                                                                     \
 do {                                                                                                        \
  if (!(condition)) {                                                                                        \
   std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n";                           \
   return 1;                                                                                                 \
  }                                                                                                          \
 } while (false)

// Pure-algorithm tests: the detector sees only sensed accelerations, never
// model truth. Game wiring waits on the quarter-car wheel/body split (see
// the input contract in slip_detect.hpp); these tests pin the algorithm so
// the wiring lands on validated ground.
int main() {
 constexpr double kDt = 0.02;
 // Silence: identical inputs (plus the zero-noise idealization) never latch,
 // even over long runs.
 {
  sim::SlipDetector detector(0.15);
  bool ever = false;
  for (int step = 0; step < 500; ++step) {
   ever = detector.step(1.0, 1.0, kDt) || ever;
  }
  CHECK(!ever);
  CHECK(detector.episodes() == 0);
 }
 // Sustained divergence latches exactly one episode, stays latched while the
 // divergence persists, and releases after the signals rejoin.
 {
  sim::SlipDetector detector(0.15);
  for (int step = 0; step < 10; ++step) {
   CHECK(!detector.step(0.0, 0.0, kDt));
  }
  bool latched = false;
  for (int step = 0; step < 30; ++step) {
   latched = detector.step(3.0, 0.0, kDt);
  }
  CHECK(latched);
  CHECK(detector.episodes() == 1);
  for (int step = 0; step < 60; ++step) {
   latched = detector.step(0.0, 0.0, kDt);
  }
  CHECK(!latched);
  CHECK(detector.episodes() == 1);
 }
 // Single-step spike (quantization-class transient) never latches.
 {
  sim::SlipDetector detector(0.15);
  CHECK(!detector.step(5.0, 0.0, kDt));
  for (int step = 0; step < 30; ++step) {
   CHECK(!detector.step(0.0, 0.0, kDt));
  }
  CHECK(detector.episodes() == 0);
 }
 // Near-perfect IMU still respects the floor: tiny residue stays silent.
 {
  sim::SlipDetector detector(0.01);
  bool ever = false;
  for (int step = 0; step < 200; ++step) {
   ever = detector.step(0.2, 0.0, kDt) || ever;
  }
  CHECK(!ever);
 }
 std::cout << "slip detector ok\n";
 return 0;
}
