#include <cassert>
#include <cstdio>

#include "robot/autonomy/behavior/behavior.hpp"

int main() {
  using namespace lost_signal::robot::autonomy;
  using limits_t = lost_signal::robot::platform::ChassisLimits;

  limits_t limits{.max_forward_mps = 2.0,
                  .max_yaw_rate_radps = 1.5,
                  .footprint_radius_m = 0.4};
  WaitOnIntrusionBehavior behavior(2.0);

  EstimatedState est;
  est.valid = true;
  PerceptionResult clear;
  clear.scan_ok = true;

  // No tracks, valid evidence -> motion allowed at full chassis speed.
  BehaviorDecision d0 = behavior.decide(est, clear, limits);
  assert(!d0.must_hold);
  assert(d0.max_forward_mps == 2.0);

  // Track inside the hold radius -> wait, with a stable reason code.
  PerceptionResult intruded = clear;
  intruded.tracks.push_back({7, 1.0, 0.0, 0.0, 0.0, 0.1});
  BehaviorDecision d1 = behavior.decide(est, intruded, limits);
  assert(d1.must_hold);
  assert(d1.max_forward_mps == 0.0);

  // Track outside the radius -> clear.
  PerceptionResult far = clear;
  far.tracks.push_back({8, 10.0, 0.0, 0.0, 0.0, 0.1});
  BehaviorDecision d2 = behavior.decide(est, far, limits);
  assert(!d2.must_hold);

  // Unknown evidence (no scan) -> hold, never free space.
  PerceptionResult blind;
  BehaviorDecision d3 = behavior.decide(est, blind, limits);
  assert(d3.must_hold);

  std::printf("behavior ok\n");
  return 0;
}
