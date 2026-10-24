#include <cassert>
#include <cstdio>

#include "robot/autonomy/behavior/behavior.hpp"
#include "robot/autonomy/planning/prediction.hpp"

int main() {
 using namespace lost_signal::robot;
  using namespace lost_signal::robot::autonomy;
  using limits_t = lost_signal::robot::platform::ChassisLimits;

  limits_t limits{.max_forward_mps = 2.0,
                  .max_yaw_rate_radps = 1.5,
                  .footprint_radius_m = 0.4};
 WaitOnIntrusionBehavior behavior;

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

 // A track behind the base is outside the forward conflict corridor and
 // must not turn the behavior layer into a global stop switch.
 PerceptionResult behind = clear;
 behind.tracks.push_back({9, -1.0, 0.0, 0.0, 0.0, 0.1});
 BehaviorDecision d_back = behavior.decide(est, behind, limits);
 assert(!d_back.must_hold);

 // A crossing track is held when its ROBOT-side constant-velocity forecast
 // reaches the ego footprint during the prediction horizon.
 PerceptionResult crossing = clear;
 crossing.tracks.push_back({10, 0.3, 0.45, 0.0, -0.4, 0.1});
 BehaviorDecision d_cross = behavior.decide(est, crossing, limits);
 assert(d_cross.must_hold);

 // The object starts beside the route and later crosses the robot's current
 // position. Holding would be unsafe; the route timing moves clear, so the
 // one-sided behavior layer leaves escape selection to the planner/validator.
 PerceptionResult approaching_side = clear;
 DynamicTrack side_track;
 side_track.id = 11;
 side_track.x_m = 0.3;
 side_track.y_m = 1.6;
 side_track.vy_mps = -1.0;
 side_track.prediction_ready = true;
 approaching_side.tracks.push_back(side_track);
 BehaviorDecision d_side = behavior.decide(est, approaching_side, limits);
 assert(!d_side.must_hold);
 assert(d_side.reason == "clear");

 // Conflict geometry follows the route rather than the robot's current
 // heading. This measured object is straight ahead, but the active route
 // turns north immediately and does not intersect it.
 Route north;
 north.valid = true;
 north.points.push_back({0.0, 0.0, 0.5 * core::kPi, 1});
 north.points.push_back({0.0, 3.0, 0.5 * core::kPi, 1});
 PerceptionResult off_route = clear;
 DynamicTrack off_route_track;
 off_route_track.id = 12;
 off_route_track.x_m = 1.0;
 off_route_track.y_m = 0.0;
 off_route_track.radius_m = 0.1;
 off_route.tracks.push_back(off_route_track);
 const PredictionBatch off_route_prediction = predict_perception(
 off_route, 0.0, {.horizon_s = 2.5, .frame_dt_s = 0.1});
 const BehaviorDecision d_off_route = behavior.decide(
 est, off_route, off_route_prediction, north, limits);
 assert(!d_off_route.must_hold);

 // A mature target that will cross the northbound route produces a
 // braking-derived cap before it becomes an immediate hold. The behavior
 // has no map labels or simulator identity; route and ROBOT forecast suffice.
 PerceptionResult route_crossing = clear;
 DynamicTrack route_track;
 route_track.id = 13;
 route_track.x_m = 0.40;
 route_track.y_m = 0.80;
 route_track.vx_mps = -1.0;
 route_track.radius_m = 0.05;
 route_track.prediction_ready = true;
 route_crossing.tracks.push_back(route_track);
 const PredictionBatch route_prediction = predict_perception(
 route_crossing, 0.0, {.horizon_s = 2.5, .frame_dt_s = 0.1});
 const BehaviorDecision d_route = behavior.decide(
 est, route_crossing, route_prediction, north, limits);
 assert(!d_route.must_hold);
 assert(d_route.max_forward_mps > 0.0);
 assert(d_route.max_forward_mps < limits.max_forward_mps);
 assert(d_route.reason == "route-conflict-speed-cap");

 est.forward_mps = limits.max_forward_mps;
 const BehaviorDecision d_route_fast = behavior.decide(
 est, route_crossing, route_prediction, north, limits);
 assert(d_route_fast.must_hold);
 assert(d_route_fast.reason == "route-conflict-hold");
 est.forward_mps = 0.0;

  // Unknown evidence (no scan) -> hold, never free space.
  PerceptionResult blind;
  BehaviorDecision d3 = behavior.decide(est, blind, limits);
  assert(d3.must_hold);

  std::printf("behavior ok\n");
  return 0;
}
