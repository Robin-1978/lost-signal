#include "autonomy/perception/dynamic_obstacle_map.hpp"
#include "autonomy/planning/local/predictor.hpp"

#include <cmath>
#include <iostream>

#define CHECK(condition)                                                                                     \
 do {                                                                                                        \
  if (!(condition)) {                                                                                        \
   std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n";                           \
   return 1;                                                                                                 \
  }                                                                                                          \
 } while (false)

// Differential test for the predictor-module split. The tracker's predict()
// must stay bit-identical to the standalone predict_frames() fed from
// predict_inputs() on every scenario: straight motion, reflection boundary,
// multi-target, non-zero age, and stale exclusion. If this test ever flips,
// the split changed behavior (text-equivalence means nothing here).
int check_parity(sim::DynamicObstacleMap& map, double time_s, double horizon_s, double frame_dt_s,
                 double max_age_s) {
 const auto legacy = map.predict(time_s, horizon_s, frame_dt_s, max_age_s);
 const auto modular = sim::predict_frames(map.predict_inputs(), time_s, horizon_s, frame_dt_s, max_age_s);
 CHECK(legacy.size() == modular.size());
 for (std::size_t frame = 0; frame < legacy.size(); ++frame) {
  CHECK(legacy[frame].size() == modular[frame].size());
  for (std::size_t i = 0; i < legacy[frame].size(); ++i) {
   CHECK(legacy[frame][i].p.x == modular[frame][i].p.x);
   CHECK(legacy[frame][i].p.y == modular[frame][i].p.y);
   CHECK(legacy[frame][i].v.x == modular[frame][i].v.x);
   CHECK(legacy[frame][i].v.y == modular[frame][i].v.y);
   CHECK(legacy[frame][i].radius == modular[frame][i].radius);
  }
 }
 return 0;
}

int main() {
 // A: straight motion, zero age.
 {
  sim::DynamicObstacleMap map;
  map.observe({0, {5.0, 5.0}, 0.5}, 0.0);
  map.observe({0, {5.2, 5.0}, 0.5}, 0.2);
  CHECK(check_parity(map, 0.2, 1.0, 0.1, 1.0) == 0);
  CHECK(map.predict(0.2, 1.0, 0.1, 1.0).size() == 11);
 }
 // B: reversal pins a bounded corridor; reflection must clamp inside it.
 {
  sim::DynamicObstacleMap map;
  map.observe({0, {5.0, 5.0}, 0.5}, 0.0);
  map.observe({0, {5.2, 5.0}, 0.5}, 0.2);
  map.observe({0, {5.0, 5.0}, 0.5}, 0.4);
  CHECK(check_parity(map, 0.4, 1.0, 0.1, 1.0) == 0);
  for (const auto& frame : map.predict(0.4, 1.0, 0.1, 1.0)) {
   CHECK(frame.size() == 1);
   CHECK(frame[0].p.x >= 5.0 - 1e-9);
   CHECK(frame[0].p.x <= 5.2 + 1e-9);
  }
 }
 // C: multi-target with different velocities and radii.
 {
  sim::DynamicObstacleMap map;
  map.observe({0, {1.0, 1.0}, 0.4}, 0.0);
  map.observe({1, {9.0, 2.0}, 0.6}, 0.0);
  map.observe({0, {1.4, 1.2}, 0.4}, 0.2);
  map.observe({1, {8.4, 2.4}, 0.6}, 0.2);
  CHECK(check_parity(map, 0.2, 0.5, 0.1, 1.0) == 0);
  CHECK(map.predict(0.2, 0.5, 0.1, 1.0).front().size() == 2);
 }
 // D: non-zero age (gap since last observation); both paths advance equally.
 {
  sim::DynamicObstacleMap map;
  map.observe({0, {3.0, 3.0}, 0.5}, 0.0);
  map.observe({0, {3.4, 3.0}, 0.5}, 0.2);
  CHECK(check_parity(map, 0.7, 1.0, 0.1, 1.0) == 0);
  const auto frames = map.predict(0.7, 1.0, 0.1, 1.0);
  CHECK(!frames.empty() && frames.front().size() == 1);
  // Age-advanced position must exceed the last observation (moving +x).
  CHECK(frames.front()[0].p.x > 3.4);
 }
 // E: stale track excluded by max_age on both paths.
 {
  sim::DynamicObstacleMap map;
  map.observe({0, {0.0, 0.0}, 0.5}, 0.0);
  map.observe({0, {0.4, 0.0}, 0.5}, 0.2);
  CHECK(check_parity(map, 5.0, 1.0, 0.1, 1.0) == 0);
  CHECK(map.predict(5.0, 1.0, 0.1, 1.0).front().empty());
 }
 std::cout << "predictor parity ok\n";
 return 0;
}
