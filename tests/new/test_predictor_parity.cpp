#include "robot/autonomy/perception/dynamic_obstacle_map.hpp"
#include "robot/autonomy/planning/local/predictor.hpp"

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
 const auto typed = map.predict_typed(
 time_s, horizon_s, frame_dt_s, max_age_s);
 const auto flattened = sim::flatten_prediction_frames(typed);
 CHECK(legacy.size() == modular.size());
 CHECK(legacy.size() == flattened.size());
 for (std::size_t frame = 0; frame < legacy.size(); ++frame) {
  CHECK(legacy[frame].size() == modular[frame].size());
 CHECK(legacy[frame].size() == flattened[frame].size());
  for (std::size_t i = 0; i < legacy[frame].size(); ++i) {
   CHECK(legacy[frame][i].p.x == modular[frame][i].p.x);
   CHECK(legacy[frame][i].p.y == modular[frame][i].p.y);
   CHECK(legacy[frame][i].v.x == modular[frame][i].v.x);
   CHECK(legacy[frame][i].v.y == modular[frame][i].v.y);
   CHECK(legacy[frame][i].radius == modular[frame][i].radius);
 CHECK(legacy[frame][i].p.x == flattened[frame][i].p.x);
 CHECK(legacy[frame][i].p.y == flattened[frame][i].p.y);
 CHECK(legacy[frame][i].v.x == flattened[frame][i].v.x);
 CHECK(legacy[frame][i].v.y == flattened[frame][i].v.y);
  }
 }
 return 0;
}

int main() {
 // A behavior proof is latched in its original clock domain. Local planning
 // consumes only the suffix after the elapsed execution time, interpolated
 // at exactly t=0; already executed motion must never be replayed.
 {
 const std::vector<sim::TimedPose> complete{
 {0.0, {0.0, 0.0}},
 {1.0, {1.0, 0.0}},
 {2.0, {1.0, 0.0}},
 {3.0, {1.0, 1.0}},
 };
 const auto remaining = sim::remaining_timed_trajectory(complete, 1.5);
 CHECK(remaining.size() == 3);
 CHECK(remaining.front().t_s == 0.0);
 CHECK(remaining.front().p.x == 1.0);
 CHECK(remaining.front().p.y == 0.0);
 CHECK(remaining[1].t_s == 0.5);
 CHECK(remaining[2].t_s == 1.5);
 CHECK(sim::remaining_timed_trajectory(complete, 4.0).size() == 1);

 // The admitted clock band is two-sided. A measured body ahead of nominal
 // consumes the fast side, a lagging body consumes the slow side, and a
 // larger mismatch clamps at the original admission bound.
 CHECK(std::abs(sim::match_timed_trajectory_phase(
 complete, {0.68, 0.0}, 0.5, 0.2) - 0.68) < 1e-12);
 CHECK(std::abs(sim::match_timed_trajectory_phase(
 complete, {0.32, 0.0}, 0.5, 0.2) - 0.32) < 1e-12);
 CHECK(std::abs(sim::match_timed_trajectory_phase(
 complete, {0.95, 0.0}, 0.5, 0.2) - 0.70) < 1e-12);
 // A stationary dwell has no observable spatial phase. Keep nominal time
 // instead of jumping to an arbitrary endpoint.
 CHECK(std::abs(sim::match_timed_trajectory_phase(
 complete, {1.0, 0.0}, 1.5, 0.2) - 1.5) < 1e-12);
 }
 // A: straight motion, zero age.
 {
  sim::DynamicObstacleMap map;
  map.observe({0, {5.0, 5.0}, 0.5}, 0.0);
 for (int sample = 1; sample <= 10; ++sample)
 map.observe({0, {5.0 + sample * 0.1, 5.0}, 0.5}, sample * 0.1);
 CHECK(check_parity(map, 1.0, 1.0, 0.1, 1.0) == 0);
 CHECK(map.predict(1.0, 1.0, 0.1, 1.0).size() == 11);
 const auto typed = map.predict_typed(1.0, 1.0, 0.1, 1.0);
 CHECK(typed.future_times_s.size() == 11);
 CHECK(typed.hypotheses.size() == 1);
 CHECK(typed.hypotheses.front().track_id == 0);
 CHECK(typed.hypotheses.front().hypothesis_id == 0);
 CHECK(typed.hypotheses.front().model ==
 sim::PredictionModel::ConstantVelocity);
 CHECK(typed.hypotheses.front().states.size() == 11);
 }
 // B: both endpoint reversals pin a bounded corridor; one reversal alone
 // must not reinterpret the still-growing observation frontier as an end.
 {
  sim::DynamicObstacleMap map;
  map.observe({0, {5.0, 5.0}, 0.5}, 0.0);
 for (int sample = 1; sample <= 10; ++sample)
 map.observe({0, {5.0 + sample * 0.1, 5.0}, 0.5}, sample * 0.1);
 for (int sample = 11; sample <= 16; ++sample)
 map.observe({0, {6.0, 5.0}, 0.5}, sample * 0.1);
 for (int sample = 1; sample <= 10; ++sample)
 map.observe({0, {6.0 - sample * 0.1, 5.0}, 0.5}, 1.6 + sample * 0.1);
CHECK(!map.predict_inputs().front().bounded_x);
for (int sample = 27; sample <= 32; ++sample)
 map.observe({0, {5.0, 5.0}, 0.5}, sample * 0.1);
for (int sample = 1; sample <= 10; ++sample)
 map.observe({0, {5.0 + sample * 0.1, 5.0}, 0.5}, 3.2 + sample * 0.1);
CHECK(map.predict_inputs().front().bounded_x);
CHECK(check_parity(map, 4.2, 1.0, 0.1, 1.0) == 0);
 const auto typed = map.predict_typed(4.2, 1.0, 0.1, 1.0);
 CHECK(typed.hypotheses.size() == 3);
 for (std::size_t index = 0; index < typed.hypotheses.size(); ++index) {
 CHECK(typed.hypotheses[index].track_id == 0);
 CHECK(typed.hypotheses[index].hypothesis_id == index);
 CHECK(typed.hypotheses[index].model ==
 sim::PredictionModel::ReflectedCorridor);
 }
for (const auto& frame : map.predict(4.2, 1.0, 0.1, 1.0)) {
 CHECK(frame.size() == 3);
   for (const auto& hypothesis : frame) {
 CHECK(hypothesis.p.x >= 4.9);
 CHECK(hypothesis.p.x <= 6.1);
   }
  }
 }
 // C: multi-target with different velocities and radii.
 {
  sim::DynamicObstacleMap map;
  map.observe({0, {1.0, 1.0}, 0.4}, 0.0);
  map.observe({1, {9.0, 2.0}, 0.6}, 0.0);
 for (int sample = 1; sample <= 10; ++sample) {
 map.observe({0, {1.0 + sample * 0.20, 1.0 + sample * 0.10}, 0.4}, sample * 0.1);
 map.observe({1, {9.0 - sample * 0.25, 2.0 + sample * 0.15}, 0.6}, sample * 0.1);
 }
 CHECK(check_parity(map, 1.0, 0.5, 0.1, 1.0) == 0);
 CHECK(map.predict(1.0, 0.5, 0.1, 1.0).front().size() == 2);
 const auto typed = map.predict_typed(1.0, 0.5, 0.1, 1.0);
 CHECK(typed.hypotheses.size() == 2);
 CHECK(typed.hypotheses[0].track_id == 0);
 CHECK(typed.hypotheses[1].track_id == 1);
 }
 // D: non-zero age (gap since last observation); both paths advance equally.
 {
  sim::DynamicObstacleMap map;
  map.observe({0, {3.0, 3.0}, 0.5}, 0.0);
 for (int sample = 1; sample <= 10; ++sample)
 map.observe({0, {3.0 + sample * 0.1, 3.0}, 0.5}, sample * 0.1);
 CHECK(check_parity(map, 1.5, 1.0, 0.1, 1.0) == 0);
 const auto frames = map.predict(1.5, 1.0, 0.1, 1.0);
  CHECK(!frames.empty() && frames.front().size() == 1);
  // Age-advanced position must exceed the last observation (moving +x).
 CHECK(frames.front()[0].p.x > 4.0);
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
