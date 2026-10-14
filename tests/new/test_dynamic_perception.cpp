#include "algo/dynamic_obstacle_map.hpp"
#include "algo/predictor.hpp"
#include "hw/sensors/hal.hpp"
#include "hw/chassis/diff_drive.hpp"
#include "algo/dwa.hpp"
#include "algo/grid_map.hpp"
#include "hw/chassis/motor_drive.hpp"
#include "hw/sensors/sensor.hpp"

#include <cmath>
#include <iostream>
#include <limits>

#define CHECK(condition)                                                                                     \
 do {                                                                                                        \
  if (!(condition)) {                                                                                        \
   std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n";                           \
   return 1;                                                                                                 \
  }                                                                                                          \
 } while (false)

int main() {
 sim::GridMap open_map(200, 100, 0.1);
 const std::vector<sim::DynState> forklift{{{5.0, 5.0}, {0.5, 0.0}, 0.5}};
 sim::Rng open_rng(7);
 const auto visible = sim::lidar_scan(open_map, {1.0, 5.0}, 0.0, 4, 10.0, 0.0, open_rng, forklift);
 CHECK(visible.hit[0]);
 CHECK(visible.dynamic_hit[0] == 0);
 CHECK(std::abs(visible.ranges[0] - 3.5) < 1e-9);

 sim::GridMap occluded_map(200, 100, 0.1);
 occluded_map.addBox(3.0, 4.5, 3.5, 5.5);
 sim::Rng occluded_rng(7);
 const auto occluded = sim::lidar_scan(occluded_map, {1.0, 5.0}, 0.0, 4, 10.0, 0.0, occluded_rng, forklift);
 CHECK(occluded.hit[0]);
 CHECK(occluded.dynamic_hit[0] == -1);
 CHECK(occluded.ranges[0] < visible.ranges[0]);

 // HAL contract: SimLidar reproduces the direct raycast bit-for-bit.
 sim::SimLidar hal_lidar(open_map);
 sim::Rng hal_rng(7);
 sim::Rng direct_rng(7);
 (void)direct_rng;
 sim::Rng open_rng2(7);
 const auto direct = sim::lidar_scan(open_map, {1.0, 5.0}, 0.0, 4, 10.0, 0.0, open_rng2, forklift);
 const auto via_hal = hal_lidar.scan({1.0, 5.0}, 0.0, 4, 10.0, 0.0, hal_rng, forklift);
 CHECK(via_hal.ranges.size() == direct.ranges.size());
 for (std::size_t i = 0; i < via_hal.ranges.size(); ++i) {
  CHECK(via_hal.ranges[i] == direct.ranges[i]);
  CHECK(via_hal.hit[i] == direct.hit[i]);
  CHECK(via_hal.dynamic_hit[i] == direct.dynamic_hit[i]);
 }

 sim::DynamicObstacleMap dynamic_map;
 dynamic_map.observe({0, {5.0, 5.0}, 0.5}, 0.0);
 dynamic_map.observe({0, {5.2, 5.0}, 0.5}, 0.2);
 const auto moving = dynamic_map.active_tracks(0.2, 1.0);
 CHECK(moving.size() == 1);
 CHECK(moving[0].state.v.x > 0.8);

 dynamic_map.observe({0, {5.0, 5.0}, 0.5}, 0.4);
 const auto predictions = dynamic_map.predict(0.4, 1.0, 0.1, 1.0);
 CHECK(predictions.size() == 11);
 for (const auto& frame : predictions) {
  CHECK(frame.size() == 1);
  CHECK(frame[0].p.x >= 5.0 - 1e-9);
  CHECK(frame[0].p.x <= 5.2 + 1e-9);
 }

 const auto dwa = sim::dwa_compute({1.0, 2.0}, 0.0, 0.0, 0.0, {8.0, 2.0}, open_map, {1.0, 2.0, 0.3}, 1.5, 1.5,
                                   2.0, 1.5, 0.1, 0.38, 8.0);
 CHECK(dwa.best.valid);
 CHECK(!dwa.best_path.empty());
 CHECK(dwa.best_path.front().x == 1.0);
 CHECK(dwa.best_path.front().y == 2.0);
 CHECK(dwa.best_path.size() > 2);
 CHECK(!dwa.rollouts.empty());
 CHECK(dwa.rollouts.front().points.size() > 2);

 double left = 0.0;
 double right = 0.0;
 sim::DiffDrive::twistToWheels(1.0, 1.0, 0.52, left, right);
 CHECK(std::abs(left - 0.74) < 1e-12);
 CHECK(std::abs(right - 1.26) < 1e-12);

 const sim::WheelPidGains test_gains = sim::tune_wheel_pid(12.0, 54.0, 3.0, 1.2);
 CHECK(test_gains.kp > 1.0);
 CHECK(test_gains.kp < 4.0);
 sim::DifferentialMotorDrive motors(
   {
     0.52,
     0.10,
     2.4,
     4.2,
     54.0,
     3.0,
     24.0,
     2.4,
     2048,
     1.0,
     2.0,
   },
   test_gains);
 const sim::GroundContact test_ground{0.7, 0.01, 4.2};
 sim::DiffState driven;
 driven = motors.step(driven, {1.0, 0.6}, 0.02, test_ground);
 CHECK(motors.telemetry().left.pwm > 0.0);
 CHECK(motors.telemetry().right.target_mps > motors.telemetry().left.target_mps);
 CHECK(std::abs(motors.telemetry().left.pwm) <= 1.0);
 CHECK(std::abs(motors.telemetry().right.pwm) <= 1.0);
 for (int step = 0; step < 100; ++step) {
  driven = motors.step(driven, {1.0, 0.6}, 0.02, test_ground);
 }
 CHECK(driven.v > 0.7);
 CHECK(driven.w > 0.2);
 CHECK(std::abs(motors.telemetry().left.measured_mps) > 0.5);

 // predicted_gap: head-on approach shrinks to the known minimum, diverging
 // tracks stay wide, empty inputs report infinity, standing still still
 // detects an approaching threat.
 {
  const std::vector<sim::Vec2> path{{0.0, 0.0}, {10.0, 0.0}};
  const std::vector<std::vector<sim::DynState>> closing = {
    {{{5.0, 0.0}, {0.0, 0.0}, 0.5}},
    {{{4.0, 0.0}, {0.0, 0.0}, 0.5}},
    {{{3.0, 0.0}, {0.0, 0.0}, 0.5}},
  };
  // Ego at frame k stands at k m (frame 0 = now at x=0); swept intervals
  // [0,1]x[5,4] and [1,2]x[4,3] close to 1.0 m center distance at the end.
  const double gap = sim::predicted_gap(path, 0, 10.0, closing, 0.1, 0.38);
  CHECK(std::abs(gap - (1.0 - 0.5 - 0.38)) < 1e-9);
  const std::vector<std::vector<sim::DynState>> leaving = {
    {{{5.0, 5.0}, {0.0, 0.0}, 0.5}},
  };
  CHECK(sim::predicted_gap(path, 0, 10.0, leaving, 0.1, 0.38) > 4.0);
  CHECK(sim::predicted_gap({}, 0, 1.0, closing, 0.1, 0.38) ==
        std::numeric_limits<double>::infinity());
  CHECK(sim::predicted_gap(path, 0, 1.0, {}, 0.1, 0.38) == std::numeric_limits<double>::infinity());
  // Standing ego, threat driving at it: still caught.
  const std::vector<std::vector<sim::DynState>> incoming = {
    {{{2.0, 0.0}, {0.0, 0.0}, 0.5}},
  };
  CHECK(sim::predicted_gap(path, 0, 0.0, incoming, 0.1, 0.38) < 1.2);
 }
 // Swept intervals: a fast crosser passing between frames is caught exactly
 // (endpoints both read ~1.0 m, true minimum 0.1 m). Discrete sampling alone
 // would report clear; the segment check must not.
 {
  const std::vector<sim::Vec2> line{{0.0, 0.0}, {10.0, 0.0}};
  const std::vector<std::vector<sim::DynState>> crossing = {
    {{{-1.0, 0.1}, {0.0, 0.0}, 0.5}},
    {{{1.0, 0.1}, {0.0, 0.0}, 0.5}},
  };
  const double swept = sim::predicted_gap(line, 0, 0.0, crossing, 0.1, 0.38);
  CHECK(std::abs(swept - (0.1 - 0.5 - 0.38)) < 1e-9);
 }
 // earliest_departure: go-now when clear, positive wait for a passing threat,
 // infinity when nothing clears in horizon; empty inputs report infinity.
 // Paths are 2 m / 1 mps = 2 s traversals inside a 4 s horizon, so the
 // full-trajectory gate (must finish inside the horizon) is satisfiable.
 {
  const std::vector<sim::Vec2> line{{0.0, 0.0}, {2.0, 0.0}};
  const std::vector<std::vector<sim::DynState>> open(40, {{{5.0, 5.0}, {0.0, 0.0}, 0.4}});
  CHECK(sim::earliest_departure(line, 0, 1.0, open, 0.1, 0.38, 0.5) == 0.0);
  // Threat parked on the path for the whole horizon: no departure works.
  // Parked at 1.5 m so the wait position itself (gap 0.72) passes margin —
  // the infinity comes from the traversal, not from a bad wait spot.
  const std::vector<std::vector<sim::DynState>> blocked(40, {{{1.5, 0.0}, {0.0, 0.0}, 0.4}});
  CHECK(sim::earliest_departure(line, 0, 1.0, blocked, 0.1, 0.38, 0.5) ==
        std::numeric_limits<double>::infinity());
  // Threat clears after 1 s: must wait, then go.
  std::vector<std::vector<sim::DynState>> clearing;
  for (int k = 0; k < 40; ++k) {
   if (k < 10) {
    clearing.push_back({{{1.5, 0.0}, {0.0, 0.0}, 0.4}});
   } else {
    clearing.push_back({{{5.0, 5.0}, {0.0, 0.0}, 0.4}});
   }
  }
  const double dep = sim::earliest_departure(line, 0, 1.0, clearing, 0.1, 0.38, 0.5);
  CHECK(dep > 0.0 && dep < std::numeric_limits<double>::infinity());
  CHECK(sim::earliest_departure({}, 0, 1.0, open, 0.1, 0.38, 0.5) ==
        std::numeric_limits<double>::infinity());
  CHECK(sim::earliest_departure(line, 0, 0.0, open, 0.1, 0.38, 0.5) ==
        std::numeric_limits<double>::infinity());
 }
 return 0;
}
