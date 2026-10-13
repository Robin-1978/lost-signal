#include "sim/dynamic_obstacle_map.hpp"
#include "sim/diff_drive.hpp"
#include "sim/dwa.hpp"
#include "sim/grid_map.hpp"
#include "sim/motor_drive.hpp"
#include "sim/sensor.hpp"

#include <cmath>
#include <iostream>

#define CHECK(condition) do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n"; \
 return 1; \
 } \
} while (false)

int main() {
 sim::GridMap open_map(200, 100, 0.1);
 const std::vector<sim::DynState> forklift{{{5.0, 5.0}, {0.5, 0.0}, 0.5}};
 sim::Rng open_rng(7);
 const auto visible = sim::lidar_scan(
 open_map, {1.0, 5.0}, 0.0, 4, 10.0, 0.0, open_rng, forklift);
 CHECK(visible.hit[0]);
 CHECK(visible.dynamic_hit[0] == 0);
 CHECK(std::abs(visible.ranges[0] - 3.5) < 1e-9);

 sim::GridMap occluded_map(200, 100, 0.1);
 occluded_map.addBox(3.0, 4.5, 3.5, 5.5);
 sim::Rng occluded_rng(7);
 const auto occluded = sim::lidar_scan(
 occluded_map, {1.0, 5.0}, 0.0, 4, 10.0, 0.0, occluded_rng, forklift);
 CHECK(occluded.hit[0]);
 CHECK(occluded.dynamic_hit[0] == -1);
 CHECK(occluded.ranges[0] < visible.ranges[0]);

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

  const auto dwa = sim::dwa_compute(
  {1.0, 2.0}, 0.0, 0.0, 0.0, {8.0, 2.0}, open_map, {1.0, 2.0, 0.3},
  1.5, 1.5, 2.0, 1.5, 0.1);
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
  sim::DifferentialMotorDrive motors({
  0.52, 0.10, 2.4, 4.2, 54.0, 3.0, 24.0, 2.4, 2048, 1.0, 2.0,
  }, test_gains);
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
 return 0;
}
