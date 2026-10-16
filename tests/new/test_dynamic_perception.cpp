#include "algo/dynamic_obstacle_map.hpp"
#include "algo/predictor.hpp"
#include "hw/sensors/hal.hpp"
#include "hw/chassis/diff_drive.hpp"
#include "algo/dwa.hpp"
#include "algo/grid_map.hpp"
#include "hw/chassis/motor_drive.hpp"
#include "hw/sensors/sensor.hpp"
#include "hw/sensors/sensor_system.hpp"

#include <algorithm>
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
 // The engine and transforms are pinned together: these values must stay
 // stable when a seeded experiment moves between standard libraries.
 sim::Rng portable_uniform(12345);
 CHECK(std::abs(portable_uniform.uniform01() - 0.89015471328273787) < 1e-15);
 CHECK(std::abs(portable_uniform.uniform01() - 0.13070729405534817) < 1e-15);
 sim::Rng portable_gaussian(12345);
 CHECK(std::abs(portable_gaussian.gauss() - 0.59509116615656177) < 1e-12);
 CHECK(std::abs(portable_gaussian.gauss(0.25) - 0.20011167700651958) < 1e-12);

 sim::GridMap open_map(200, 100, 0.1);
 const std::vector<sim::DynState> forklift{{{5.0, 5.0}, {0.5, 0.0}, 0.5}};
 sim::Rng open_rng(7);
 const auto visible = sim::lidar_scan(open_map, {1.0, 5.0}, 0.0, 4, 10.0, 0.0, open_rng, forklift);
 CHECK(visible.hit[0]);
 CHECK(std::abs(visible.ranges[0] - 3.5) < 1e-9);
 auto ideal = visible;
 ideal.header = {1.25, 9, "lidar_link", 0.02, 1, "test.lidar"};
 sim::Rng noise_a(99);
 sim::Rng noise_b(99);
 const auto noisy_a = sim::apply_lidar_noise(ideal, 0.05, noise_a);
 const auto noisy_b = sim::apply_lidar_noise(ideal, 0.05, noise_b);
 CHECK(noisy_a.ranges == noisy_b.ranges);
 CHECK(noisy_a.ranges != ideal.ranges);
 CHECK(noisy_a.header.stamp_s == ideal.header.stamp_s);
 CHECK(noisy_a.header.frame_id == ideal.header.frame_id);
 sim::Rng model_rng(3);
 const auto quantized = sim::apply_lidar_noise(
 ideal, sim::LidarNoiseModel{0.0, 0.12, 0.1, 0.0}, model_rng);
 CHECK(std::abs(quantized.ranges[0] - 3.6) < 1e-12);
 CHECK(quantized.header.sequence == ideal.header.sequence);
 const auto dropped = sim::apply_lidar_noise(
 ideal, sim::LidarNoiseModel{0.0, 0.0, 0.0, 1.0}, model_rng);
 CHECK(!dropped.hit[0]);
 CHECK(dropped.ranges[0] == dropped.max_range);
 CHECK(dropped.intensities[0] == 0.0);

 sim::GridMap occluded_map(200, 100, 0.1);
 occluded_map.addBox(3.0, 4.5, 3.5, 5.5);
 sim::Rng occluded_rng(7);
 const auto occluded = sim::lidar_scan(occluded_map, {1.0, 5.0}, 0.0, 4, 10.0, 0.0, occluded_rng, forklift);
 CHECK(occluded.hit[0]);
 CHECK(occluded.ranges[0] < visible.ranges[0]);

 // HAL contract: SimLidar reproduces the direct raycast bit-for-bit.
 sim::SimLidar hal_lidar(open_map);
 sim::Rng open_rng2(7);
 const auto direct = sim::lidar_scan(open_map, {1.0, 5.0}, 0.0, 4, 10.0, 0.0, open_rng2, forklift);
 const auto via_hal = hal_lidar.scan({1.0, 5.0}, 0.0, 4, 10.0, forklift);
 CHECK(via_hal.ranges.size() == direct.ranges.size());
 for (std::size_t i = 0; i < via_hal.ranges.size(); ++i) {
  CHECK(via_hal.ranges[i] == direct.ranges[i]);
  CHECK(via_hal.hit[i] == direct.hit[i]);
 }

 // Production perception receives anonymous ranges. Static-map differencing,
 // clustering and temporal confirmation create the track without truth IDs.
 sim::SensorSystem sensors(open_map, 7, 11);
 sim::WorldTruth truth;
 truth.robot = {1.0, 5.0, 0.0, 0.0, 0.0};
 truth.dynamic_objects = forklift;
 sim::EstimatedState estimate;
 estimate.position = {1.0, 5.0};
 estimate.heading_rad = 0.0;
 sim::LidarConfig lidar_config;
 lidar_config.rays = 360;
 lidar_config.range_m = 10.0;
 lidar_config.scan_hz = 10.0;
 const auto first_frame = sensors.capture_lidar(truth, 0.0, lidar_config);
 CHECK(first_frame.lidar->header.stamp_s == 0.0);
 CHECK(first_frame.lidar->header.sequence == 0);
 CHECK(first_frame.lidar->header.frame_id == "lidar_link");
 CHECK(first_frame.lidar->scan_period_s == 0.1);
CHECK(first_frame.lidar->time_increment_s == 0.0); // ideal Domain comparison is instantaneous
CHECK(first_frame.lidar->intensities.size() == first_frame.lidar->ranges.size());
CHECK(first_frame.lidar->intensities[0] > 0.0);
 const auto first_perception = sensors.process_lidar(first_frame, estimate, 1.0);
 CHECK(!first_perception.dynamic_object_detected);
 const auto second_frame = sensors.capture_lidar(truth, 0.1, lidar_config);
 CHECK(second_frame.lidar->header.stamp_s == 0.1);
 CHECK(second_frame.lidar->header.sequence == 1);
 const auto second_perception = sensors.process_lidar(second_frame, estimate, 1.0);
 CHECK(second_perception.dynamic_object_detected);
 const auto anonymous_tracks = sensors.dynamic_obstacles().active_tracks(0.1, 1.0);
 CHECK(anonymous_tracks.size() == 1);
 CHECK(std::abs(anonymous_tracks.front().state.p.x - 5.0) < 0.25);

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

 const auto cliff_aware = sim::dwa_compute(
 {1.0, 2.0}, 0.0, 0.0, 0.0, {8.0, 2.0}, open_map, {1.0, 2.0, 0.3}, 1.5, 1.5,
 2.0, 1.5, 0.1, 0.38, 8.0,
 [](sim::Vec2, sim::Vec2 to) { return to.x < 1.2; });
 CHECK(cliff_aware.best.valid);
 for (const auto point : cliff_aware.best_path)
 CHECK(point.x < 1.2);
 CHECK(std::any_of(cliff_aware.rollouts.begin(), cliff_aware.rollouts.end(),
 [](const auto& rollout) { return rollout.collision; }));

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
 0.5,
 0.5,
 2.0,
 {{0.25, 0.0}, {-0.25, 0.0}},
   },
   test_gains);
 const sim::GroundContact test_ground{0.7, 0.01, 4.2};
 sim::DiffState driven;
 driven = motors.step(driven, {1.0, 0.6}, 0.02, test_ground);
 CHECK(motors.telemetry().left.pwm > 0.0);
 CHECK(motors.telemetry().right.target_mps > motors.telemetry().left.target_mps);
 CHECK(motors.telemetry().passive_caster_steer_count == 2);
 CHECK(std::abs(motors.telemetry().left.pwm) <= 1.0);
 CHECK(std::abs(motors.telemetry().right.pwm) <= 1.0);
 for (int step = 0; step < 100; ++step) {
  driven = motors.step(driven, {1.0, 0.6}, 0.02, test_ground);
 }
 CHECK(driven.v > 0.7);
 CHECK(driven.w > 0.2);
 // Both wheel PIDs saturate equally on the first acceleration frame, so the
 // caster direction only becomes observable after yaw response develops.
 CHECK(motors.telemetry().passive_caster_steer_rad[0] > 0.0);
 CHECK(motors.telemetry().passive_caster_steer_rad[1] < 0.0);
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
  CHECK(sim::predicted_gap({}, 0, 1.0, closing, 0.1, 0.38) == std::numeric_limits<double>::infinity());
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
 // Departure reasons: infinity is three different verdicts with three owners.
 {
  const std::vector<sim::Vec2> line{{0.0, 0.0}, {2.0, 0.0}};
  const std::vector<std::vector<sim::DynState>> open(40, {{{5.0, 5.0}, {0.0, 0.0}, 0.4}});
  CHECK(sim::assess_departure(line, 0, 1.0, open, 0.1, 0.38, 0.5).reason == sim::DepartReason::kGoNow);
  std::vector<std::vector<sim::DynState>> clearing;
  for (int k = 0; k < 40; ++k) {
   clearing.push_back(k < 10 ? std::vector<sim::DynState>{{{1.5, 0.0}, {0.0, 0.0}, 0.4}}
                             : std::vector<sim::DynState>{{{5.0, 5.0}, {0.0, 0.0}, 0.4}});
  }
  const auto wait = sim::assess_departure(line, 0, 1.0, clearing, 0.1, 0.38, 0.5);
  CHECK(wait.reason == sim::DepartReason::kWait && wait.wait_s > 0.0);
  // Parked threat, safe waiting spot: genuinely blocked inside the window.
  const std::vector<std::vector<sim::DynState>> parked(40, {{{1.5, 0.0}, {0.0, 0.0}, 0.4}});
  CHECK(sim::assess_departure(line, 0, 1.0, parked, 0.1, 0.38, 0.5).reason ==
        sim::DepartReason::kBlockedInWindow);
  // Threat drives through the waiting spot: staying is what kills (Phase-2).
  std::vector<std::vector<sim::DynState>> runover;
  for (int k = 0; k < 51; ++k) {
   runover.push_back({{{4.0 - 2.0 * k * 0.1, 0.0}, {0.0, 0.0}, 0.4}});
  }
  const std::vector<sim::Vec2> corridor{{0.0, 0.0}, {6.0, 0.0}};
  CHECK(sim::assess_departure(corridor, 0, 1.0, runover, 0.1, 0.38, 0.5).reason ==
        sim::DepartReason::kWaitUnsafe);
  // Open field, traversal longer than the window: unknown, not unsafe.
  const std::vector<sim::Vec2> long_line{{0.0, 0.0}, {10.0, 0.0}};
  const std::vector<std::vector<sim::DynState>> short_open(21, {{{50.0, 50.0}, {0.0, 0.0}, 0.4}});
  CHECK(sim::assess_departure(long_line, 0, 1.0, short_open, 0.1, 0.38, 0.5).reason ==
        sim::DepartReason::kHorizonShort);
  CHECK(sim::assess_departure({}, 0, 1.0, open, 0.1, 0.38, 0.5).reason == sim::DepartReason::kInvalid);
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
  CHECK(sim::earliest_departure({}, 0, 1.0, open, 0.1, 0.38, 0.5) == std::numeric_limits<double>::infinity());
  CHECK(sim::earliest_departure(line, 0, 0.0, open, 0.1, 0.38, 0.5) ==
        std::numeric_limits<double>::infinity());
 }
 // timed_gap: constant-speed timed input reproduces predicted_gap exactly;
 // a hold-then-go pattern (pocket miniature) clears what an immediate
 // departure collides with; empty ego reports infinity.
 {
  const std::vector<sim::Vec2> path{{0.0, 0.0}, {10.0, 0.0}};
  const std::vector<std::vector<sim::DynState>> closing = {
    {{{5.0, 0.0}, {0.0, 0.0}, 0.5}},
    {{{4.0, 0.0}, {0.0, 0.0}, 0.5}},
    {{{3.0, 0.0}, {0.0, 0.0}, 0.5}},
  };
  std::vector<sim::TimedPose> constant;
  for (int k = 0; k < 3; ++k) {
   constant.push_back({k * 0.1, {10.0 * k * 0.1, 0.0}});
  }
  CHECK(std::abs(sim::timed_gap(constant, closing, 0.1, 0.38) -
                 sim::predicted_gap(path, 0, 10.0, closing, 0.1, 0.38)) < 1e-12);
  // Threat sits at x=1.5 for 1 s, then leaves; ego holds at x=0 for 1 s
  // (safe: gap 0.72), then advances. Holding lets the threat clear;
  // rushing meets it. Pocket pattern in miniature.
  std::vector<std::vector<sim::DynState>> visiting;
  for (int k = 0; k < 21; ++k) {
   visiting.push_back(k < 10 ? std::vector<sim::DynState>{{{1.5, 0.0}, {0.0, 0.0}, 0.4}}
                             : std::vector<sim::DynState>{{{5.0, 5.0}, {0.0, 0.0}, 0.4}});
  }
  const std::vector<sim::TimedPose> hold_go{{{0.0, {0.0, 0.0}}, {1.0, {0.0, 0.0}}, {2.0, {1.0, 0.0}}}};
  const double held = sim::timed_gap(hold_go, visiting, 0.1, 0.38);
  CHECK(held >= 0.5);
  const std::vector<sim::TimedPose> rush{{{0.0, {0.0, 0.0}}, {2.0, {2.0, 0.0}}}};
  CHECK(sim::timed_gap(rush, visiting, 0.1, 0.38) < held);
  CHECK(sim::timed_gap({}, visiting, 0.1, 0.38) == std::numeric_limits<double>::infinity());
 }
 return 0;
}
