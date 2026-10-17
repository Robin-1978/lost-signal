#include "autonomy/perception/dynamic_obstacle_map.hpp"
#include "autonomy/planning/local/predictor.hpp"
#include "hw/sensors/hal.hpp"
#include "hw/chassis/diff_drive.hpp"
#include "autonomy/planning/local/dwa.hpp"
#include "autonomy/mapping/grid_map.hpp"
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
 const std::vector<sim::DynamicObjectState> forklift{{{5.0, 5.0}, {0.5, 0.0}, 0.5}};
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
 const auto quantized = sim::apply_lidar_noise(ideal, sim::LidarNoiseModel{0.0, 0.12, 0.1, 0.0}, model_rng);
 CHECK(std::abs(quantized.ranges[0] - 3.6) < 1e-12);
 CHECK(quantized.header.sequence == ideal.header.sequence);
 const auto dropped = sim::apply_lidar_noise(ideal, sim::LidarNoiseModel{0.0, 0.0, 0.0, 1.0}, model_rng);
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
 // IMU is a hardware-layer device: physics supplies ideal six-axis truth;
 // this device owns frame/sequence, fixed bias, random walk and white noise.
 const sim::ImuTruth imu_truth{0.1, -0.2, 0.3, 1.2, -0.4, 9.7};
 const auto ideal_imu = sensors.capture_imu(imu_truth, 0.02, 0.02, {});
 CHECK(ideal_imu.roll_rate_radps == imu_truth.roll_rate_radps);
 CHECK(ideal_imu.pitch_rate_radps == imu_truth.pitch_rate_radps);
 CHECK(ideal_imu.yaw_rate_radps == imu_truth.yaw_rate_radps);
 CHECK(ideal_imu.forward_acceleration_mps2 == imu_truth.forward_specific_force_mps2);
 CHECK(ideal_imu.left_acceleration_mps2 == imu_truth.left_specific_force_mps2);
 CHECK(ideal_imu.up_acceleration_mps2 == imu_truth.up_specific_force_mps2);
 CHECK(ideal_imu.header.frame_id == "imu_link");
 CHECK(ideal_imu.header.sequence == 0);
 sim::SensorSystem noisy_imu_a(open_map, 37, 11);
 sim::SensorSystem noisy_imu_b(open_map, 37, 11);
 const sim::ImuConfig noisy_imu_config{0.01, 0.02, 0.003, 0.05, "imu_link", 0.004};
 const auto imu_a = noisy_imu_a.capture_imu(imu_truth, 1.0, 0.02, noisy_imu_config);
 const auto imu_b = noisy_imu_b.capture_imu(imu_truth, 1.0, 0.02, noisy_imu_config);
 CHECK(imu_a.roll_rate_radps == imu_b.roll_rate_radps);
 CHECK(imu_a.pitch_rate_radps == imu_b.pitch_rate_radps);
 CHECK(imu_a.yaw_rate_radps == imu_b.yaw_rate_radps);
 CHECK(imu_a.forward_acceleration_mps2 == imu_b.forward_acceleration_mps2);
 CHECK(imu_a.header.latency_s == 0.004);
 // Independent profile oracle: with identical RNG draws and zero white
 // noise, fixed bias scales exactly with the configured standard deviation.
 // This compares device outputs on the same truth trace instead of comparing
 // two closed-loop missions that follow different paths.
 sim::SensorSystem basic_imu(open_map, 73, 11);
 sim::SensorSystem precise_imu(open_map, 73, 11);
 const sim::ImuConfig basic_bias{0.0, 0.005, 0.0, 0.0, "imu_link", 0.0};
 const sim::ImuConfig precise_bias{0.0, 0.001, 0.0, 0.0, "imu_link", 0.0};
 const auto basic_sample = basic_imu.capture_imu(imu_truth, 1.0, 0.02, basic_bias);
 const auto precise_sample = precise_imu.capture_imu(imu_truth, 1.0, 0.02, precise_bias);
 CHECK(std::abs((basic_sample.roll_rate_radps - imu_truth.roll_rate_radps) -
                5.0 * (precise_sample.roll_rate_radps - imu_truth.roll_rate_radps)) < 1e-15);
 CHECK(std::abs((basic_sample.pitch_rate_radps - imu_truth.pitch_rate_radps) -
                5.0 * (precise_sample.pitch_rate_radps - imu_truth.pitch_rate_radps)) < 1e-15);
 CHECK(std::abs((basic_sample.yaw_rate_radps - imu_truth.yaw_rate_radps) -
                5.0 * (precise_sample.yaw_rate_radps - imu_truth.yaw_rate_radps)) < 1e-15);
 sim::WorldTruth truth;
 truth.robot = {1.0, 5.0, 0.0, 0.0, 0.0};
 truth.dynamic_objects = forklift;
 sim::EstimatedState estimate;
 estimate.position = {1.0, 5.0};
 estimate.heading_rad = 0.0;
 sim::LidarConfig lidar_config;
 lidar_config.rays = 360;
 lidar_config.min_range_m = 0.2;
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
 sim::SensorSystem near_field_sensors(open_map, 7, 11);
 sim::WorldTruth near_field_truth = truth;
 near_field_truth.dynamic_objects = {{{1.3, 5.0}, {}, 0.1}};
 lidar_config.min_range_m = 0.25;
 const auto near_field = near_field_sensors.capture_lidar(near_field_truth, 0.0, lidar_config);
 CHECK(!near_field.lidar->hit[0]);
 CHECK(near_field.lidar->ranges[0] == near_field.lidar->max_range);
 CHECK(near_field.lidar->intensities[0] == 0.0);
 lidar_config.min_range_m = 0.2;
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

 // A learned shuttle corridor remains occupied during endpoint dwell. The
 // public track correctly reports near-zero current velocity, while the
 // forecast covers the plausible reflected return instead of declaring the
 // whole crossing clear.
 sim::DynamicObstacleMap shuttle;
 shuttle.observe({7, {0.0, 0.0}, 0.4}, 0.0);
 shuttle.observe({7, {1.0, 0.0}, 0.4}, 1.0);
 shuttle.observe({7, {1.0, 0.0}, 0.4}, 1.21);
 shuttle.observe({7, {0.79, 0.0}, 0.4}, 1.42); // reversal learns x bounds
 shuttle.observe({7, {0.0, 0.0}, 0.4}, 2.21);
 shuttle.observe({7, {0.0, 0.0}, 0.4}, 2.42);
 shuttle.observe({7, {0.0, 0.0}, 0.4}, 2.63);
 shuttle.observe({7, {0.0, 0.0}, 0.4}, 2.84);
 const auto stopped_track = shuttle.active_tracks(2.84, 1.0);
 CHECK(stopped_track.size() == 1);
 CHECK(std::abs(stopped_track.front().state.v.x) < 0.10);
 const auto dwell_forecast = shuttle.predict(2.84, 1.0, 0.1, 1.0);
 CHECK(dwell_forecast.size() == 11);
 CHECK(dwell_forecast.front().front().p.x < 1e-9);
 CHECK(dwell_forecast[5].front().p.x > 0.3);

 // Even without a learned reversal boundary (for example after track
 // fragmentation), endpoint dwell keeps continuation and reversal modes.
 sim::DynamicObstacleMap fragmented;
 fragmented.observe({9, {0.0, 0.0}, 0.4}, 0.0);
 fragmented.observe({9, {1.0, 0.0}, 0.4}, 1.0);
 fragmented.observe({9, {1.0, 0.0}, 0.4}, 1.21);
 fragmented.observe({9, {1.0, 0.0}, 0.4}, 1.42);
 fragmented.observe({9, {1.0, 0.0}, 0.4}, 1.63);
 const auto ambiguous = fragmented.predict(1.63, 1.0, 0.1, 1.0);
 CHECK(ambiguous.front().size() == 2);
 CHECK(ambiguous.back()[0].p.x > 1.5);
 CHECK(ambiguous.back()[1].p.x < 0.5);
 const auto nominal = fragmented.predict_nominal(1.63, 1.0, 0.1, 1.0);
 CHECK(nominal.front().size() == 1);
 CHECK(nominal.back().front().p.x > 1.5);

 const auto dwa = sim::dwa_compute({1.0, 2.0}, 0.0, 0.0, 0.0, {8.0, 2.0}, open_map, {1.0, 2.0, 0.3}, 1.5, 1.5,
                                   2.0, 1.5, 0.1, 0.38, 8.0);
 CHECK(dwa.best.valid);
 CHECK(!dwa.best_path.empty());
 CHECK(dwa.best_path.front().x == 1.0);
 CHECK(dwa.best_path.front().y == 2.0);
 CHECK(dwa.best_path.size() > 2);
 CHECK(!dwa.rollouts.empty());
 CHECK(dwa.rollouts.front().points.size() > 2);

 const auto cliff_aware =
   sim::dwa_compute({1.0, 2.0}, 0.0, 0.0, 0.0, {8.0, 2.0}, open_map, {1.0, 2.0, 0.3}, 1.5, 1.5, 2.0, 1.5, 0.1,
                    0.38, 8.0, [](sim::Vec2, sim::Vec2 to) { return to.x < 1.2; });
 CHECK(cliff_aware.best.valid);
 for (const auto point : cliff_aware.best_path)
  CHECK(point.x < 1.2);
 CHECK(std::any_of(cliff_aware.rollouts.begin(), cliff_aware.rollouts.end(),
                   [](const auto& rollout) { return rollout.collision; }));

 // Behavior can forbid reversing after it commits to a mapped crossing;
 // DWA still samples and collision-checks the remaining forward window.
 const auto committed =
   sim::dwa_compute_trajectory({1.0, 2.0}, 0.0, 0.0, 0.0, {8.0, 2.0}, open_map, {1.0, 2.0, 0.3}, {}, 1.5, 1.5,
                               2.0, 1.5, 0.1, 0.1, 0.0, 0.38, 8.0, {}, 0.0);
 CHECK(committed.best.valid);
 CHECK(committed.best.v >= 0.0);
 CHECK(std::all_of(committed.rollouts.begin(), committed.rollouts.end(),
                   [](const auto& rollout) { return rollout.command.v >= 0.0; }));

 // A soft-margin incursion admits an escape rollout only when separation
 // grows; stopping inside the margin must not be the planner's sole answer.
 const std::vector<std::vector<sim::DynamicObjectState>> close_behind(16, {{{0.1, 2.0}, {0.0, 0.0}, 0.4}});
 const auto escaping =
   sim::dwa_compute_trajectory({1.0, 2.0}, 0.0, 0.0, 0.0, {8.0, 2.0}, open_map, {1.0, 2.0, 0.3}, close_behind,
                               1.5, 1.5, 2.0, 1.5, 0.1, 0.1, 0.5, 0.38, 8.0, {}, 0.0);
 CHECK(escaping.best.valid);
 CHECK(escaping.best.v > 0.0);
 const auto low_speed_crossing =
   sim::dwa_compute_trajectory({1.0, 2.0}, 0.0, 0.0, 0.0, {8.0, 2.0}, open_map, {1.0, 2.0, 0.3}, {}, 0.20,
                               1.5, 2.0, 1.5, 0.1, 0.1, 0.5, 0.38, 8.0, {}, 0.0);
 CHECK(low_speed_crossing.best.valid);
 CHECK(std::abs(low_speed_crossing.best.v - 0.20) < 1e-12);

 double left = 0.0;
 double right = 0.0;
 sim::DiffDrive::twistToWheels(1.0, 1.0, 0.52, left, right);
 CHECK(std::abs(left - 0.74) < 1e-12);
 CHECK(std::abs(right - 1.26) < 1e-12);

 const sim::WheelPidGains test_gains = sim::tune_wheel_pid(12.0, 54.0, 3.0, 1.2);
 CHECK(test_gains.kp > 1.0);
 CHECK(test_gains.kp < 4.0);
 sim::MotorDriveParams test_motor_params;
 test_motor_params.wheel_base_m = 0.52;
 test_motor_params.wheel_radius_m = 0.10;
 test_motor_params.max_wheel_speed_mps = 2.4;
 test_motor_params.max_wheel_accel_mps2 = 4.2;
 test_motor_params.stall_force_N = 54.0;
 test_motor_params.free_speed_mps = 3.0;
 test_motor_params.mass_kg = 24.0;
 test_motor_params.yaw_inertia_kgm2 = 2.4;
 test_motor_params.encoder_counts_per_revolution = 2048;
 test_motor_params.pwm_limit = 1.0;
 test_motor_params.integral_limit = 2.0;
 test_motor_params.support_width_m = 0.5;
 test_motor_params.support_length_m = 0.5;
 test_motor_params.max_yaw_rate_radps = 2.0;
 sim::DifferentialMotorDrive motors(test_motor_params, test_gains);
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
  const std::vector<std::vector<sim::DynamicObjectState>> closing = {
    {{{5.0, 0.0}, {0.0, 0.0}, 0.5}},
    {{{4.0, 0.0}, {0.0, 0.0}, 0.5}},
    {{{3.0, 0.0}, {0.0, 0.0}, 0.5}},
  };
  // Ego at frame k stands at k m (frame 0 = now at x=0); swept intervals
  // [0,1]x[5,4] and [1,2]x[4,3] close to 1.0 m center distance at the end.
  const double gap = sim::predicted_gap(path, 0, 10.0, closing, 0.1, 0.38);
  CHECK(std::abs(gap - (1.0 - 0.5 - 0.38)) < 1e-9);
  const std::vector<std::vector<sim::DynamicObjectState>> leaving = {
    {{{5.0, 5.0}, {0.0, 0.0}, 0.5}},
  };
  CHECK(sim::predicted_gap(path, 0, 10.0, leaving, 0.1, 0.38) > 4.0);
  CHECK(sim::predicted_gap({}, 0, 1.0, closing, 0.1, 0.38) == std::numeric_limits<double>::infinity());
  CHECK(sim::predicted_gap(path, 0, 1.0, {}, 0.1, 0.38) == std::numeric_limits<double>::infinity());
  // Standing ego, threat driving at it: still caught.
  const std::vector<std::vector<sim::DynamicObjectState>> incoming = {
    {{{2.0, 0.0}, {0.0, 0.0}, 0.5}},
  };
  CHECK(sim::predicted_gap(path, 0, 0.0, incoming, 0.1, 0.38) < 1.2);
 }
 // Swept intervals: a fast crosser passing between frames is caught exactly
 // (endpoints both read ~1.0 m, true minimum 0.1 m). Discrete sampling alone
 // would report clear; the segment check must not.
 {
  const std::vector<sim::Vec2> line{{0.0, 0.0}, {10.0, 0.0}};
  const std::vector<std::vector<sim::DynamicObjectState>> crossing = {
    {{{-1.0, 0.1}, {0.0, 0.0}, 0.5}},
    {{{1.0, 0.1}, {0.0, 0.0}, 0.5}},
  };
  const double swept = sim::predicted_gap(line, 0, 0.0, crossing, 0.1, 0.38);
  CHECK(std::abs(swept - (0.1 - 0.5 - 0.38)) < 1e-9);
 }
 // Departure reasons: infinity is three different verdicts with three owners.
 {
  const std::vector<sim::Vec2> line{{0.0, 0.0}, {2.0, 0.0}};
  const std::vector<std::vector<sim::DynamicObjectState>> open(40, {{{5.0, 5.0}, {0.0, 0.0}, 0.4}});
  CHECK(sim::assess_departure(line, 0, 1.0, open, 0.1, 0.38, 0.5).reason == sim::DepartReason::kGoNow);
  std::vector<std::vector<sim::DynamicObjectState>> clearing;
  for (int k = 0; k < 40; ++k) {
   clearing.push_back(k < 10 ? std::vector<sim::DynamicObjectState>{{{1.5, 0.0}, {0.0, 0.0}, 0.4}}
                             : std::vector<sim::DynamicObjectState>{{{5.0, 5.0}, {0.0, 0.0}, 0.4}});
  }
  const auto wait = sim::assess_departure(line, 0, 1.0, clearing, 0.1, 0.38, 0.5);
  CHECK(wait.reason == sim::DepartReason::kWait && wait.wait_s > 0.0);
  // Parked threat, safe waiting spot: genuinely blocked inside the window.
  const std::vector<std::vector<sim::DynamicObjectState>> parked(40, {{{1.5, 0.0}, {0.0, 0.0}, 0.4}});
  CHECK(sim::assess_departure(line, 0, 1.0, parked, 0.1, 0.38, 0.5).reason ==
        sim::DepartReason::kBlockedInWindow);
  // Threat drives through the waiting spot: staying is what kills (Phase-2).
  std::vector<std::vector<sim::DynamicObjectState>> runover;
  for (int k = 0; k < 51; ++k) {
   runover.push_back({{{4.0 - 2.0 * k * 0.1, 0.0}, {0.0, 0.0}, 0.4}});
  }
  const std::vector<sim::Vec2> corridor{{0.0, 0.0}, {6.0, 0.0}};
  CHECK(sim::assess_departure(corridor, 0, 1.0, runover, 0.1, 0.38, 0.5).reason ==
        sim::DepartReason::kWaitUnsafe);
  // Open field, traversal longer than the window: unknown, not unsafe.
  const std::vector<sim::Vec2> long_line{{0.0, 0.0}, {10.0, 0.0}};
  const std::vector<std::vector<sim::DynamicObjectState>> short_open(21, {{{50.0, 50.0}, {0.0, 0.0}, 0.4}});
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
  const std::vector<std::vector<sim::DynamicObjectState>> open(40, {{{5.0, 5.0}, {0.0, 0.0}, 0.4}});
  CHECK(sim::earliest_departure(line, 0, 1.0, open, 0.1, 0.38, 0.5) == 0.0);
  // Threat parked on the path for the whole horizon: no departure works.
  // Parked at 1.5 m so the wait position itself (gap 0.72) passes margin —
  // the infinity comes from the traversal, not from a bad wait spot.
  const std::vector<std::vector<sim::DynamicObjectState>> blocked(40, {{{1.5, 0.0}, {0.0, 0.0}, 0.4}});
  CHECK(sim::earliest_departure(line, 0, 1.0, blocked, 0.1, 0.38, 0.5) ==
        std::numeric_limits<double>::infinity());
  // Threat clears after 1 s: must wait, then go.
  std::vector<std::vector<sim::DynamicObjectState>> clearing;
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
  const std::vector<std::vector<sim::DynamicObjectState>> closing = {
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
  std::vector<std::vector<sim::DynamicObjectState>> visiting;
  for (int k = 0; k < 21; ++k) {
   visiting.push_back(k < 10 ? std::vector<sim::DynamicObjectState>{{{1.5, 0.0}, {0.0, 0.0}, 0.4}}
                             : std::vector<sim::DynamicObjectState>{{{5.0, 5.0}, {0.0, 0.0}, 0.4}});
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
