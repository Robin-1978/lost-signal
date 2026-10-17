#include "robot/autonomy/perception/dynamic_obstacle_map.hpp"
#include "robot/autonomy/perception/lidar_perception_system.hpp"
#include "robot/autonomy/planning/local/predictor.hpp"
#include "sim/hardware/physics/lidar_raycast.hpp"
#include "robot/platform/differential_kinematics.hpp"
#include "robot/autonomy/planning/local/dwa.hpp"
#include "robot/autonomy/mapping/grid_map.hpp"
#include "sim/hardware/vehicle/motor_drive.hpp"
#include "sim/hardware/sensors/sensor.hpp"
#include "sim/hardware/sensors/sensor_system.hpp"

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
 const auto visible = sim::planar_lidar_raycast(open_map, {1.0, 5.0}, 0.0, 4, 10.0, forklift);
 CHECK(visible.hit[0]);
 CHECK(std::abs(visible.ranges[0] - 3.5) < 1e-9);
 auto ideal = visible;
 ideal.header = {1.25, 9, "lidar_link", 0.02, sim::kMeasurementSchemaVersion, "test.lidar"};
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
 const auto occluded = sim::planar_lidar_raycast(
 occluded_map, {1.0, 5.0}, 0.0, 4, 10.0, forklift);
 CHECK(occluded.hit[0]);
 CHECK(occluded.ranges[0] < visible.ranges[0]);

 const auto direct = sim::planar_lidar_truth(
 open_map, {1.0, 5.0}, 0.0, 4, 10.0, forklift);
 CHECK(direct.combined.ranges == visible.ranges);
 CHECK(direct.static_environment.ranges.size() == visible.ranges.size());

 // Production perception receives anonymous ranges. Static-map differencing,
 // clustering and temporal confirmation create the track without truth IDs.
 sim::SensorSystem sensors(7, 11);
 sim::LidarPerceptionSystem perception(open_map, 11);
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
 sim::SensorSystem noisy_imu_a(37, 11);
 sim::SensorSystem noisy_imu_b(37, 11);
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
 sim::SensorSystem basic_imu(73, 11);
 sim::SensorSystem precise_imu(73, 11);
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
 const auto first_frame = sensors.capture_lidar(
 truth, 0.0, lidar_config,
 sim::planar_lidar_truth(open_map, {1.0, 5.0}, 0.0, 360, 10.0, truth.dynamic_objects));
 CHECK(first_frame.lidar->header.stamp_s == 0.0);
 CHECK(first_frame.lidar->header.sequence == 0);
 CHECK(first_frame.lidar->header.frame_id == "lidar_link");
 CHECK(first_frame.lidar->scan_period_s == 0.1);
 CHECK(first_frame.lidar->time_increment_s == 0.0); // ideal Domain comparison is instantaneous
 CHECK(first_frame.lidar->intensities.size() == first_frame.lidar->ranges.size());
 CHECK(first_frame.lidar->intensities[0] > 0.0);
 sim::SensorSystem near_field_sensors(7, 11);
 sim::WorldTruth near_field_truth = truth;
 near_field_truth.dynamic_objects = {{{1.3, 5.0}, {}, 0.1}};
 lidar_config.min_range_m = 0.25;
 const auto near_field = near_field_sensors.capture_lidar(
 near_field_truth, 0.0, lidar_config,
 sim::planar_lidar_truth(open_map, {1.0, 5.0}, 0.0, 360, 10.0,
 near_field_truth.dynamic_objects));
 CHECK(!near_field.lidar->hit[0]);
 CHECK(near_field.lidar->ranges[0] == near_field.lidar->max_range);
 CHECK(near_field.lidar->intensities[0] == 0.0);
 lidar_config.min_range_m = 0.2;
 const auto projection_state = sim::LidarProjectionState{
 estimate.position, estimate.heading_rad, estimate.linear_velocity_mps,
 estimate.lateral_velocity_mps, estimate.angular_velocity_radps};
 const auto projection_config = sim::LidarProjectionConfig{};
 const auto first_perception = perception.process(
 *first_frame.lidar, *first_frame.lidar_static_reference,
 projection_state, projection_config, 1.0);
 CHECK(!first_perception.dynamic_object_detected);
 CHECK(std::any_of(first_perception.obstacle_rays.begin(),
 first_perception.obstacle_rays.end(), [](const auto& ray) { return ray.hit; }));
 bool motion_confirmed = false;
 double last_frame_time = 0.0;
 for (int sample = 1; sample <= 20 && !motion_confirmed; ++sample) {
 last_frame_time = sample * 0.1;
 truth.dynamic_objects.front().p.x = 5.0 + 0.55 * last_frame_time;
 const auto moving_frame = sensors.capture_lidar(
 truth, last_frame_time, lidar_config,
 sim::planar_lidar_truth(open_map, {1.0, 5.0}, 0.0, 360, 10.0,
 truth.dynamic_objects));
 CHECK(moving_frame.lidar->header.stamp_s == last_frame_time);
 CHECK(moving_frame.lidar->header.sequence == static_cast<std::uint64_t>(sample));
 const auto moving_perception = perception.process(
 *moving_frame.lidar, *moving_frame.lidar_static_reference,
 projection_state, projection_config, 1.0);
 motion_confirmed = moving_perception.dynamic_object_detected;
 }
 CHECK(motion_confirmed);
 const auto anonymous_tracks = perception.dynamic_obstacles().active_tracks(last_frame_time, 1.0);
 CHECK(anonymous_tracks.size() == 1);
CHECK(anonymous_tracks.front().motion_class != sim::TrackMotionClass::Unconfirmed);
CHECK(std::abs(anonymous_tracks.front().state.p.x - truth.dynamic_objects.front().p.x) < 0.35);

 // A map-alignment residual at a wall edge belongs to occupancy mapping,
 // not dynamic-object tracking. Repeated frames must not manufacture a
 // traffic participant from authored static geometry.
 sim::GridMap wall_map(100, 100, 0.1);
 wall_map.addBox(5.0, 4.0, 6.0, 6.0);
 sim::SensorSystem wall_sensors(17, 19);
 sim::LidarPerceptionSystem wall_perception_system(wall_map, 19);
 sim::WorldTruth wall_truth;
 wall_truth.robot = {1.0, 5.0, 0.0, 0.0, 0.0};
 sim::EstimatedState wall_estimate;
 wall_estimate.position = {1.0, 5.0};
 wall_estimate.heading_rad = 0.0;
 for (int frame_index = 0; frame_index < 3; ++frame_index) {
 auto wall_frame = wall_sensors.capture_lidar(
 wall_truth, 0.1 * frame_index, lidar_config,
 sim::planar_lidar_truth(wall_map, {1.0, 5.0}, 0.0, 360, 10.0));
 // Move a small angular group toward the sensor far enough to pass the
 // foreground residual threshold; its inferred centre still lies in wall.
 for (int ray : {0, 1, 359}) {
 wall_frame.lidar->hit[ray] = 1;
 wall_frame.lidar->ranges[ray] = 3.60;
 }
 const auto wall_perception = wall_perception_system.process(
 *wall_frame.lidar, *wall_frame.lidar_static_reference,
 {wall_estimate.position, wall_estimate.heading_rad,
 wall_estimate.linear_velocity_mps, wall_estimate.angular_velocity_radps},
 {}, 1.0);
 CHECK(!wall_perception.dynamic_object_detected);
 if (frame_index == 0) {
 CHECK(std::none_of(wall_perception.obstacle_rays.begin(),
 wall_perception.obstacle_rays.end(), [](const auto& ray) { return ray.hit; }));
 }
 }
 CHECK(wall_perception_system.dynamic_obstacles().active_tracks(0.2, 1.0).empty());

 // A cluster centroid can alternate between opposite visible edges of a
 // stationary object. The tracker must not publish this as motion. This is
 // an independent bounded-output oracle; it does not duplicate filter math.
 sim::DynamicObstacleMap stationary_jitter;
 stationary_jitter.observe({0, {5.0, 5.0}, 0.5}, 0.0);
 for (int sample = 1; sample <= 40; ++sample) {
 const double edge_jitter = sample % 2 == 0 ? -0.15 : 0.15;
 stationary_jitter.observe({0, {5.0 + edge_jitter, 5.0}, 0.5}, sample * 0.1);
 }
 const auto stationary = stationary_jitter.active_tracks(4.0, 1.0);
 CHECK(stationary.size() == 1);
 CHECK(std::abs(stationary.front().state.v.x) < 0.20);
 CHECK(std::abs(stationary.front().state.v.y) < 1e-12);
 CHECK(std::abs(stationary.front().state.p.x - 5.0) < 0.10);

 // A constant 0.55 m/s trace converges without using simulation truth IDs
 // or a two-sample finite difference as the acceptance criterion.
 sim::DynamicObstacleMap constant_velocity;
 constant_velocity.observe({1, {0.0, 0.0}, 0.5}, 0.0);
 for (int sample = 1; sample <= 20; ++sample)
 constant_velocity.observe({1, {0.55 * sample * 0.1, 0.0}, 0.5}, sample * 0.1);
 const auto moving = constant_velocity.active_tracks(2.0, 1.0);
 CHECK(moving.size() == 1);
 CHECK(std::abs(moving.front().state.v.x - 0.55) < 0.08);
 CHECK(std::abs(moving.front().state.p.x - 1.10) < 0.08);
 // A stale sample is ignored according to sensor time; arrival order cannot
 // reverse an otherwise valid track.
 constant_velocity.observe({1, {-20.0, 0.0}, 0.5}, 1.9);
 const auto after_stale = constant_velocity.active_tracks(2.0, 1.0);
 CHECK(after_stale.front().state.p.x == moving.front().state.p.x);
 CHECK(after_stale.front().state.v.x == moving.front().state.v.x);

 // A learned shuttle corridor remains occupied during endpoint dwell. The
 // public track correctly reports near-zero current velocity, while the
 // forecast covers the plausible reflected return instead of declaring the
 // whole crossing clear.
 sim::DynamicObstacleMap shuttle;
 shuttle.observe({7, {0.0, 0.0}, 0.4}, 0.0);
 for (int sample = 1; sample <= 20; ++sample)
 shuttle.observe({7, {0.05 * sample, 0.0}, 0.4}, sample * 0.1);
 for (int sample = 21; sample <= 30; ++sample)
 shuttle.observe({7, {1.0, 0.0}, 0.4}, sample * 0.1);
 const auto stopped_track = shuttle.active_tracks(3.0, 1.0);
 CHECK(stopped_track.size() == 1);
 CHECK(std::abs(stopped_track.front().state.v.x) < 0.10);
 const auto dwell_forecast = shuttle.predict(3.0, 1.0, 0.1, 1.0);
 CHECK(dwell_forecast.size() == 11);
 CHECK(dwell_forecast.front().size() == 2);
 CHECK(dwell_forecast[5][0].p.x != dwell_forecast[5][1].p.x);

 // Once sustained measurements establish the return leg, velocity changes
 // sign and the learned corridor contains all reflected forecasts.
 for (int sample = 1; sample <= 20; ++sample)
 shuttle.observe({7, {1.0 - 0.05 * sample, 0.0}, 0.4}, 3.0 + sample * 0.1);
 const auto reversed_track = shuttle.active_tracks(5.0, 1.0);
 CHECK(reversed_track.front().state.v.x < -0.40);
 const auto learned_motion = shuttle.predict_inputs();
 CHECK(learned_motion.size() == 1);
 CHECK(learned_motion.front().endpoint_dwell_s >= 0.5);
 const auto predictions = shuttle.predict(5.0, 1.0, 0.1, 1.0);
 CHECK(predictions.size() == 11);
 for (const auto& frame : predictions) {
 CHECK(frame.size() == 1);
 CHECK(frame[0].p.x >= -0.10);
 CHECK(frame[0].p.x <= 1.10);
 }

 // Even without a learned reversal boundary (for example after track
 // fragmentation), endpoint dwell keeps continuation and reversal modes.
 sim::DynamicObstacleMap fragmented;
 fragmented.observe({9, {0.0, 0.0}, 0.4}, 0.0);
 for (int sample = 1; sample <= 20; ++sample)
 fragmented.observe({9, {0.05 * sample, 0.0}, 0.4}, sample * 0.1);
 for (int sample = 21; sample <= 30; ++sample)
 fragmented.observe({9, {1.0, 0.0}, 0.4}, sample * 0.1);
 const auto ambiguous = fragmented.predict(3.0, 1.0, 0.1, 1.0);
 CHECK(ambiguous.front().size() == 2);
 CHECK(ambiguous.back()[0].p.x > 1.3);
 CHECK(ambiguous.back()[1].p.x < 0.7);
 const auto nominal = fragmented.predict_nominal(3.0, 1.0, 0.1, 1.0);
 CHECK(nominal.front().size() == 1);
 CHECK(nominal.back().front().p.x > 1.3);

 // At a 5 Hz sensor cadence, the smooth position state may coast beyond a
 // shuttle endpoint. Intent classification must follow the measurement-time
 // regression and enter the ambiguous state during the dwell, before the
 // physical target reverses. Small alternating centroid noise must not keep
 // publishing the stale outbound intent as certain.
 sim::DynamicObstacleMap slow_shuttle;
 slow_shuttle.observe({10, {0.0, 0.0}, 0.4}, 0.0);
 for (int sample = 1; sample <= 10; ++sample)
 slow_shuttle.observe({10, {0.11 * sample, 0.0}, 0.4}, sample * 0.2);
 for (int sample = 11; sample <= 20; ++sample) {
 const double centroid_noise = sample % 2 == 0 ? 0.02 : -0.02;
 slow_shuttle.observe({10, {1.10 + centroid_noise, 0.0}, 0.4}, sample * 0.2);
 }
 const auto slow_dwell_track = slow_shuttle.active_tracks(4.0, 1.0);
 CHECK(slow_dwell_track.size() == 1);
 CHECK(slow_dwell_track.front().motion_class == sim::TrackMotionClass::IntentAmbiguous);
 const auto slow_dwell_forecast = slow_shuttle.predict(4.0, 1.0, 0.1, 1.0);
 CHECK(!slow_dwell_forecast.empty());
 CHECK(slow_dwell_forecast.front().size() == 2);

 const auto dwa = sim::dwa_compute({1.0, 2.0}, 0.0, 0.0, 0.0, {8.0, 2.0}, open_map, {1.0, 2.0, 0.3}, 1.5, 1.5,
                                   2.0, 1.5, 0.1, 0.38, 8.0);
 CHECK(dwa.best.command.valid);
 CHECK(!dwa.best.samples.empty());
 CHECK(dwa.best.samples.front().position.x == 1.0);
 CHECK(dwa.best.samples.front().position.y == 2.0);
 CHECK(dwa.best.samples.size() > 2);
 CHECK(!dwa.rollouts.empty());
 CHECK(dwa.rollouts.front().trajectory.samples.size() > 2);

 // A moving object crossing between prediction frames must be checked at the
 // candidate's exact time. Floor-index lookup misses this collision because
 // neither endpoint occupies the ego position.
 const std::vector<std::vector<sim::DynamicObjectState>> crossing_frames{
 {{{0.0, 0.0}, {2.0, 0.0}, 0.10}},
 {{{2.0, 0.0}, {2.0, 0.0}, 0.10}},
 };
 const auto interpolated_crossing = sim::dwa_compute_trajectory(
 {1.0, 0.0}, 0.0, 0.0, 0.0, {1.0, 0.0}, open_map,
 {1.0, 2.0, 0.3}, crossing_frames, 0.0, 0.0, 1.0,
 0.5, 0.5, 1.0, 0.0, 0.10, 1.0, {}, 0.0);
 CHECK(!interpolated_crossing.rollouts.empty());
 CHECK(interpolated_crossing.rollouts.front().collision);
 CHECK(!interpolated_crossing.best.command.valid);

 // Prediction uncertainty grows monotonically with forecast time and remains
 // distinct from the object's physical radius.
 sim::PredictInput uncertain;
 uncertain.state = {{0.0, 0.0}, {1.0, 0.0}, 0.20, 0.03};
 uncertain.position_uncertainty_growth_mps = 0.10;
 uncertain.last_observed_s = 0.0;
 const auto uncertain_frames = sim::predict_frames({uncertain}, 0.0, 1.0, 0.5, 2.0);
 CHECK(uncertain_frames.size() == 3);
 CHECK(std::abs(uncertain_frames.front().front().position_uncertainty_m - 0.03) < 1e-12);
 CHECK(std::abs(uncertain_frames.back().front().position_uncertainty_m - 0.13) < 1e-12);
 CHECK(uncertain_frames.back().front().radius == 0.20);

 const auto partial_step = sim::dwa_compute({1.0, 2.0}, 0.0, 0.0, 0.0, {8.0, 2.0}, open_map, {1.0, 2.0, 0.3},
                                            1.5, 1.5, 2.0, 0.25, 0.1, 0.38, 8.0);
 CHECK(partial_step.best.command.valid);
 CHECK(std::abs(partial_step.best.samples.back().t_s - 0.25) < 1e-12);
 for (std::size_t sample = 1; sample < partial_step.best.samples.size(); ++sample)
  CHECK(partial_step.best.samples[sample].t_s > partial_step.best.samples[sample - 1].t_s);

 const auto cliff_aware =
   sim::dwa_compute({1.0, 2.0}, 0.0, 0.0, 0.0, {8.0, 2.0}, open_map, {1.0, 2.0, 0.3}, 1.5, 1.5, 2.0, 1.5, 0.1,
                    0.38, 8.0, [](sim::Vec2, sim::Vec2 to) { return to.x < 1.2; });
 CHECK(cliff_aware.best.command.valid);
 for (const auto& sample : cliff_aware.best.samples)
  CHECK(sample.position.x < 1.2);
 CHECK(std::any_of(cliff_aware.rollouts.begin(), cliff_aware.rollouts.end(),
                   [](const auto& rollout) { return rollout.collision; }));

 // Behavior can forbid reversing after it commits to a mapped crossing;
 // DWA still samples and collision-checks the remaining forward window.
 const auto committed =
   sim::dwa_compute_trajectory({1.0, 2.0}, 0.0, 0.0, 0.0, {8.0, 2.0}, open_map, {1.0, 2.0, 0.3}, {}, 1.5, 1.5,
                               2.0, 1.5, 0.1, 0.1, 0.0, 0.38, 8.0, {}, 0.0);
 CHECK(committed.best.command.valid);
 CHECK(committed.best.command.v >= 0.0);
 CHECK(std::all_of(committed.rollouts.begin(), committed.rollouts.end(),
                   [](const auto& rollout) { return rollout.trajectory.command.v >= 0.0; }));

 // A soft-margin incursion admits an escape rollout only when separation
 // grows; stopping inside the margin must not be the planner's sole answer.
 const std::vector<std::vector<sim::DynamicObjectState>> close_behind(16, {{{0.1, 2.0}, {0.0, 0.0}, 0.4}});
 const auto escaping =
   sim::dwa_compute_trajectory({1.0, 2.0}, 0.0, 0.0, 0.0, {8.0, 2.0}, open_map, {1.0, 2.0, 0.3}, close_behind,
                               1.5, 1.5, 2.0, 1.5, 0.1, 0.1, 0.5, 0.38, 8.0, {}, 0.0);
 CHECK(escaping.best.command.valid);
 CHECK(escaping.best.command.v > 0.0);
 const auto low_speed_crossing =
   sim::dwa_compute_trajectory({1.0, 2.0}, 0.0, 0.0, 0.0, {8.0, 2.0}, open_map, {1.0, 2.0, 0.3}, {}, 0.20,
                               1.5, 2.0, 1.5, 0.1, 0.1, 0.5, 0.38, 8.0, {}, 0.0);
 CHECK(low_speed_crossing.best.command.valid);
 CHECK(std::abs(low_speed_crossing.best.command.v - 0.20) < 1e-12);
 const auto low_speed_with_reverse =
   sim::dwa_compute_trajectory({1.0, 2.0}, 0.0, 0.0, 0.0, {8.0, 2.0}, open_map, {1.0, 2.0, 1.0}, {}, 0.10,
                               1.5, 2.0, 1.5, 0.1, 0.1, 0.5, 0.38, 8.0, {}, -0.5);
 CHECK(low_speed_with_reverse.best.command.valid);
 CHECK(std::abs(low_speed_with_reverse.best.command.v - 0.10) < 1e-12);

 double left = 0.0;
 double right = 0.0;
 sim::DifferentialKinematics::twist_to_wheels(1.0, 1.0, 0.52, left, right);
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
