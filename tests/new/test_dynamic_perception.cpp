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
 // A temporary object does not need to move before it becomes a dynamic
 // safety input. Repeated timestamped observations establish a stationary
 // model; the old non-zero-velocity gate silently dropped pallets and parked
 // vehicles from prediction.
 sim::DynamicObstacleMap stationary_map;
 for (int sample = 0; sample < 6; ++sample)
 stationary_map.observe({99, {2.0, 3.0}, 0.35}, 0.1 * sample);
 const auto stationary_inputs = stationary_map.predict_inputs();
 CHECK(stationary_inputs.size() == 1);
 CHECK(stationary_inputs.front().state.prediction_ready);
 CHECK(std::hypot(stationary_inputs.front().state.v.x,
 stationary_inputs.front().state.v.y) < 1e-9);
 const auto stationary_frames = stationary_map.predict(0.5, 1.0, 0.1, 1.0);
 CHECK(stationary_frames.size() == 11);
 for (const auto& frame : stationary_frames) {
 CHECK(frame.size() == 1);
 CHECK(std::abs(frame.front().p.x - 2.0) < 1e-9);
 CHECK(std::abs(frame.front().p.y - 3.0) < 1e-9);
 }

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
 CHECK(first_frame.lidar->time_increment_s == 0.0); // ideal focused scan is instantaneous
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
 CHECK(std::all_of(first_perception.obstacle_rays.begin(),
 first_perception.obstacle_rays.end(), [](const auto& ray) {
 return !ray.hit || ray.physical_return;
 }));
 bool motion_confirmed = false;
 bool confirmed_track_removed_from_static_layer = false;
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
 if (motion_confirmed) {
 CHECK(!moving_perception.dynamic_clearing_regions.empty());
 confirmed_track_removed_from_static_layer = std::none_of(
 moving_perception.obstacle_rays.begin(),
 moving_perception.obstacle_rays.end(), [](const auto& ray) {
 return ray.hit;
 });
 CHECK(std::any_of(
 moving_perception.obstacle_rays.begin(),
 moving_perception.obstacle_rays.end(), [](const auto& ray) {
 return ray.physical_return;
 }));
 }
 }
 CHECK(motion_confirmed);
 CHECK(confirmed_track_removed_from_static_layer);
 const auto anonymous_tracks = perception.dynamic_obstacles().active_tracks(last_frame_time, 1.0);
 CHECK(anonymous_tracks.size() == 1);
CHECK(anonymous_tracks.front().motion_class != sim::TrackMotionClass::Unconfirmed);
CHECK(std::abs(anonymous_tracks.front().state.p.x - truth.dynamic_objects.front().p.x) < 0.35);

 // Angular sample density is a sensor characteristic, not an object count.
 // The same isolated moving cylinder must remain one confirmed anonymous
 // track with sparse teaching, ordinary and dense reference scans. One whole
 // dropped scan is allowed by the declared track age and must not change ID.
 for (const int rays : {48, 360, 800}) {
 sim::SensorSystem density_sensors(101 + rays, 103 + rays);
 sim::LidarPerceptionSystem density_perception(open_map, 107 + rays);
 sim::WorldTruth density_truth;
 density_truth.robot = {1.0, 5.0, 0.0, 0.0, 0.0};
 density_truth.dynamic_objects = {{{5.0, 4.5}, {0.0, 0.30}, 0.5}};
 sim::LidarConfig density_config;
 density_config.rays = rays;
 density_config.min_range_m = 0.2;
 density_config.range_m = 10.0;
 density_config.scan_hz = 10.0;
 std::size_t confirmed_id = std::numeric_limits<std::size_t>::max();
 for (int sample = 0; sample <= 30; ++sample) {
 const double stamp_s = 0.1 * sample;
 density_truth.dynamic_objects.front().p.y = 4.5 + 0.30 * stamp_s;
 if (sample != 16) {
 const auto frame = density_sensors.capture_lidar(
 density_truth, stamp_s, density_config,
 sim::planar_lidar_truth(open_map, {1.0, 5.0}, 0.0, rays, 10.0,
 density_truth.dynamic_objects));
 density_perception.process(
 *frame.lidar, *frame.lidar_static_reference,
 {{1.0, 5.0}, 0.0, 0.0, 0.0, 0.0}, {}, 1.0);
 }
 const auto density_tracks =
 density_perception.dynamic_obstacles().active_tracks(stamp_s, 1.0);
 const auto confirmed_count = static_cast<int>(std::count_if(
 density_tracks.begin(), density_tracks.end(), [](const auto& track) {
 return track.motion_class != sim::TrackMotionClass::Unconfirmed;
 }));
 CHECK(confirmed_count <= 1);
 if (confirmed_count == 1) {
 const auto& track = *std::find_if(
 density_tracks.begin(), density_tracks.end(), [](const auto& candidate) {
 return candidate.motion_class != sim::TrackMotionClass::Unconfirmed;
 });
 if (confirmed_id == std::numeric_limits<std::size_t>::max())
 confirmed_id = track.object_id;
 CHECK(track.object_id == confirmed_id);
 }
 }
 const auto density_tracks =
 density_perception.dynamic_obstacles().active_tracks(3.0, 1.0);
 CHECK(density_tracks.size() == 1);
 CHECK(density_tracks.front().object_id == confirmed_id);
 CHECK(density_tracks.front().motion_class != sim::TrackMotionClass::Unconfirmed);
 // Forty-eight rays quantize a target at this range by roughly half a
 // metre, so it is a valid teaching profile for detection/track continuity,
 // not a truthful 0.3 m/s velocity instrument. Standard and dense profiles
 // must recover the motion quantitatively.
 if (rays >= 360)
 CHECK(std::abs(density_tracks.front().state.v.y - 0.30) < 0.12);
 CHECK(std::abs(density_tracks.front().state.p.y - 5.40) < 0.35);
 }

 // One missing foreground beam inside a dense silhouette is not evidence of
 // two physical objects. Endpoint-distance clustering may bridge exactly one
 // beam when both sides remain geometrically connected.
 {
 constexpr int kRays = 800;
 sim::SensorSystem missed_beam_sensors(1201, 1203);
 sim::LidarPerceptionSystem missed_beam_perception(open_map, 1207);
 sim::WorldTruth missed_beam_truth;
 missed_beam_truth.robot = {1.0, 5.0, 0.0, 0.0, 0.0};
 missed_beam_truth.dynamic_objects = {{{4.0, 8.0}, {}, 0.5}};
 sim::LidarConfig missed_beam_config;
 missed_beam_config.rays = kRays;
 missed_beam_config.min_range_m = 0.2;
 missed_beam_config.range_m = 10.0;
 missed_beam_config.scan_hz = 10.0;
 for (int sample = 0; sample < 6; ++sample) {
 auto frame = missed_beam_sensors.capture_lidar(
 missed_beam_truth, sample * 0.1, missed_beam_config,
 sim::planar_lidar_truth(open_map, {1.0, 5.0}, 0.0, kRays, 10.0,
 missed_beam_truth.dynamic_objects));
 std::vector<int> target_rays;
 for (int ray = 0; ray < kRays; ++ray) {
 if (frame.lidar->hit[static_cast<std::size_t>(ray)] &&
 !frame.lidar_static_reference->hit[static_cast<std::size_t>(ray)]) {
 target_rays.push_back(ray);
 }
 }
 CHECK(target_rays.size() >= 5);
 const auto missing = static_cast<std::size_t>(
 target_rays[target_rays.size() / 2]);
 frame.lidar->hit[missing] = 0;
 frame.lidar->ranges[missing] = frame.lidar->max_range;
 frame.lidar->intensities[missing] = 0.0;
 missed_beam_perception.process(
 *frame.lidar, *frame.lidar_static_reference,
 {{1.0, 5.0}, 0.0, 0.0, 0.0, 0.0}, {}, 1.0);
 }
 const auto missed_beam_tracks =
 missed_beam_perception.dynamic_obstacles().active_tracks(0.5, 1.0);
 CHECK(missed_beam_tracks.size() == 1);
 }
 // A wider, repeatable silhouette split reaches the detection-merge layer
 // rather than the one-beam cluster bridge. Overlapping reconstructed
 // occupied circles still represent one round target and must confirm one ID.
 {
 constexpr int kRays = 800;
 sim::SensorSystem occlusion_sensors(1301, 1303);
 sim::LidarPerceptionSystem occlusion_perception(open_map, 1307);
 sim::WorldTruth occlusion_truth;
 occlusion_truth.robot = {1.0, 5.0, 0.0, 0.0, 0.0};
 occlusion_truth.dynamic_objects = {{{4.0, 8.0}, {}, 0.5}};
 sim::LidarConfig occlusion_config;
 occlusion_config.rays = kRays;
 occlusion_config.min_range_m = 0.2;
 occlusion_config.range_m = 10.0;
 occlusion_config.scan_hz = 10.0;
 for (int sample = 0; sample < 6; ++sample) {
 auto frame = occlusion_sensors.capture_lidar(
 occlusion_truth, sample * 0.1, occlusion_config,
 sim::planar_lidar_truth(open_map, {1.0, 5.0}, 0.0, kRays, 10.0,
 occlusion_truth.dynamic_objects));
 std::vector<int> target_rays;
 for (int ray = 0; ray < kRays; ++ray) {
 if (frame.lidar->hit[static_cast<std::size_t>(ray)] &&
 !frame.lidar_static_reference->hit[static_cast<std::size_t>(ray)])
 target_rays.push_back(ray);
 }
 CHECK(target_rays.size() >= 11);
 const std::size_t middle = target_rays.size() / 2;
 for (std::size_t offset = middle - 2; offset <= middle + 2; ++offset) {
 const auto missing = static_cast<std::size_t>(target_rays[offset]);
 frame.lidar->hit[missing] = 0;
 frame.lidar->ranges[missing] = frame.lidar->max_range;
 frame.lidar->intensities[missing] = 0.0;
 }
 occlusion_perception.process(
 *frame.lidar, *frame.lidar_static_reference,
 {{1.0, 5.0}, 0.0, 0.0, 0.0, 0.0}, {}, 1.0);
 }
 CHECK(occlusion_perception.dynamic_obstacles().active_tracks(0.5, 1.0).size() == 1);
 }

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
 CHECK(std::any_of(wall_perception.obstacle_rays.begin(),
 wall_perception.obstacle_rays.end(), [](const auto& ray) {
 return ray.physical_return;
 }));
 }
 }
 CHECK(wall_perception_system.dynamic_obstacles().active_tracks(0.2, 1.0).empty());

 // A three-degree registration error moves a thin wall edge between rays.
 // Repeated static frames must remain map-consistent instead of acquiring a
 // fictitious high-speed track. The angular offset is an independent input;
 // the test does not reproduce the correspondence implementation.
 sim::LidarPerceptionSystem shifted_wall_perception(wall_map, 23);
 for (int frame_index = 0; frame_index < 8; ++frame_index) {
 auto wall_frame = wall_sensors.capture_lidar(
 wall_truth, 1.0 + 0.1 * frame_index, lidar_config,
 sim::planar_lidar_truth(wall_map, {1.0, 5.0}, 0.0, 360, 10.0));
 auto shifted_ranges = wall_frame.lidar->ranges;
 auto shifted_hits = wall_frame.lidar->hit;
 auto shifted_intensities = wall_frame.lidar->intensities;
 for (std::size_t ray = 0; ray < shifted_ranges.size(); ++ray) {
 const std::size_t source = (ray + shifted_ranges.size() - 3) % shifted_ranges.size();
 wall_frame.lidar->ranges[ray] = shifted_ranges[source];
 wall_frame.lidar->hit[ray] = shifted_hits[source];
 wall_frame.lidar->intensities[ray] = shifted_intensities[source];
 }
 const auto shifted = shifted_wall_perception.process(
 *wall_frame.lidar, *wall_frame.lidar_static_reference,
 {wall_estimate.position, wall_estimate.heading_rad,
 wall_estimate.linear_velocity_mps, wall_estimate.lateral_velocity_mps,
 wall_estimate.angular_velocity_radps}, {}, 1.0);
 CHECK(!shifted.dynamic_object_detected);
 }
 CHECK(shifted_wall_perception.dynamic_obstacles().active_tracks(1.7, 1.0).empty());

 // A degraded map pose must not turn static geometry into anonymous moving
 // actors. The raw foreground returns remain available to the local obstacle
 // layer and localization-independent safety guard, but map-relative dynamic
 // tracks are withheld until pose confidence recovers.
 sim::LidarPerceptionSystem uncertain_wall_perception(wall_map, 31);
 for (int frame_index = 0; frame_index < 4; ++frame_index) {
 auto wall_frame = wall_sensors.capture_lidar(
 wall_truth, 2.0 + 0.1 * frame_index, lidar_config,
 sim::planar_lidar_truth(wall_map, {1.0, 5.0}, 0.0, 360, 10.0));
 auto uncertain_state = wall_estimate;
 uncertain_state.localization_confidence = 0.20;
 uncertain_state.localization_valid = true;
 const auto uncertain = uncertain_wall_perception.process(
 *wall_frame.lidar, *wall_frame.lidar_static_reference,
 {uncertain_state.position, uncertain_state.heading_rad,
 uncertain_state.linear_velocity_mps, uncertain_state.lateral_velocity_mps,
 uncertain_state.angular_velocity_radps, 0.0, 0.0, 0.0, false,
 uncertain_state.localization_confidence, uncertain_state.localization_valid},
 {}, 1.0);
 CHECK(!uncertain.dynamic_object_detected);
 }
 CHECK(uncertain_wall_perception.dynamic_obstacles().active_tracks(2.3, 1.0).empty());

 // A pitched single-line scanner can intersect a slope or flat ground. The
 // endpoint is free in a 2D occupancy grid but lies on the 2.5D elevation
 // layer, so it must be ground rather than a moving object. The reference is
 // deliberately empty to model a small attitude/registration mismatch.
 sim::GridMap elevation_map(100, 50, 0.2);
 for (int y = 0; y < elevation_map.h(); ++y) {
 for (int x = 0; x < elevation_map.w(); ++x) {
 const auto centre = elevation_map.gridToWorld(x, y);
 elevation_map.setElevation(centre.x, centre.y, 0.0);
 }
 }
 sim::LidarPerceptionSystem ground_perception(elevation_map, 29);
 sim::Scan ground_scan;
 ground_scan.n_rays = 360;
 ground_scan.max_range = 10.0;
 ground_scan.angle_increment_rad = 2.0 * sim::kPi / 360.0;
 ground_scan.ranges.assign(360, ground_scan.max_range);
 ground_scan.hit.assign(360, 0);
 ground_scan.ranges[0] = 0.56 / std::sin(0.10);
 ground_scan.hit[0] = 1;
 ground_scan.header.frame_id = "lidar_link";
 sim::Scan empty_ground_reference = ground_scan;
 empty_ground_reference.ranges[0] = empty_ground_reference.max_range;
 empty_ground_reference.hit[0] = 0;
 for (int frame_index = 0; frame_index < 8; ++frame_index) {
 ground_scan.header.stamp_s = 2.0 + 0.1 * frame_index;
 empty_ground_reference.header.stamp_s = ground_scan.header.stamp_s;
 const auto frame = ground_perception.process(
 ground_scan, empty_ground_reference,
 {{1.0, 5.0}, 0.0, 0.0, 0.0, 0.0,
 0.24, 0.0, 0.10, true},
 {{0.0, 0.0}, 0.0, 0.0, 0.45,
 0.32, 0.0, 0.0, 0.30}, 1.0);
 CHECK(!frame.obstacle_rays[0].hit);
 CHECK(frame.obstacle_rays[0].physical_return);
 CHECK(!frame.dynamic_object_detected);
 }
 CHECK(ground_perception.dynamic_obstacles().active_tracks(2.7, 1.0).empty());

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
 const auto long_constant_forecast =
 constant_velocity.predict(2.0, 3.0, 1.0, 1.0);
 CHECK(long_constant_forecast.size() == 4);
 CHECK(long_constant_forecast.back().front().position_uncertainty_m > 0.10);
 CHECK(long_constant_forecast.back().front().position_uncertainty_m >=
 long_constant_forecast.front().front().position_uncertainty_m);
 // A stale sample is ignored according to sensor time; arrival order cannot
 // reverse an otherwise valid track.
 constant_velocity.observe({1, {-20.0, 0.0}, 0.5}, 1.9);
 const auto after_stale = constant_velocity.active_tracks(2.0, 1.0);
 CHECK(after_stale.front().state.p.x == moving.front().state.p.x);
 CHECK(after_stale.front().state.v.x == moving.front().state.v.x);

 // Intent is a temporal state, not a one-sample speed threshold. The first
 // fitted-speed sample below the low threshold must retain the established
 // motion intent; a second distinct observation confirms ambiguity. This is
 // an independent state-transition oracle and does not reproduce the filter.
 sim::DynamicObstacleMap intent_confirmation;
 intent_confirmation.observe({2, {0.0, 0.0}, 0.5}, 0.0);
 for (int sample = 1; sample <= 20; ++sample)
 intent_confirmation.observe(
 {2, {0.55 * sample * 0.1, 0.0}, 0.5}, sample * 0.1);
 bool saw_first_low_sample = false;
 bool saw_confirmed_ambiguity = false;
 double held_position = 1.10;
 for (int sample = 21; sample <= 40; ++sample) {
 intent_confirmation.observe(
 {2, {held_position, 0.0}, 0.5}, sample * 0.1);
 const auto track = intent_confirmation.active_tracks(sample * 0.1, 1.0);
 CHECK(track.size() == 1);
 if (std::hypot(track.front().state.v.x, track.front().state.v.y) < 0.40) {
 if (!saw_first_low_sample) {
 CHECK(track.front().motion_class == sim::TrackMotionClass::Moving);
 saw_first_low_sample = true;
 } else {
 CHECK(track.front().motion_class == sim::TrackMotionClass::IntentAmbiguous);
 saw_confirmed_ambiguity = true;
 break;
 }
 }
 }
 CHECK(saw_first_low_sample);
 CHECK(saw_confirmed_ambiguity);

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
 CHECK(dwell_forecast.front().size() >= 2);
 CHECK(dwell_forecast[5][0].p.x != dwell_forecast[5][1].p.x);

 // One observed reversal establishes one endpoint, not a complete shuttle
 // corridor. The travelled frontier on the unseen side must remain open;
 // otherwise each new observation moves a fake reflection boundary and can
 // invalidate a previously admitted crossing schedule.
 bool checked_reversal_position = false;
 for (int sample = 1; sample <= 20; ++sample)
 {
 shuttle.observe({7, {1.0 - 0.05 * sample, 0.0}, 0.4}, 3.0 + sample * 0.1);
 const auto current = shuttle.active_tracks(3.0 + sample * 0.1, 1.0);
 const auto modes = shuttle.predict_inputs();
 if (!current.empty() && !modes.empty() &&
 modes.front().alternate_velocity_valid &&
 current.front().state.p.x > 0.10 && current.front().state.p.x < 0.90) {
 // Velocity ambiguity may add modes, but every mode starts at the current
 // timestamped position. Endpoint snapping here would teleport the actor
 // and manufacture a false crossing conflict several metres away.
 CHECK(std::abs(modes.front().state.p.x - current.front().state.p.x) < 1e-12);
 checked_reversal_position = true;
 }
 }
 CHECK(checked_reversal_position);
 const auto reversed_track = shuttle.active_tracks(5.0, 1.0);
 CHECK(reversed_track.front().state.v.x < -0.40);
 const auto learned_motion = shuttle.predict_inputs();
 CHECK(learned_motion.size() == 1);
 CHECK(learned_motion.front().endpoint_dwell_s >= 0.5);
 CHECK(!learned_motion.front().bounded_x);
// Readiness comes from sufficient timestamped history, not from observing a
// complete shuttle cycle. The missing opposite endpoint limits the periodic
// hypothesis but does not erase a usable generic motion estimate.
CHECK(learned_motion.front().state.prediction_ready);
 const auto predictions = shuttle.predict(5.0, 1.0, 0.1, 1.0);
 CHECK(predictions.size() == 11);
 CHECK(!predictions.back().empty());
 CHECK(predictions.back().front().p.x < -0.10);

 // Endpoint dwell may be asymmetric. After learning a long pause at x=1,
 // an immediate reversal at x=0 lowers the shared conservative dwell bound
 // to zero; applying the long pause to both ends would create a false safe
 // crossing window just before the no-dwell return.
 for (int sample = 1; sample <= 10; ++sample)
 shuttle.observe({7, {0.05 * sample, 0.0}, 0.4}, 5.0 + sample * 0.1);
 const auto asymmetric_dwell = shuttle.predict_inputs();
 CHECK(asymmetric_dwell.size() == 1);
 CHECK(asymmetric_dwell.front().endpoint_dwell_s == 0.0);
 CHECK(asymmetric_dwell.front().bounded_x);
 CHECK(asymmetric_dwell.front().state.prediction_ready);
 const auto bounded_predictions = shuttle.predict(6.0, 1.0, 0.1, 1.0);
 for (const auto& frame : bounded_predictions) {
 CHECK(frame.size() >= 3);
 for (const auto& hypothesis : frame) {
 CHECK(hypothesis.p.x >= -0.10);
 CHECK(hypothesis.p.x <= 1.10);
 }
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

 // Leaving the field of view does not create a behavior-grade phantom.
 // Once the last observation exceeds the declared freshness budget, every
 // prediction frame is empty and crossing behavior must seek independent
 // positive free-space evidence rather than extrapolate this track.
 sim::DynamicObstacleMap departed;
 departed.observe({11, {0.0, 0.0}, 0.4}, 0.0);
 for (int sample = 1; sample <= 10; ++sample)
 departed.observe({11, {0.05 * sample, 0.0}, 0.4}, sample * 0.1);
 const auto stale_departure = departed.predict(4.0, 2.0, 0.1, 1.0);
 CHECK(stale_departure.size() == 21);
 for (const auto& frame : stale_departure)
 CHECK(frame.empty());

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
 CHECK(slow_dwell_forecast.front().size() >= 2);

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

 // Velocity uncertainty on a bounded motion manifold changes arrival phase
 // along that manifold. It must not inflate the actor equally in the
 // cross-track direction.
 sim::PredictInput phase_uncertain;
 phase_uncertain.state = {{0.0, 2.0}, {1.0, 0.0}, 0.20, 0.03, true};
 phase_uncertain.velocity_uncertainty_mps = 0.20;
 phase_uncertain.min_observed = {0.0, 2.0};
 phase_uncertain.max_observed = {10.0, 2.0};
 phase_uncertain.bounded_x = true;
 const auto phase_frames = sim::predict_frames(
 {phase_uncertain}, 0.0, 1.0, 1.0, 2.0);
 CHECK(phase_frames.back().size() == 3);
 CHECK(std::abs(phase_frames.back()[0].p.x - 0.8) < 1e-12);
 CHECK(std::abs(phase_frames.back()[1].p.x - 1.0) < 1e-12);
 CHECK(std::abs(phase_frames.back()[2].p.x - 1.2) < 1e-12);
 for (const auto& hypothesis : phase_frames.back()) {
 CHECK(std::abs(hypothesis.p.y - 2.0) < 1e-12);
 CHECK(std::abs(hypothesis.position_uncertainty_m - 0.03) < 1e-12);
 }

 // Adjacent-frame interpolation relies on stable object/hypothesis order.
 // One nominal+alternate input followed by a nominal-only input must retain
 // that exact three-slot identity for the complete forecast.
 sim::PredictInput multimode;
 multimode.state = {{0.0, 0.0}, {1.0, 0.0}, 0.2};
 multimode.alternate_velocity = {-1.0, 0.0};
 multimode.alternate_velocity_valid = true;
 sim::PredictInput second_object;
 second_object.state = {{10.0, 0.0}, {2.0, 0.0}, 0.3};
 const auto ordered_frames = sim::predict_frames(
 {multimode, second_object}, 0.0, 1.0, 0.5, 2.0);
 CHECK(ordered_frames.size() == 3);
 CHECK(std::all_of(ordered_frames.begin(), ordered_frames.end(),
 [](const auto& frame) { return frame.size() == 3; }));
 CHECK(std::abs(ordered_frames.back()[0].p.x - 1.0) < 1e-12);
 CHECK(std::abs(ordered_frames.back()[1].p.x + 1.0) < 1e-12);
 CHECK(std::abs(ordered_frames.back()[2].p.x - 12.0) < 1e-12);

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
 const auto swept_evidence = sim::predicted_gap_evidence(
 line, 0, 0.0, crossing, 0.1, 0.38, 0.0);
 CHECK(swept_evidence.conflict);
 CHECK(std::abs(swept_evidence.min_gap_m - swept) < 1e-12);
 // The envelope radius is 0.88 m and the crossing stays at y=0.1 m.
 // This analytic root is the first entry, earlier than closest approach.
 const double expected_entry_fraction =
 (1.0 - std::sqrt(0.88 * 0.88 - 0.1 * 0.1)) / 2.0;
 CHECK(std::abs(swept_evidence.first_conflict_s -
 expected_entry_fraction * 0.1) < 1e-12);
 CHECK(std::abs(swept_evidence.first_conflict_position.x) < 1e-12);
 // A target can use the same route position long before the ego gets there.
 // A spatial union would report a false conflict; same-time relative motion
 // remains five metres apart throughout this finite window.
 const std::vector<std::vector<sim::DynamicObjectState>> already_gone = {
 {{{5.0, 0.0}, {1.0, 0.0}, 0.2}},
 {{{6.0, 0.0}, {1.0, 0.0}, 0.2}},
 };
 const auto different_times = sim::predicted_gap_evidence(
 line, 0, 1.0, already_gone, 1.0, 0.38, 0.5);
 CHECK(!different_times.conflict);
 CHECK(std::abs(different_times.min_gap_m - 4.42) < 1e-12);
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
 // A planner may correctly drop an unobserved target after its freshness
 // budget while the tracker retains a longer dormant association hypothesis.
 // Reappearing after a two-second occlusion must update the established ID,
 // not create a new immature actor and discard learned motion history.
 {
 constexpr int kRays = 800;
 sim::SensorSystem coast_sensors(1901, 1903);
 sim::LidarPerceptionSystem coast_perception(open_map, 1907);
 sim::WorldTruth coast_truth;
 coast_truth.robot = {1.0, 5.0, 0.0, 0.0, 0.0};
 sim::LidarConfig coast_config;
 coast_config.rays = kRays;
 coast_config.min_range_m = 0.2;
 coast_config.range_m = 10.0;
 coast_config.scan_hz = 10.0;
 const auto process_coast_scan = [&](double stamp_s) {
 const auto frame = coast_sensors.capture_lidar(
 coast_truth, stamp_s, coast_config,
 sim::planar_lidar_truth(open_map, {1.0, 5.0}, 0.0, kRays, 10.0,
 coast_truth.dynamic_objects));
 coast_perception.process(
 *frame.lidar, *frame.lidar_static_reference,
 {{1.0, 5.0}, 0.0, 0.0, 0.0, 0.0}, {}, 1.25);
 };
 for (int sample = 0; sample < 10; ++sample) {
 const double stamp_s = 0.1 * sample;
 coast_truth.dynamic_objects = {{{5.0, 4.5 + 0.30 * stamp_s},
 {0.0, 0.30}, 0.5}};
 process_coast_scan(stamp_s);
 }
 const auto before_occlusion =
 coast_perception.dynamic_obstacles().active_tracks(0.9, 1.25);
 CHECK(before_occlusion.size() == 1);
 const auto coast_id = before_occlusion.front().object_id;
 coast_truth.dynamic_objects.clear();
 for (int sample = 10; sample < 30; ++sample)
 process_coast_scan(0.1 * sample);
 CHECK(coast_perception.dynamic_obstacles().active_tracks(2.9, 1.25).empty());
 coast_truth.dynamic_objects = {{{5.0, 5.4}, {0.0, 0.30}, 0.5}};
 process_coast_scan(3.0);
 const auto after_occlusion =
 coast_perception.dynamic_obstacles().active_tracks(3.0, 1.25);
 CHECK(after_occlusion.size() == 1);
 CHECK(after_occlusion.front().object_id == coast_id);
 }

 // Association must use the same bounded-motion phase model as planning.
 // Train a shuttle over both corridor legs, hide it for 2.4 seconds, then
 // reintroduce it at the analytically reflected phase. Linear coasting would
 // be metres outside the corridor and would fragment the identity here.
 {
 constexpr int kRays = 800;
 sim::SensorSystem shuttle_sensors(2001, 2003);
 sim::LidarPerceptionSystem shuttle_perception(open_map, 2007);
 sim::WorldTruth shuttle_truth;
 shuttle_truth.robot = {1.0, 5.0, 0.0, 0.0, 0.0};
 sim::LidarConfig shuttle_config;
 shuttle_config.rays = kRays;
 shuttle_config.min_range_m = 0.2;
 shuttle_config.range_m = 10.0;
 shuttle_config.scan_hz = 10.0;
 const auto process_shuttle_scan = [&](double stamp_s) {
 const auto frame = shuttle_sensors.capture_lidar(
 shuttle_truth, stamp_s, shuttle_config,
 sim::planar_lidar_truth(open_map, {1.0, 5.0}, 0.0,
 kRays, 10.0, shuttle_truth.dynamic_objects));
 shuttle_perception.process(
 *frame.lidar, *frame.lidar_static_reference,
 {{1.0, 5.0}, 0.0, 0.0, 0.0, 0.0}, {}, 1.25);
 };
 for (int sample = 0; sample <= 20; ++sample) {
 const double stamp_s = 0.1 * sample;
 shuttle_truth.dynamic_objects = {{{5.0, 4.5 + 0.05 * sample},
 {0.0, 0.5}, 0.5}};
 process_shuttle_scan(stamp_s);
 }
 for (int sample = 21; sample <= 40; ++sample) {
 const double stamp_s = 0.1 * sample;
 shuttle_truth.dynamic_objects = {{{5.0, 5.5 - 0.05 * (sample - 20)},
 {0.0, -0.5}, 0.5}};
 process_shuttle_scan(stamp_s);
 }
 // A travelled lower extent is not a confirmed endpoint until motion turns
 // there. Observe the beginning of the second outbound leg before claiming a
 // closed periodic corridor.
 for (int sample = 41; sample <= 50; ++sample) {
 const double stamp_s = 0.1 * sample;
 shuttle_truth.dynamic_objects = {{{5.0, 4.5 + 0.05 * (sample - 40)},
 {0.0, 0.5}, 0.5}};
 process_shuttle_scan(stamp_s);
 }
 const auto learned = shuttle_perception.dynamic_obstacles().predict_inputs();
 CHECK(learned.size() == 1);
 CHECK(learned.front().bounded_y);
 const auto before_hide =
 shuttle_perception.dynamic_obstacles().active_tracks(5.0, 1.25);
 CHECK(before_hide.size() == 1);
 const auto shuttle_id = before_hide.front().object_id;

 shuttle_truth.dynamic_objects.clear();
for (int sample = 51; sample <= 74; ++sample)
 process_shuttle_scan(0.1 * sample);
CHECK(shuttle_perception.dynamic_obstacles().active_tracks(7.4, 1.25).empty());
 // From y=5.0 moving up, reflection at the learned upper endpoint sends the
 // target to y=4.75 at t=7.5 on its return toward the lower endpoint.
shuttle_truth.dynamic_objects = {{{5.0, 4.75}, {0.0, -0.5}, 0.5}};
process_shuttle_scan(7.5);
 const auto after_hide =
 shuttle_perception.dynamic_obstacles().active_tracks(7.5, 1.25);
 CHECK(after_hide.size() == 1);
 CHECK(after_hide.front().object_id == shuttle_id);
 CHECK(after_hide.front().motion_class != sim::TrackMotionClass::Unconfirmed);
 }
 return 0;
}
