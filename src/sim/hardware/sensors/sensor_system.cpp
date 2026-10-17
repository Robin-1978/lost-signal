#include "sim/hardware/sensors/sensor_system.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace sim {
namespace {

void apply_lidar_range_limits(Scan& scan, double min_range_m) {
 for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
 if (!scan.hit[index] || scan.ranges[index] >= min_range_m)
 continue;
 // The device cannot distinguish a reliable surface inside its optical
 // near-field. Publish the same invalid/no-return representation as a miss;
 // never clamp it to min_range and invent a valid obstacle there.
 scan.hit[index] = 0;
 scan.ranges[index] = scan.max_range;
 if (!scan.intensities.empty())
 scan.intensities[index] = 0.0;
 }
}

void validate_lidar_config(const LidarConfig& config) {
 if (config.rays <= 0 || config.range_m <= 0.0 || config.min_range_m < 0.0 ||
 config.min_range_m >= config.range_m || config.scan_hz <= 0.0 ||
 config.latency_s < 0.0 || config.noise.range_stddev_m < 0.0) {
 throw std::invalid_argument("invalid lidar configuration");
 }
}

void apply_point_cloud_noise(LidarPointCloud& cloud, double min_range_m,
 const LidarNoiseModel& model, Rng& rng) {
 std::vector<LidarPointReturn> filtered;
 filtered.reserve(cloud.points.size());
 for (auto point : cloud.points) {
 const double range = std::sqrt(point.x_m * point.x_m + point.y_m * point.y_m +
 point.z_m * point.z_m);
 if (!(range >= min_range_m) || range > cloud.max_range_m ||
 rng.uniform01() < model.dropout_probability)
 continue;
 double measured = range + model.range_bias_m + rng.gauss(model.range_stddev_m);
 if (model.range_resolution_m > 0.0)
 measured = std::round(measured / model.range_resolution_m) * model.range_resolution_m;
 if (!(measured >= min_range_m) || measured > cloud.max_range_m)
 continue;
 const double scale = measured / std::max(1e-12, range);
 point.x_m *= scale;
 point.y_m *= scale;
 point.z_m *= scale;
 filtered.push_back(point);
 }
 cloud.points = std::move(filtered);
}

} // namespace

Scan project_point_cloud_to_planar(
 const LidarPointCloud& cloud, const PointCloudProjectionConfig& config) {
 if (cloud.azimuth_samples == 0 || cloud.rings == 0 ||
 !(cloud.max_range_m > 0.0) ||
 config.min_obstacle_height_m >= config.max_obstacle_height_m ||
 !std::isfinite(config.ground_seed_max_height_m) ||
 !std::isfinite(config.ground_max_slope_rad) ||
 config.ground_max_slope_rad < 0.0 ||
 config.ground_max_slope_rad >= 0.5 * kPi) {
 throw std::invalid_argument("invalid point-cloud projection configuration");
 }
 Scan scan;
 scan.n_rays = static_cast<int>(cloud.azimuth_samples);
 scan.max_range = cloud.max_range_m;
 scan.angle_increment_rad = 2.0 * kPi / static_cast<double>(cloud.azimuth_samples);
 scan.scan_period_s = cloud.scan_period_s;
 scan.time_increment_s = cloud.scan_period_s / static_cast<double>(cloud.azimuth_samples);
 scan.header = cloud.header;
 scan.header.source_id = "sim.lidar";
 scan.ranges.assign(cloud.azimuth_samples, cloud.max_range_m);
 scan.hit.assign(cloud.azimuth_samples, 0);
 scan.intensities.assign(cloud.azimuth_samples, 0.0);
 std::vector<std::vector<std::size_t>> returns_by_azimuth(cloud.azimuth_samples);
 for (std::size_t index = 0; index < cloud.points.size(); ++index) {
 const auto& point = cloud.points[index];
 double angle = std::atan2(point.y_m, point.x_m);
 if (angle < 0.0)
 angle += 2.0 * kPi;
 const auto azimuth = static_cast<std::size_t>(std::llround(
 angle / (2.0 * kPi) * static_cast<double>(cloud.azimuth_samples))) %
 cloud.azimuth_samples;
 returns_by_azimuth[azimuth].push_back(index);
 }

 std::vector<char> ground(cloud.points.size(), 0);
 for (auto& azimuth_returns : returns_by_azimuth) {
 std::sort(azimuth_returns.begin(), azimuth_returns.end(),
 [&](std::size_t left, std::size_t right) {
 const auto elevation = [&](std::size_t index) {
 const auto& point = cloud.points[index];
 return std::atan2(point.z_m, std::hypot(point.x_m, point.y_m));
 };
 const double left_elevation = elevation(left);
 const double right_elevation = elevation(right);
 if (left_elevation != right_elevation)
 return left_elevation < right_elevation;
 return cloud.points[left].ring < cloud.points[right].ring;
 });
 if (azimuth_returns.empty())
 continue;
 const auto first = azimuth_returns.front();
 if (cloud.points[first].z_m <= config.ground_seed_max_height_m)
 ground[first] = 1;
 for (std::size_t offset = 1; offset < azimuth_returns.size(); ++offset) {
 const auto previous = azimuth_returns[offset - 1];
 const auto current = azimuth_returns[offset];
 if (!ground[previous])
 continue;
 const double previous_range = std::hypot(
 cloud.points[previous].x_m, cloud.points[previous].y_m);
 const double current_range = std::hypot(
 cloud.points[current].x_m, cloud.points[current].y_m);
 const double horizontal_delta = current_range - previous_range;
 const double vertical_delta = cloud.points[current].z_m - cloud.points[previous].z_m;
 const double surface_slope = std::abs(std::atan2(
 vertical_delta, std::abs(horizontal_delta)));
 if (surface_slope <= config.ground_max_slope_rad)
 ground[current] = 1;
 }
 }

 for (std::size_t index = 0; index < cloud.points.size(); ++index) {
 const auto& point = cloud.points[index];
 if (ground[index] || point.z_m < config.min_obstacle_height_m ||
 point.z_m > config.max_obstacle_height_m)
 continue;
 const double planar_range = std::hypot(point.x_m, point.y_m);
 if (!(planar_range > 0.0) || planar_range > cloud.max_range_m)
 continue;
 double angle = std::atan2(point.y_m, point.x_m);
 if (angle < 0.0)
 angle += 2.0 * kPi;
 const auto azimuth = static_cast<std::size_t>(std::llround(
 angle / (2.0 * kPi) * static_cast<double>(cloud.azimuth_samples))) %
 cloud.azimuth_samples;
 if (planar_range < scan.ranges[azimuth]) {
 scan.ranges[azimuth] = planar_range;
 scan.hit[azimuth] = 1;
 scan.intensities[azimuth] = point.intensity;
 }
 }
 return scan;
}

SensorSystem::SensorSystem(std::uint32_t noise_seed, std::uint32_t localization_seed)
 : noise_rng_(noise_seed),
 localization_rng_(localization_seed ^ 0xbb67ae85U),
 imu_gyro_rng_(noise_seed ^ 0x51ed2707U), imu_accel_rng_(noise_seed ^ 0xa54ff53aU),
 imu_bias_walk_rng_(noise_seed ^ 0x3c6ef372U) {}

void SensorSystem::reset(std::uint32_t noise_seed, std::uint32_t localization_seed) {
 noise_rng_ = Rng(noise_seed);
 localization_rng_ = Rng(localization_seed ^ 0xbb67ae85U);
 imu_gyro_rng_ = Rng(noise_seed ^ 0x51ed2707U);
 imu_accel_rng_ = Rng(noise_seed ^ 0xa54ff53aU);
 imu_bias_walk_rng_ = Rng(noise_seed ^ 0x3c6ef372U);
 lidar_sequence_ = 0;
 point_cloud_sequence_ = 0;
 pose_sequence_ = 0;
 imu_sequence_ = 0;
 imu_bias_initialized_ = false;
 imu_roll_bias_radps_ = 0.0;
 imu_pitch_bias_radps_ = 0.0;
 imu_yaw_bias_radps_ = 0.0;
}

ObservationFrame SensorSystem::capture_lidar_point_cloud(
 const WorldTruth& truth, double time_s, const LidarConfig& config,
 const LidarPointCloudRaycastTruth& raycast_truth,
 bool include_pose_observation) {
 validate_lidar_config(config);
 const auto valid_cloud = [&](const LidarPointCloud& cloud) {
 return cloud.pattern == LidarPatternKind::MultiBeamSpinning &&
 cloud.rings == config.vertical_angles_rad.size() &&
 cloud.azimuth_samples == static_cast<std::uint32_t>(config.rays) &&
 std::abs(cloud.max_range_m - config.range_m) < 1e-9;
 };
 if (config.vertical_angles_rad.size() < 2 ||
 !valid_cloud(raycast_truth.combined) ||
 !valid_cloud(raycast_truth.static_environment)) {
 throw std::invalid_argument("multibeam raycast truth does not match sensor configuration");
 }
 ObservationFrame frame;
 frame.time_s = time_s;
 frame.lidar_point_cloud = raycast_truth.combined;
 frame.lidar_point_cloud->header = {
 time_s, point_cloud_sequence_++, config.frame_id, config.latency_s,
 kMeasurementSchemaVersion, "sim.lidar-point-cloud"};
 apply_point_cloud_noise(*frame.lidar_point_cloud, config.min_range_m,
 config.noise, noise_rng_);
 auto static_cloud = raycast_truth.static_environment;
 static_cloud.header = frame.lidar_point_cloud->header;
 frame.lidar = project_point_cloud_to_planar(
 *frame.lidar_point_cloud,
 {config.projection_min_height_m, config.projection_max_height_m,
 config.projection_ground_seed_max_height_m,
 config.projection_ground_max_slope_rad});
 frame.lidar_static_reference = project_point_cloud_to_planar(
 static_cloud,
 {config.projection_min_height_m, config.projection_max_height_m,
 config.projection_ground_seed_max_height_m,
 config.projection_ground_max_slope_rad});
 const MeasurementHeader planar_header{
 time_s, lidar_sequence_++, config.frame_id, config.latency_s,
 kMeasurementSchemaVersion, "sim.lidar"};
 frame.lidar->header = planar_header;
 frame.lidar_static_reference->header = planar_header;
 if (include_pose_observation)
 frame.absolute_pose = capture_pose(truth, time_s);
 return frame;
}

ImuObservation SensorSystem::capture_imu(const ImuTruth& truth, double time_s,
 double sample_period_s, const ImuConfig& config) {
 if (!std::isfinite(time_s) || time_s < 0.0 || !std::isfinite(sample_period_s) ||
 sample_period_s <= 0.0 || config.frame_id.empty() || config.latency_s < 0.0 ||
 config.gyro_noise_radps < 0.0 || config.gyro_bias_stddev_radps < 0.0 ||
 config.gyro_bias_random_walk_radps_sqrt_s < 0.0 || config.accel_noise_mps2 < 0.0) {
 throw std::invalid_argument("invalid imu configuration or sample time");
 }
 if (!imu_bias_initialized_) {
 imu_roll_bias_radps_ = imu_gyro_rng_.gauss(config.gyro_bias_stddev_radps);
 imu_pitch_bias_radps_ = imu_gyro_rng_.gauss(config.gyro_bias_stddev_radps);
 imu_yaw_bias_radps_ = imu_gyro_rng_.gauss(config.gyro_bias_stddev_radps);
 imu_bias_initialized_ = true;
 }
 const double walk_sigma =
 config.gyro_bias_random_walk_radps_sqrt_s * std::sqrt(sample_period_s);
 if (walk_sigma > 0.0) {
 imu_roll_bias_radps_ += imu_bias_walk_rng_.gauss(walk_sigma);
 imu_pitch_bias_radps_ += imu_bias_walk_rng_.gauss(walk_sigma);
 imu_yaw_bias_radps_ += imu_bias_walk_rng_.gauss(walk_sigma);
 }
 ImuObservation observation;
 observation.roll_rate_radps = truth.roll_rate_radps + imu_roll_bias_radps_ +
 imu_gyro_rng_.gauss(config.gyro_noise_radps);
 observation.pitch_rate_radps = truth.pitch_rate_radps + imu_pitch_bias_radps_ +
 imu_gyro_rng_.gauss(config.gyro_noise_radps);
 observation.yaw_rate_radps = truth.yaw_rate_radps + imu_yaw_bias_radps_ +
 imu_gyro_rng_.gauss(config.gyro_noise_radps);
 observation.forward_acceleration_mps2 = truth.forward_specific_force_mps2 +
 imu_accel_rng_.gauss(config.accel_noise_mps2);
 observation.left_acceleration_mps2 = truth.left_specific_force_mps2 +
 imu_accel_rng_.gauss(config.accel_noise_mps2);
 observation.up_acceleration_mps2 = truth.up_specific_force_mps2 +
 imu_accel_rng_.gauss(config.accel_noise_mps2);
 observation.noise_sigma_radps = config.gyro_noise_radps;
 observation.acceleration_noise_sigma_mps2 = config.accel_noise_mps2;
 observation.header = {time_s, imu_sequence_++, config.frame_id,
 config.latency_s, kMeasurementSchemaVersion, "sim.imu"};
 return observation;
}

ObservationFrame SensorSystem::capture_lidar(const WorldTruth& truth, double time_s,
 const LidarConfig& config, const LidarRaycastTruth& raycast_truth,
 bool include_pose_observation) {
 validate_lidar_config(config);
 const auto valid_truth_scan = [&](const Scan& scan) {
 return scan.n_rays == config.rays && scan.ranges.size() == static_cast<std::size_t>(config.rays) &&
 scan.hit.size() == static_cast<std::size_t>(config.rays) &&
 std::abs(scan.max_range - config.range_m) < 1e-9;
 };
 if (!valid_truth_scan(raycast_truth.combined) ||
 !valid_truth_scan(raycast_truth.static_environment)) {
 throw std::invalid_argument("lidar raycast truth does not match sensor configuration");
 }
 ObservationFrame frame;
 frame.time_s = time_s;
 frame.lidar = apply_lidar_noise(raycast_truth.combined, config.noise, noise_rng_);
 frame.lidar_static_reference = raycast_truth.static_environment;
 apply_lidar_range_limits(*frame.lidar, config.min_range_m);
 apply_lidar_range_limits(*frame.lidar_static_reference, config.min_range_m);
 frame.lidar->scan_period_s = 1.0 / config.scan_hz;
 frame.lidar_static_reference->scan_period_s = 1.0 / config.scan_hz;
 const MeasurementHeader header{time_s, lidar_sequence_++, config.frame_id, config.latency_s, kMeasurementSchemaVersion, "sim.lidar"};
 frame.lidar->header = header;
 frame.lidar_static_reference->header = header;
 if (include_pose_observation) {
  const double position_sigma = std::max(0.01, config.noise.range_stddev_m * 1.5);
 constexpr double heading_sigma = 0.006;
 frame.absolute_pose = PoseObservation{
 {
 truth.robot.x + localization_rng_.gauss(position_sigma),
 truth.robot.y + localization_rng_.gauss(position_sigma),
 },
 truth.robot.theta + localization_rng_.gauss(heading_sigma),
 position_sigma,
 heading_sigma,
 {time_s, pose_sequence_++, "map", 0.0, kMeasurementSchemaVersion, "sim.localization"},
 };
 }
 return frame;
}

PoseObservation SensorSystem::capture_pose(const WorldTruth& truth, double time_s,
 double position_sigma_m,
                                           double heading_sigma_rad) {
 return {
   {
     truth.robot.x + localization_rng_.gauss(position_sigma_m),
     truth.robot.y + localization_rng_.gauss(position_sigma_m),
   },
   truth.robot.theta + localization_rng_.gauss(heading_sigma_rad),
   position_sigma_m,
   heading_sigma_rad,
 {time_s, pose_sequence_++, "map", 0.0, kMeasurementSchemaVersion, "sim.localization"},
 };
}

} // namespace sim
