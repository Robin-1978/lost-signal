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

} // namespace

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
 pose_sequence_ = 0;
 imu_sequence_ = 0;
 imu_bias_initialized_ = false;
 imu_roll_bias_radps_ = 0.0;
 imu_pitch_bias_radps_ = 0.0;
 imu_yaw_bias_radps_ = 0.0;
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
