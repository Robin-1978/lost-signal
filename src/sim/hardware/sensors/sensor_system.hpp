#pragma once

#include "sim/hardware/sensors/sensor.hpp"
#include "sim/hardware/sensors/lidar_truth.hpp"
#include "sim/hardware/world_truth.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace sim {

struct LidarConfig {
 int rays{0};
 double min_range_m{0.0};
 double range_m{0.0};
 double scan_hz{0.0};
 LidarNoiseModel noise;
 LidarReturnModel returns;
 std::string frame_id{"lidar_link"};
 double latency_s{0.0};
 Vec2 position_in_base;
 double yaw_in_base_rad{0.0};
 double height_in_base_m{0.0};
 double roll_in_base_rad{0.0};
 double pitch_in_base_rad{0.0};
 std::vector<double> vertical_angles_rad;
 double projection_min_height_m{-0.20};
 double projection_max_height_m{1.50};
 double projection_ground_seed_max_height_m{-0.25};
 double projection_ground_max_slope_rad{30.0 * kPi / 180.0};
};

struct PointCloudProjectionConfig {
 double min_obstacle_height_m{-0.20};
 double max_obstacle_height_m{1.50};
 double ground_seed_max_height_m{-0.25};
 double ground_max_slope_rad{30.0 * kPi / 180.0};
};

Scan project_point_cloud_to_planar(
 const LidarPointCloud& cloud, const PointCloudProjectionConfig& config);

// Ideal rigid-body input to a simulated IMU. Values are already evaluated at
// the calibrated imu_link origin and expressed in imu_link; the device model
// below owns bias, random walk, white noise, timing and sequence numbers.
struct ImuTruth {
 double roll_rate_radps{0.0};
 double pitch_rate_radps{0.0};
 double yaw_rate_radps{0.0};
 double forward_specific_force_mps2{0.0};
 double left_specific_force_mps2{0.0};
 double up_specific_force_mps2{9.81};
};

struct ImuConfig {
 double gyro_noise_radps{0.0};
 double gyro_bias_stddev_radps{0.0};
 double gyro_bias_random_walk_radps_sqrt_s{0.0};
 double accel_noise_mps2{0.0};
 std::string frame_id{"imu_link"};
 double latency_s{0.0};
};

class SensorSystem {
 public:
 SensorSystem(std::uint32_t noise_seed, std::uint32_t localization_seed);

 void reset(std::uint32_t noise_seed, std::uint32_t localization_seed);
 ObservationFrame capture_lidar(const WorldTruth& truth, double time_s, const LidarConfig& config,
 const LidarRaycastTruth& raycast_truth, bool include_pose_observation = false);
 ObservationFrame capture_lidar_point_cloud(
 const WorldTruth& truth, double time_s, const LidarConfig& config,
 const LidarPointCloudRaycastTruth& raycast_truth,
 bool include_pose_observation = false);
 PoseObservation capture_pose(const WorldTruth& truth, double time_s,
 double position_sigma_m = 0.01,
                              double heading_sigma_rad = 0.006);
 ImuObservation capture_imu(const ImuTruth& truth, double time_s,
 double sample_period_s, const ImuConfig& config);
 private:
 Rng noise_rng_;
 Rng localization_rng_;
 Rng imu_gyro_rng_;
 Rng imu_accel_rng_;
 Rng imu_bias_walk_rng_;
 std::uint64_t lidar_sequence_{0};
 std::uint64_t point_cloud_sequence_{0};
 std::uint64_t pose_sequence_{0};
 std::uint64_t imu_sequence_{0};
 bool imu_bias_initialized_{false};
 double imu_roll_bias_radps_{0.0};
 double imu_pitch_bias_radps_{0.0};
 double imu_yaw_bias_radps_{0.0};
};

} // namespace sim
