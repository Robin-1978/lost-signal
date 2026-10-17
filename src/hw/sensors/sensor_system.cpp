#include "sensor_system.hpp"

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
namespace {

struct Cluster {
 std::vector<Vec2> points;
 double mean_range_m{0.0};
};

Vec2 lidar_return_point(const Scan& scan, Vec2 current_origin, double current_heading,
 double linear_velocity_mps, double angular_velocity_radps, int ray) {
 const double time_offset_s = scan.time_increment_s > 0.0
 ? -scan.scan_period_s + scan.time_increment_s * static_cast<double>(ray)
 : 0.0;
 const Vec2 sample_origin{
 current_origin.x + std::cos(current_heading) * linear_velocity_mps * time_offset_s,
 current_origin.y + std::sin(current_heading) * linear_velocity_mps * time_offset_s,
 };
 const double sample_heading = current_heading + angular_velocity_radps * time_offset_s;
 const double angle = sample_heading + scan.angle_min_rad +
 scan.angle_increment_rad * static_cast<double>(ray);
 return {
 sample_origin.x + std::cos(angle) * scan.ranges[ray],
 sample_origin.y + std::sin(angle) * scan.ranges[ray],
 };
}

double point_distance(Vec2 a, Vec2 b) {
 return std::hypot(a.x - b.x, a.y - b.y);
}

std::vector<Cluster> dynamic_clusters(const Scan& scan, const Scan& static_scan, Vec2 origin,
 double heading, double linear_velocity_mps, double angular_velocity_radps,
 double noise_sigma_m) {
 std::vector<Cluster> clusters;
 Cluster current;
 // Map differencing must tolerate pose-estimation error at wall edges. A
 // moving object needs a meaningful foreground gap; sub-35 cm residuals are
 // treated as map alignment noise and cannot create persistent ghost tracks.
 const double threshold = std::max(0.35, noise_sigma_m * 4.0);
 for (int ray = 0; ray < scan.n_rays; ++ray) {
  const bool dynamic =
    scan.hit[ray] && (!static_scan.hit[ray] || scan.ranges[ray] + threshold < static_scan.ranges[ray]);
  if (!dynamic) {
   if (!current.points.empty()) {
    clusters.push_back(std::move(current));
    current = {};
   }
   continue;
  }
 current.points.push_back(lidar_return_point(scan, origin, heading,
 linear_velocity_mps, angular_velocity_radps, ray));
  current.mean_range_m += scan.ranges[ray];
 }
 if (!current.points.empty()) {
  clusters.push_back(std::move(current));
 }
 if (clusters.size() > 1 && scan.n_rays > 0) {
  const bool first_is_dynamic =
    scan.hit.front() &&
    (!static_scan.hit.front() || scan.ranges.front() + threshold < static_scan.ranges.front());
  const bool last_is_dynamic = scan.hit.back() && (!static_scan.hit.back() || scan.ranges.back() + threshold <
                                                                                static_scan.ranges.back());
  if (first_is_dynamic && last_is_dynamic) {
   Cluster merged = std::move(clusters.back());
   clusters.pop_back();
   merged.points.insert(merged.points.end(), clusters.front().points.begin(), clusters.front().points.end());
   merged.mean_range_m += clusters.front().mean_range_m;
   clusters.front() = std::move(merged);
  }
 }
 return clusters;
}

DynamicObservation observation_from_cluster(const Cluster& cluster, Vec2 origin, int rays,
                                            Rng& perception_rng) {
 Vec2 arc;
 for (const auto point : cluster.points) {
  arc.x += point.x;
  arc.y += point.y;
 }
 const double count = static_cast<double>(cluster.points.size());
 arc.x /= count;
 arc.y /= count;
 const double mean_range = cluster.mean_range_m / count;
 const double angular_width = 2.0 * kPi * count / std::max(1, rays);
 const double geometric_radius = std::clamp(mean_range * std::sin(angular_width * 0.5), 0.45, 0.65);
 Vec2 sight{arc.x - origin.x, arc.y - origin.y};
 const double sight_length = std::max(1e-9, std::hypot(sight.x, sight.y));
 const double residual = 0.01 / std::sqrt(count);
 return {
   0,
   {
     arc.x + sight.x / sight_length * geometric_radius + perception_rng.gauss(residual),
     arc.y + sight.y / sight_length * geometric_radius + perception_rng.gauss(residual),
   },
   geometric_radius + 0.10,
 };
}

} // namespace

SensorSystem::SensorSystem(const GridMap& static_map, std::uint32_t noise_seed, std::uint32_t perception_seed)
    : static_map_(static_map), lidar_(std::make_unique<SimLidar>(static_map)), noise_rng_(noise_seed),
 perception_rng_(perception_seed), localization_rng_(perception_seed ^ 0xbb67ae85U),
 imu_gyro_rng_(noise_seed ^ 0x51ed2707U), imu_accel_rng_(noise_seed ^ 0xa54ff53aU),
 imu_bias_walk_rng_(noise_seed ^ 0x3c6ef372U) {}

void SensorSystem::reset(std::uint32_t noise_seed, std::uint32_t perception_seed) {
 noise_rng_ = Rng(noise_seed);
 perception_rng_ = Rng(perception_seed);
 localization_rng_ = Rng(perception_seed ^ 0xbb67ae85U);
 imu_gyro_rng_ = Rng(noise_seed ^ 0x51ed2707U);
 imu_accel_rng_ = Rng(noise_seed ^ 0xa54ff53aU);
 imu_bias_walk_rng_ = Rng(noise_seed ^ 0x3c6ef372U);
 dynamic_obstacles_.clear();
 pending_.clear();
 next_track_id_ = 0;
 lidar_sequence_ = 0;
 pose_sequence_ = 0;
 imu_sequence_ = 0;
 imu_bias_initialized_ = false;
 imu_roll_bias_radps_ = 0.0;
 imu_pitch_bias_radps_ = 0.0;
 imu_yaw_bias_radps_ = 0.0;
 last_config_ = {};
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
 config.latency_s, 1, "sim.imu"};
 return observation;
}

ObservationFrame SensorSystem::capture_lidar(const WorldTruth& truth, double time_s,
                                             const LidarConfig& config) {
 return capture_lidar(truth, time_s, config, false);
}

ObservationFrame SensorSystem::capture_lidar(const WorldTruth& truth, double time_s,
                                             const LidarConfig& config, bool include_pose_observation) {
 validate_lidar_config(config);
 last_config_ = config;
 ObservationFrame frame;
 frame.time_s = time_s;
 const double cosine = std::cos(truth.robot.theta);
 const double sine = std::sin(truth.robot.theta);
 const Vec2 sensor_origin{
 truth.robot.x + cosine * config.position_in_base.x - sine * config.position_in_base.y,
 truth.robot.y + sine * config.position_in_base.x + cosine * config.position_in_base.y,
 };
 frame.lidar = apply_lidar_noise(
  lidar_->scan(sensor_origin, truth.robot.theta + config.yaw_in_base_rad,
   config.rays, config.range_m, truth.dynamic_objects),
  config.noise, noise_rng_);
 apply_lidar_range_limits(*frame.lidar, config.min_range_m);
 frame.lidar->scan_period_s = 1.0 / config.scan_hz;
 frame.lidar->header = {time_s, lidar_sequence_++, config.frame_id, config.latency_s, 1, "sim.lidar"};
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
 {time_s, pose_sequence_++, "map", 0.0, 1, "sim.localization"},
  };
 }
 return frame;
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
 last_config_ = config;
 ObservationFrame frame;
 frame.time_s = time_s;
 frame.lidar = apply_lidar_noise(raycast_truth.combined, config.noise, noise_rng_);
 frame.lidar_static_reference = raycast_truth.static_environment;
 apply_lidar_range_limits(*frame.lidar, config.min_range_m);
 apply_lidar_range_limits(*frame.lidar_static_reference, config.min_range_m);
 frame.lidar->scan_period_s = 1.0 / config.scan_hz;
 frame.lidar_static_reference->scan_period_s = 1.0 / config.scan_hz;
 const MeasurementHeader header{time_s, lidar_sequence_++, config.frame_id, config.latency_s, 1, "sim.lidar"};
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
 {time_s, pose_sequence_++, "map", 0.0, 1, "sim.localization"},
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
 {time_s, pose_sequence_++, "map", 0.0, 1, "sim.localization"},
 };
}

PerceptionFrame SensorSystem::process_lidar(const ObservationFrame& observation,
                                            const EstimatedState& estimate, double track_max_age_s) {
 if (!observation.lidar || last_config_.rays <= 0) {
  throw std::invalid_argument("perception update requires a captured lidar frame");
 }
 const Scan& scan = *observation.lidar;
 if (scan.header.frame_id.empty() || std::abs(scan.header.stamp_s - observation.time_s) > 1e-9) {
 throw std::invalid_argument("lidar measurement header does not match observation frame");
 }
 const double cosine = std::cos(estimate.heading_rad);
 const double sine = std::sin(estimate.heading_rad);
 const Vec2 sensor_origin{
 estimate.position.x + cosine * last_config_.position_in_base.x -
 sine * last_config_.position_in_base.y,
 estimate.position.y + sine * last_config_.position_in_base.x +
 cosine * last_config_.position_in_base.y,
 };
 const double sensor_heading = estimate.heading_rad + last_config_.yaw_in_base_rad;
 Scan static_scan;
 if (observation.lidar_static_reference) {
 static_scan = *observation.lidar_static_reference;
 if (static_scan.n_rays != scan.n_rays || static_scan.ranges.size() != scan.ranges.size() ||
 static_scan.hit.size() != scan.hit.size()) {
 throw std::invalid_argument("static lidar reference does not match captured scan");
 }
 } else {
  static_scan = lidar_raycast(static_map_, sensor_origin, sensor_heading, scan.n_rays,
   scan.max_range);
 }
 PerceptionFrame output;
 for (int ray = 0; ray < scan.n_rays; ++ray) {
  if (!scan.hit[ray]) {
   continue;
  }
 output.lidar_hits.push_back(lidar_return_point(scan, sensor_origin, sensor_heading,
 estimate.linear_velocity_mps, estimate.angular_velocity_radps, ray));
  if (scan.ranges[ray] <= std::min(scan.max_range, 2.5)) {
   output.static_obstacle_near = true;
  }
 }

 const auto clusters =
 dynamic_clusters(scan, static_scan, sensor_origin, sensor_heading,
 estimate.linear_velocity_mps, estimate.angular_velocity_radps,
  last_config_.noise.range_stddev_m);
 auto tracks = dynamic_obstacles_.active_tracks(observation.time_s, track_max_age_s);
 std::vector<bool> used_tracks(tracks.size(), false);
 std::vector<bool> used_pending(pending_.size(), false);
 for (const auto& cluster : clusters) {
  DynamicObservation detected =
 observation_from_cluster(cluster, sensor_origin, scan.n_rays, perception_rng_);
  std::size_t match = tracks.size();
  double best_distance = 1.2;
  for (std::size_t index = 0; index < tracks.size(); ++index) {
   if (used_tracks[index]) {
    continue;
   }
   const double candidate_distance = point_distance(detected.position, tracks[index].state.p);
   if (candidate_distance < best_distance) {
    best_distance = candidate_distance;
    match = index;
   }
  }
  if (match < tracks.size()) {
   used_tracks[match] = true;
   detected.object_id = tracks[match].object_id;
   dynamic_obstacles_.observe(detected, observation.time_s);
   continue;
  }

  std::size_t pending_match = pending_.size();
  best_distance = 1.0;
  for (std::size_t index = 0; index < pending_.size(); ++index) {
   if (used_pending[index]) {
    continue;
   }
   const double candidate_distance = point_distance(detected.position, pending_[index].position);
   if (candidate_distance < best_distance) {
    best_distance = candidate_distance;
    pending_match = index;
   }
  }
  if (pending_match == pending_.size()) {
   pending_.push_back({next_track_id_++, detected.position, detected.radius, observation.time_s, 1});
   used_pending.push_back(true);
   continue;
  }
  used_pending[pending_match] = true;
  auto& pending = pending_[pending_match];
  pending.position = detected.position;
  pending.radius_m = detected.radius;
  pending.last_seen_s = observation.time_s;
  ++pending.confirmations;
  if (pending.confirmations >= 2) {
   detected.object_id = pending.id;
   dynamic_obstacles_.observe(detected, observation.time_s);
  }
 }
 pending_.erase(
   std::remove_if(pending_.begin(), pending_.end(),
                  [&](const auto& pending) { return observation.time_s - pending.last_seen_s > 0.5; }),
   pending_.end());
 output.dynamic_object_detected =
   !dynamic_obstacles_.active_tracks(observation.time_s, track_max_age_s).empty();
 return output;
}

} // namespace sim
