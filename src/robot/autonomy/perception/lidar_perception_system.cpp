#include "robot/autonomy/perception/lidar_perception_system.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace sim {
namespace {

constexpr double kTrackRadiusPaddingM = 0.10;

struct Cluster {
 std::vector<Vec2> points;
 double mean_range_m{0.0};
};

struct Vector3 {
 double x{0.0};
 double y{0.0};
 double z{0.0};
};

struct ProjectedReturn {
 ObservedRay2D ray;
 double endpoint_height_m{0.0};
};

Vector3 rotate_zyx(Vector3 value, double roll, double pitch, double yaw) {
 const double cr = std::cos(roll);
 const double sr = std::sin(roll);
 const double cp = std::cos(pitch);
 const double sp = std::sin(pitch);
 const double cy = std::cos(yaw);
 const double sy = std::sin(yaw);
 return {
 cy * cp * value.x + (cy * sp * sr - sy * cr) * value.y +
 (cy * sp * cr + sy * sr) * value.z,
 sy * cp * value.x + (sy * sp * sr + cy * cr) * value.y +
 (sy * sp * cr - cy * sr) * value.z,
 -sp * value.x + cp * sr * value.y + cp * cr * value.z,
 };
}

ProjectedReturn project_ray(const Scan& scan, const LidarProjectionState& state,
 const LidarProjectionConfig& config, int ray) {
 const double time_offset_s = scan.time_increment_s > 0.0
 ? -scan.scan_period_s + scan.time_increment_s * static_cast<double>(ray)
 : 0.0;
 const double sample_heading = state.base_heading_rad +
 state.angular_velocity_radps * time_offset_s;
 const Vec2 sample_origin{
 state.base_position.x + (std::cos(state.base_heading_rad) * state.linear_velocity_mps -
 std::sin(state.base_heading_rad) * state.lateral_velocity_mps) * time_offset_s,
 state.base_position.y + (std::sin(state.base_heading_rad) * state.linear_velocity_mps +
 std::cos(state.base_heading_rad) * state.lateral_velocity_mps) * time_offset_s,
 };
 const double angle = scan.angle_min_rad +
 scan.angle_increment_rad * static_cast<double>(ray);
 const double roll = state.attitude_valid ? state.base_roll_rad : 0.0;
 const double pitch = state.attitude_valid ? state.base_pitch_rad : 0.0;
 const auto mount_offset = rotate_zyx(
 {config.position_in_base.x, config.position_in_base.y,
 config.height_in_base_m}, roll, pitch, sample_heading);
 const auto direction_in_base = rotate_zyx(
 {std::cos(angle), std::sin(angle), 0.0},
 config.roll_in_base_rad, config.pitch_in_base_rad,
 config.yaw_in_base_rad);
 const auto direction = rotate_zyx(
 direction_in_base, roll, pitch, sample_heading);
 const Vector3 sensor_origin{
 sample_origin.x + mount_offset.x,
 sample_origin.y + mount_offset.y,
 state.base_height_m + mount_offset.z,
 };
 return {
 {
 {sensor_origin.x, sensor_origin.y},
 {sensor_origin.x + direction.x * scan.ranges[ray],
 sensor_origin.y + direction.y * scan.ranges[ray]},
 scan.hit[ray] != 0,
 },
 sensor_origin.z + direction.z * scan.ranges[ray],
 };
}

double point_distance(Vec2 a, Vec2 b) {
 return std::hypot(a.x - b.x, a.y - b.y);
}

std::vector<Cluster> dynamic_clusters(const Scan& scan,
 const std::vector<ObservedRay2D>& projected,
 const std::vector<char>& foreground) {
 std::vector<Cluster> clusters;
 Cluster current;
 for (int ray = 0; ray < scan.n_rays; ++ray) {
 const bool dynamic = foreground[static_cast<std::size_t>(ray)] != 0;
 if (!dynamic) {
 if (!current.points.empty()) {
 clusters.push_back(std::move(current));
 current = {};
 }
 continue;
 }
 current.points.push_back(projected[static_cast<std::size_t>(ray)].endpoint);
 current.mean_range_m += scan.ranges[ray];
 }
 if (!current.points.empty())
 clusters.push_back(std::move(current));

 if (clusters.size() > 1 && scan.n_rays > 0) {
 const bool first_dynamic = foreground.front() != 0;
 const bool last_dynamic = foreground.back() != 0;
 if (first_dynamic && last_dynamic) {
 Cluster merged = std::move(clusters.back());
 clusters.pop_back();
 merged.points.insert(merged.points.end(), clusters.front().points.begin(), clusters.front().points.end());
 merged.mean_range_m += clusters.front().mean_range_m;
 clusters.front() = std::move(merged);
 }
 }
 return clusters;
}

bool is_unmapped_return(const Scan& scan, const Scan& static_scan, int ray,
 double noise_sigma_m, double map_resolution_m,
 double map_alignment_tolerance_m) {
 if (!scan.hit[ray])
 return false;
 // A foreground endpoint must exceed the localization/map-alignment envelope
 // before becoming a second obstacle beside an authored wall. Half a source
 // cell plus measured range noise covers quantization. The localization
 // system supplies the remaining alignment envelope explicitly.
 const double residual_tolerance_m =
 std::max(map_alignment_tolerance_m,
 0.5 * map_resolution_m + 4.0 * noise_sigma_m);
 // A small pose/attitude error moves a thin static feature into a neighboring
 // azimuth even when its range remains correct. Comparing only equal ray
 // indices turns tree trunks and shelf edges into high-speed phantom tracks.
 // Search the angular envelope implied by the localization tolerance and use
 // the nearest known static return as the correspondence. A true foreground
 // object remains closer than every static candidate by the range envelope.
 const double angular_tolerance_rad = std::min(
 8.0 * kPi / 180.0,
 std::atan2(map_alignment_tolerance_m,
 std::max(0.5, scan.ranges[ray])));
 const int neighbor_count = std::min(
 scan.n_rays / 2,
 static_cast<int>(std::ceil(angular_tolerance_rad /
 std::max(1e-9, std::abs(scan.angle_increment_rad)))));
 double nearest_static_range = static_scan.max_range;
 bool static_candidate = false;
 for (int offset = -neighbor_count; offset <= neighbor_count; ++offset) {
 int candidate = (ray + offset) % scan.n_rays;
 if (candidate < 0)
 candidate += scan.n_rays;
 if (!static_scan.hit[static_cast<std::size_t>(candidate)])
 continue;
 static_candidate = true;
 nearest_static_range = std::min(
 nearest_static_range,
 static_scan.ranges[static_cast<std::size_t>(candidate)]);
 }
 return !static_candidate ||
 scan.ranges[ray] + residual_tolerance_m < nearest_static_range;
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
 {arc.x + sight.x / sight_length * geometric_radius + perception_rng.gauss(residual),
 arc.y + sight.y / sight_length * geometric_radius + perception_rng.gauss(residual)},
 geometric_radius + kTrackRadiusPaddingM,
 };
}

void validate_scan_pair(const Scan& scan, const Scan& reference) {
 const auto valid_shape = [](const Scan& value) {
 return value.n_rays > 0 && value.ranges.size() == static_cast<std::size_t>(value.n_rays) &&
 value.hit.size() == static_cast<std::size_t>(value.n_rays);
 };
 if (!valid_shape(scan) || !valid_shape(reference) || reference.n_rays != scan.n_rays ||
 reference.ranges.size() != scan.ranges.size() || reference.hit.size() != scan.hit.size()) {
 throw std::invalid_argument("lidar scan and static reference geometry do not match");
 }
 if (scan.header.frame_id.empty())
 throw std::invalid_argument("lidar scan requires a calibrated frame id");
}

} // namespace

LidarPerceptionSystem::LidarPerceptionSystem(const GridMap& static_map, std::uint32_t seed)
 : static_map_(static_map), perception_rng_(seed) {}

void LidarPerceptionSystem::reset(std::uint32_t seed) {
 perception_rng_ = Rng(seed);
 dynamic_obstacles_.clear();
 pending_.clear();
 next_track_id_ = 0;
}

LidarPerceptionFrame LidarPerceptionSystem::process(const Scan& scan, const Scan& static_reference,
 const LidarProjectionState& state, const LidarProjectionConfig& config,
 double track_max_age_s) {
 validate_scan_pair(scan, static_reference);
 LidarPerceptionFrame output;
 output.obstacle_rays.reserve(static_cast<std::size_t>(scan.n_rays));
 const double roll = state.attitude_valid ? state.base_roll_rad : 0.0;
 const double pitch = state.attitude_valid ? state.base_pitch_rad : 0.0;
 const auto current_mount_offset = rotate_zyx(
 {config.position_in_base.x, config.position_in_base.y,
 config.height_in_base_m}, roll, pitch, state.base_heading_rad);
 const Vec2 sensor_origin{
 state.base_position.x + current_mount_offset.x,
 state.base_position.y + current_mount_offset.y,
 };
 std::vector<char> foreground(static_cast<std::size_t>(scan.n_rays), 0);
 for (int ray = 0; ray < scan.n_rays; ++ray) {
 auto projected = project_ray(scan, state, config, ray);
 bool ground_return = false;
 if (state.attitude_valid && scan.hit[ray]) {
 const auto ground_height = static_map_.elevation(
 projected.ray.endpoint.x, projected.ray.endpoint.y);
 ground_return = ground_height &&
 std::abs(projected.endpoint_height_m - *ground_height) <=
 config.ground_alignment_tolerance_m;
 }
 // Known static geometry is already owned and inflated by the immutable map
 // layer. Mark only foreground endpoints in the transient observation layer;
 // every beam still participates in clearing that layer.
 projected.ray.hit = !ground_return && is_unmapped_return(
 scan, static_reference, ray, config.range_noise_sigma_m, static_map_.res(),
 config.map_alignment_tolerance_m);
 foreground[static_cast<std::size_t>(ray)] = projected.ray.hit ? 1 : 0;
 output.obstacle_rays.push_back(projected.ray);
 if (!scan.hit[ray])
 continue;
 output.hits.push_back(output.obstacle_rays.back().endpoint);
 if (!ground_return && scan.ranges[ray] <= std::min(scan.max_range, 2.5))
 output.static_obstacle_near = true;
 }

 const auto clusters = dynamic_clusters(scan, output.obstacle_rays, foreground);
 auto tracks = dynamic_obstacles_.active_tracks(scan.header.stamp_s, track_max_age_s);
 std::vector<bool> used_tracks(tracks.size(), false);
 std::vector<bool> used_pending(pending_.size(), false);
 const auto static_alignment_mask = static_map_.inflated(
 config.map_alignment_tolerance_m + 0.5 * static_map_.res());
 for (const auto& cluster : clusters) {
 // Associate raw endpoints with the static map before estimating an object
 // centre. The centre reconstruction deliberately projects beyond the visible
 // surface by an estimated radius; using that synthetic point for map
 // association can move a tree trunk outside its own occupancy cell and turn
 // pose jitter into apparent motion. A cluster dominated by known geometry is
 // static. Its rays still update the transient collision layer, so this
 // semantic rejection cannot erase a nearby obstacle from local safety.
 const std::size_t static_points = static_cast<std::size_t>(std::count_if(
 cluster.points.begin(), cluster.points.end(), [&](Vec2 point) {
 return !static_alignment_mask.isFree(point.x, point.y);
 }));
 if (static_points * 2 >= cluster.points.size())
 continue;
 DynamicObservation detected = observation_from_cluster(
 cluster, sensor_origin, scan.n_rays, perception_rng_);
 const auto static_overlap_mask =
 static_map_.inflated(std::max(0.0, detected.radius - kTrackRadiusPaddingM));
 if (!static_overlap_mask.isFree(detected.position.x, detected.position.y))
 continue;

 std::size_t match = tracks.size();
 double best_distance = 1.2;
 for (std::size_t index = 0; index < tracks.size(); ++index) {
 if (used_tracks[index])
 continue;
 const double candidate_distance = point_distance(detected.position, tracks[index].state.p);
 if (candidate_distance < best_distance) {
 best_distance = candidate_distance;
 match = index;
 }
 }
 if (match < tracks.size()) {
 used_tracks[match] = true;
 detected.object_id = tracks[match].object_id;
 dynamic_obstacles_.observe(detected, scan.header.stamp_s);
 continue;
 }

 std::size_t pending_match = pending_.size();
 best_distance = 1.0;
 for (std::size_t index = 0; index < pending_.size(); ++index) {
 if (used_pending[index])
 continue;
 const double candidate_distance = point_distance(detected.position, pending_[index].position);
 if (candidate_distance < best_distance) {
 best_distance = candidate_distance;
 pending_match = index;
 }
 }
 if (pending_match == pending_.size()) {
 pending_.push_back({next_track_id_++, detected.position, detected.radius, scan.header.stamp_s, 1});
 used_pending.push_back(true);
 continue;
 }
 used_pending[pending_match] = true;
 auto& pending = pending_[pending_match];
 pending.position = detected.position;
 pending.radius_m = detected.radius;
 pending.last_seen_s = scan.header.stamp_s;
 ++pending.confirmations;
 if (pending.confirmations >= 2) {
 detected.object_id = pending.id;
 dynamic_obstacles_.observe(detected, scan.header.stamp_s);
 }
 }
 pending_.erase(std::remove_if(pending_.begin(), pending_.end(), [&](const auto& pending) {
 return scan.header.stamp_s - pending.last_seen_s > 0.5;
 }), pending_.end());
 const auto classified_tracks = dynamic_obstacles_.active_tracks(scan.header.stamp_s, track_max_age_s);
 output.dynamic_object_detected = std::any_of(
 classified_tracks.begin(), classified_tracks.end(), [](const auto& track) {
 return track.motion_class != TrackMotionClass::Unconfirmed;
 });
 return output;
}

} // namespace sim
