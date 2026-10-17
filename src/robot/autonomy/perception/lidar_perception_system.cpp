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

ObservedRay2D project_ray(const Scan& scan, Vec2 current_origin, double current_heading,
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
 sample_origin,
 {sample_origin.x + std::cos(angle) * scan.ranges[ray],
 sample_origin.y + std::sin(angle) * scan.ranges[ray]},
 scan.hit[ray] != 0,
 };
}

double point_distance(Vec2 a, Vec2 b) {
 return std::hypot(a.x - b.x, a.y - b.y);
}

std::vector<Cluster> dynamic_clusters(const Scan& scan, const Scan& static_scan,
 const std::vector<ObservedRay2D>& projected, double noise_sigma_m) {
 std::vector<Cluster> clusters;
 Cluster current;
 const double threshold = std::max(0.35, noise_sigma_m * 4.0);
 for (int ray = 0; ray < scan.n_rays; ++ray) {
 const bool dynamic = scan.hit[ray] &&
 (!static_scan.hit[ray] || scan.ranges[ray] + threshold < static_scan.ranges[ray]);
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
 const bool first_dynamic = scan.hit.front() &&
 (!static_scan.hit.front() || scan.ranges.front() + threshold < static_scan.ranges.front());
 const bool last_dynamic = scan.hit.back() &&
 (!static_scan.hit.back() || scan.ranges.back() + threshold < static_scan.ranges.back());
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
 if (!static_scan.hit[ray])
 return true;
 // A foreground endpoint must exceed the localization/map-alignment envelope
 // before becoming a second obstacle beside an authored wall. Half a source
 // cell plus measured range noise covers quantization. The localization
 // system supplies the remaining alignment envelope explicitly.
 const double residual_tolerance_m =
 std::max(map_alignment_tolerance_m,
 0.5 * map_resolution_m + 4.0 * noise_sigma_m);
 return scan.ranges[ray] + residual_tolerance_m < static_scan.ranges[ray];
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
 const double cosine = std::cos(state.base_heading_rad);
 const double sine = std::sin(state.base_heading_rad);
 const Vec2 sensor_origin{
 state.base_position.x + cosine * config.position_in_base.x - sine * config.position_in_base.y,
 state.base_position.y + sine * config.position_in_base.x + cosine * config.position_in_base.y,
 };
 const double sensor_heading = state.base_heading_rad + config.yaw_in_base_rad;

 LidarPerceptionFrame output;
 output.obstacle_rays.reserve(static_cast<std::size_t>(scan.n_rays));
 for (int ray = 0; ray < scan.n_rays; ++ray) {
 auto projected = project_ray(scan, sensor_origin, sensor_heading,
 state.linear_velocity_mps, state.angular_velocity_radps, ray);
 // Known static geometry is already owned and inflated by the immutable map
 // layer. Mark only foreground endpoints in the transient observation layer;
 // every beam still participates in clearing that layer.
 projected.hit = is_unmapped_return(
 scan, static_reference, ray, config.range_noise_sigma_m, static_map_.res(),
 config.map_alignment_tolerance_m);
 output.obstacle_rays.push_back(projected);
 if (!scan.hit[ray])
 continue;
 output.hits.push_back(output.obstacle_rays.back().endpoint);
 if (scan.ranges[ray] <= std::min(scan.max_range, 2.5))
 output.static_obstacle_near = true;
 }

 const auto clusters = dynamic_clusters(scan, static_reference, output.obstacle_rays,
 config.range_noise_sigma_m);
 auto tracks = dynamic_obstacles_.active_tracks(scan.header.stamp_s, track_max_age_s);
 std::vector<bool> used_tracks(tracks.size(), false);
 std::vector<bool> used_pending(pending_.size(), false);
 for (const auto& cluster : clusters) {
 DynamicObservation detected = observation_from_cluster(cluster, sensor_origin, scan.n_rays, perception_rng_);
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
