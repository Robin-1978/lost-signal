#include "robot/autonomy/mapping/keyframe_map_rebuilder.hpp"

#include "robot/autonomy/mapping/elevation_fusion.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace sim::autonomy {
namespace {

Vec2 rotate(Vec2 point, double heading) {
 const double cosine = std::cos(heading);
 const double sine = std::sin(heading);
 return {cosine * point.x - sine * point.y,
 sine * point.x + cosine * point.y};
}

} // namespace

KeyframeMapRebuilder::KeyframeMapRebuilder(KeyframeMapRebuildConfig config)
 : config_(std::move(config)) {}

bool KeyframeMapRebuilder::valid_config() const {
 const bool raster_valid = config_.derive_extents_from_session
 ? config_.maximum_width > 4 && config_.maximum_height > 4
 : config_.width > 4 && config_.height > 4;
 return raster_valid && std::isfinite(config_.extent_margin_m) &&
 config_.extent_margin_m >= 0.0 &&
 std::isfinite(config_.resolution_m) && config_.resolution_m > 0.0 &&
 std::isfinite(config_.maximum_range_m) && config_.maximum_range_m > 0.0 &&
 std::isfinite(config_.sensor_x_m) && std::isfinite(config_.sensor_y_m) &&
 std::isfinite(config_.sensor_yaw_rad) &&
 std::isfinite(config_.sensor_height_m) && config_.sensor_height_m > 0.0 &&
 std::isfinite(config_.ground_rejection_band_m) &&
 config_.ground_rejection_band_m >= 0.0 && !config_.map_id.empty();
}

KeyframeMapRebuildResult KeyframeMapRebuilder::rebuild(
 const std::vector<MappingSessionKeyframe>& keyframes,
 std::uint64_t revision, double publication_time_s) const {
 KeyframeMapRebuildResult output;
 if (!valid_config()) {
 output.diagnostic = "invalid keyframe map rebuild configuration";
 return output;
 }
 if (keyframes.empty()) {
 output.diagnostic = "cannot rebuild a map without keyframes";
 return output;
 }
 if (revision == 0 || !std::isfinite(publication_time_s) || publication_time_s < 0.0) {
 output.diagnostic = "invalid map revision or publication time";
 return output;
 }
 runtime::MapGrid map;
 map.resolution_m = config_.resolution_m;
 if (config_.derive_extents_from_session) {
 double minimum_x = std::numeric_limits<double>::infinity();
 double minimum_y = std::numeric_limits<double>::infinity();
 double maximum_x = -std::numeric_limits<double>::infinity();
 double maximum_y = -std::numeric_limits<double>::infinity();
 for (const auto& keyframe : keyframes) {
 const auto& pose = keyframe.graph.initial_pose;
 if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y)) {
 output.diagnostic = "keyframe pose is not finite";
 return output;
 }
 minimum_x = std::min(minimum_x, pose.position.x);
 minimum_y = std::min(minimum_y, pose.position.y);
 maximum_x = std::max(maximum_x, pose.position.x);
 maximum_y = std::max(maximum_y, pose.position.y);
 }
 const double half_extent = config_.maximum_range_m + config_.extent_margin_m;
 const double resolution = config_.resolution_m;
 minimum_x = std::floor((minimum_x - half_extent) / resolution) * resolution;
 minimum_y = std::floor((minimum_y - half_extent) / resolution) * resolution;
 maximum_x = std::ceil((maximum_x + half_extent) / resolution) * resolution;
 maximum_y = std::ceil((maximum_y + half_extent) / resolution) * resolution;
 const auto width = static_cast<std::uint64_t>(std::max(1.0,
 std::ceil((maximum_x - minimum_x) / resolution)));
 const auto height = static_cast<std::uint64_t>(std::max(1.0,
 std::ceil((maximum_y - minimum_y) / resolution)));
 if (width > config_.maximum_width || height > config_.maximum_height) {
 output.diagnostic = "mapping session exceeds rebuild raster bounds: " +
 std::to_string(width) + "x" + std::to_string(height);
 return output;
 }
 map.width = static_cast<std::uint32_t>(width);
 map.height = static_cast<std::uint32_t>(height);
 map.origin_m = {minimum_x, minimum_y};
 } else {
 map.width = config_.width;
 map.height = config_.height;
 map.origin_m = {-0.5 * static_cast<double>(config_.width) * config_.resolution_m,
 -0.5 * static_cast<double>(config_.height) * config_.resolution_m};
 }
 map.occupancy.assign(static_cast<std::size_t>(map.width) * map.height, -1);
 if (config_.publish_elevation_layer) {
 map.elevation_m.assign(map.occupancy.size(),
 std::numeric_limits<float>::quiet_NaN());
 map.elevation_sigma_m.assign(map.occupancy.size(),
 std::numeric_limits<float>::quiet_NaN());
 }
 const double ray_step = std::max(0.02, config_.resolution_m * 0.5);
 ElevationFusion elevation(config_.elevation);
 elevation.reset(map.occupancy.size());
 OccupancyLogOdds occupancy(config_.occupancy);
 occupancy.reset(map.occupancy.size());
 for (const auto& keyframe : keyframes) {
 const auto& pose = keyframe.graph.initial_pose;
 const auto& scan = keyframe.scan;
 for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
 const double source_range = scan.ranges[index];
 if (!std::isfinite(source_range) || source_range <= 0.05)
 continue;
 const bool hit = scan.hit[index] && source_range < scan.max_range;
 const double range = std::min(source_range, config_.maximum_range_m);
 double time_offset_s = 0.0;
 if (index < scan.ray_time_offsets_s.size() &&
 std::isfinite(scan.ray_time_offsets_s[index])) {
 time_offset_s = scan.ray_time_offsets_s[index];
 } else if (scan.time_increment_s > 0.0 && std::isfinite(scan.time_increment_s)) {
 time_offset_s = -scan.time_increment_s *
 static_cast<double>(scan.n_rays - 1 - static_cast<int>(index));
 } else if (scan.scan_period_s > 0.0 && std::isfinite(scan.scan_period_s) &&
 scan.n_rays > 1) {
 time_offset_s = -scan.scan_period_s *
 static_cast<double>(scan.n_rays - 1 - static_cast<int>(index)) /
 static_cast<double>(scan.n_rays - 1);
 }
 const double ray_heading = pose.heading_rad +
 keyframe.motion.angular_velocity_radps * time_offset_s;
 const auto body_delta = rotate({keyframe.motion.linear_velocity_mps * time_offset_s,
 keyframe.motion.lateral_velocity_mps * time_offset_s}, pose.heading_rad);
 const auto offset = rotate({config_.sensor_x_m, config_.sensor_y_m}, ray_heading);
 const Vec2 sensor{pose.position.x + body_delta.x + offset.x,
 pose.position.y + body_delta.y + offset.y};
 const double angle = ray_heading + scan.angle_min_rad +
 scan.angle_increment_rad * static_cast<double>(index) +
 config_.sensor_yaw_rad;
 const int subdivisions = std::max(1, static_cast<int>(std::ceil(range / ray_step)));
 const int free_steps = hit ? std::max(0, subdivisions - 1) : subdivisions;
 for (int step = 1; step <= free_steps; ++step) {
 const double distance = range * static_cast<double>(step) /
 static_cast<double>(subdivisions);
 const Vec2 point{sensor.x + distance * std::cos(angle),
 sensor.y + distance * std::sin(angle)};
 const auto cell = runtime::world_to_grid(map, point);
 if (!cell)
 break;
 const auto cell_index = static_cast<std::size_t>(cell->y) * map.width + cell->x;
 occupancy.observe_free(cell_index);
 }
 if (!hit)
 continue;
 const Vec2 endpoint{sensor.x + range * std::cos(angle),
 sensor.y + range * std::sin(angle)};
 const auto cell = runtime::world_to_grid(map, endpoint);
 if (!cell)
 continue;
 const auto cell_index = static_cast<std::size_t>(cell->y) * map.width + cell->x;
 occupancy.observe_occupied(cell_index);
 if (config_.publish_elevation_layer && index < scan.ray_heights_m.size() &&
 std::isfinite(scan.ray_heights_m[index])) {
 const auto estimate = elevation.fuse(
 cell_index, config_.sensor_height_m + scan.ray_heights_m[index]);
 if (estimate.valid) {
 map.elevation_m[cell_index] = static_cast<float>(estimate.height_m);
 map.elevation_sigma_m[cell_index] = static_cast<float>(estimate.sigma_m);
 }
 }
 }
 }
 map.occupancy = occupancy.values();
 auto snapshot = std::make_shared<runtime::MapSnapshot>();
 snapshot->map_id = config_.map_id;
 snapshot->revision = revision;
 snapshot->frame_id = "map";
 snapshot->sample_time = runtime::TimePoint::from_seconds(publication_time_s);
 snapshot->provenance = {"slam", "robot.keyframe-map-rebuilder",
 std::to_string(revision)};
 snapshot->grid = std::move(map);
 output.success = true;
 output.diagnostic = "keyframe map reconstructed";
 output.snapshot = std::move(snapshot);
 return output;
}

} // namespace sim::autonomy
