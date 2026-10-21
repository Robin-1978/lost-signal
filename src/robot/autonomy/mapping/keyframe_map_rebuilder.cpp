#include "robot/autonomy/mapping/keyframe_map_rebuilder.hpp"

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
 return config_.width > 4 && config_.height > 4 &&
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
 map.width = config_.width;
 map.height = config_.height;
 map.resolution_m = config_.resolution_m;
 map.origin_m = {-0.5 * static_cast<double>(config_.width) * config_.resolution_m,
 -0.5 * static_cast<double>(config_.height) * config_.resolution_m};
 map.occupancy.assign(static_cast<std::size_t>(map.width) * map.height, -1);
 if (config_.publish_elevation_layer) {
 map.elevation_m.assign(map.occupancy.size(),
 std::numeric_limits<float>::quiet_NaN());
 map.elevation_sigma_m.assign(map.occupancy.size(),
 std::numeric_limits<float>::quiet_NaN());
 }
 const double ray_step = std::max(0.02, config_.resolution_m * 0.5);
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
 if (map.occupancy[cell_index] < 50)
 map.occupancy[cell_index] = 0;
 }
 if (!hit)
 continue;
 const Vec2 endpoint{sensor.x + range * std::cos(angle),
 sensor.y + range * std::sin(angle)};
 const auto cell = runtime::world_to_grid(map, endpoint);
 if (!cell)
 continue;
 const auto cell_index = static_cast<std::size_t>(cell->y) * map.width + cell->x;
 map.occupancy[cell_index] = 100;
 if (config_.publish_elevation_layer && index < scan.ray_heights_m.size() &&
 std::isfinite(scan.ray_heights_m[index])) {
 map.elevation_m[cell_index] = static_cast<float>(
 config_.sensor_height_m + scan.ray_heights_m[index]);
 }
 }
 }
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
