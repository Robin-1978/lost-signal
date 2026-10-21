#include "robot/autonomy/mapping/online_slam.hpp"

#include "robot/core/math.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace sim::autonomy {
namespace {

Vec2 rotate(Vec2 point, double heading) {
 const double c = std::cos(heading);
 const double s = std::sin(heading);
 return {c * point.x - s * point.y, s * point.x + c * point.y};
}

bool finite_pose(const EstimatedState& state) {
 return std::isfinite(state.position.x) && std::isfinite(state.position.y) &&
 std::isfinite(state.heading_rad) && std::isfinite(state.confidence);
}

bool finite_scan(const Scan& scan) {
 if (!(scan.n_rays > 0 &&
 scan.ranges.size() == static_cast<std::size_t>(scan.n_rays) &&
 scan.hit.size() == scan.ranges.size() && std::isfinite(scan.max_range) &&
 scan.max_range > 0.0 && std::isfinite(scan.angle_min_rad) &&
 std::isfinite(scan.angle_increment_rad) &&
 (scan.ray_heights_m.empty() ||
 scan.ray_heights_m.size() == scan.ranges.size())))
 return false;
 for (const double offset : scan.ray_time_offsets_s) {
 if (!std::isfinite(offset) || offset > 1e-9 ||
 (scan.scan_period_s > 0.0 && offset < -scan.scan_period_s - 1e-9))
 return false;
 }
 for (const double height : scan.ray_heights_m) {
 if (!std::isfinite(height) && !std::isnan(height))
 return false;
 }
 return true;
}

} // namespace

OnlineSlamMapBuilder::OnlineSlamMapBuilder(OnlineSlamConfig config)
 : config_(std::move(config)) {
 initialize_map();
}

void OnlineSlamMapBuilder::initialize_map() {
 map_ = {};
 map_.width = config_.width;
 map_.height = config_.height;
 map_.resolution_m = config_.resolution_m;
 map_.origin_m = {
 -0.5 * static_cast<double>(config_.width) * config_.resolution_m,
 -0.5 * static_cast<double>(config_.height) * config_.resolution_m};
 map_.origin_yaw_rad = 0.0;
 map_.occupancy.assign(static_cast<std::size_t>(config_.width) * config_.height, -1);
 map_.elevation_m.clear();
 map_.elevation_sigma_m.clear();
 elevation_ = ElevationFusion(config_.elevation_fusion);
 occupancy_ = OccupancyLogOdds(config_.occupancy);
 occupancy_.reset(map_.occupancy.size());
}

void OnlineSlamMapBuilder::reset() {
 initialize_map();
 initialized_ = false;
 last_stamp_s_ = -1.0;
 anchor_world_position_ = {};
 anchor_world_heading_rad_ = 0.0;
 pose_ = {};
 heading_rad_ = 0.0;
 revision_ = 0;
}

bool OnlineSlamMapBuilder::valid_config() const {
 return config_.width > 4 && config_.height > 4 &&
 std::isfinite(config_.resolution_m) && config_.resolution_m > 0.0 &&
 std::isfinite(config_.maximum_range_m) && config_.maximum_range_m > 0.0 &&
 std::isfinite(config_.sensor_x_m) && std::isfinite(config_.sensor_y_m) &&
 std::isfinite(config_.sensor_yaw_rad) &&
 std::isfinite(config_.sensor_height_m) && config_.sensor_height_m > 0.0 &&
 std::isfinite(config_.ground_rejection_band_m) &&
 config_.ground_rejection_band_m >= 0.0 &&
 std::isfinite(config_.translation_search_m) && config_.translation_search_m >= 0.0 &&
 std::isfinite(config_.translation_step_m) && config_.translation_step_m > 0.0 &&
 std::isfinite(config_.heading_search_rad) && config_.heading_search_rad >= 0.0 &&
 std::isfinite(config_.heading_step_rad) && config_.heading_step_rad > 0.0 &&
 config_.maximum_matching_returns > 0 &&
 std::isfinite(config_.minimum_match_score) &&
 std::isfinite(config_.minimum_match_margin) &&
 config_.map_id.size() > 0;
}

Vec2 OnlineSlamMapBuilder::world_to_local(Vec2 world) const {
 return rotate({world.x - anchor_world_position_.x,
 world.y - anchor_world_position_.y}, -anchor_world_heading_rad_);
}

Vec2 OnlineSlamMapBuilder::sensor_position(Vec2 pose, double heading) const {
 const auto offset = rotate({config_.sensor_x_m, config_.sensor_y_m}, heading);
 return {pose.x + offset.x, pose.y + offset.y};
}

std::size_t OnlineSlamMapBuilder::known_cells() const {
 return static_cast<std::size_t>(std::count_if(
 map_.occupancy.begin(), map_.occupancy.end(), [](std::int8_t value) {
 return value >= 0;
 }));
}

double OnlineSlamMapBuilder::ray_time_offset_s(const Scan& scan,
 std::size_t index) const {
 if (index < scan.ray_time_offsets_s.size() &&
 std::isfinite(scan.ray_time_offsets_s[index]))
 return scan.ray_time_offsets_s[index];
 if (scan.time_increment_s > 0.0 && std::isfinite(scan.time_increment_s)) {
 return -scan.time_increment_s *
 static_cast<double>(scan.n_rays - 1 - static_cast<int>(index));
 }
 if (scan.scan_period_s > 0.0 && std::isfinite(scan.scan_period_s) &&
 scan.n_rays > 1) {
 return -scan.scan_period_s *
 static_cast<double>(scan.n_rays - 1 - static_cast<int>(index)) /
 static_cast<double>(scan.n_rays - 1);
 }
 return 0.0;
}

std::optional<Scan> OnlineSlamMapBuilder::project_cloud(
 const LidarPointCloud& cloud) const {
 if (cloud.rings == 0 || cloud.azimuth_samples == 0 ||
 !std::isfinite(cloud.max_range_m) || cloud.max_range_m <= 0.0 ||
 !std::isfinite(cloud.scan_period_s) || cloud.scan_period_s <= 0.0)
 return std::nullopt;
 Scan scan;
 scan.n_rays = static_cast<int>(cloud.azimuth_samples);
 scan.max_range = std::min(cloud.max_range_m, config_.maximum_range_m);
 scan.angle_min_rad = 0.0;
 scan.angle_increment_rad = 2.0 * kPi /
 static_cast<double>(cloud.azimuth_samples);
 scan.scan_period_s = cloud.scan_period_s;
 scan.ranges.assign(cloud.azimuth_samples, scan.max_range);
 scan.hit.assign(cloud.azimuth_samples, 0);
 scan.intensities.assign(cloud.azimuth_samples, 0.0);
 scan.ray_time_offsets_s.assign(cloud.azimuth_samples, -scan.scan_period_s);
 if (config_.publish_elevation_layer)
 scan.ray_heights_m.assign(cloud.azimuth_samples,
 std::numeric_limits<double>::quiet_NaN());
 scan.header = cloud.header;
 for (const auto& point : cloud.points) {
 if (!std::isfinite(point.x_m) || !std::isfinite(point.y_m) ||
 !std::isfinite(point.z_m) ||
 !std::isfinite(point.time_offset_s) || point.time_offset_s > 1e-9 ||
 point.time_offset_s < -cloud.scan_period_s - 1e-9 ||
 (point.x_m == 0.0 && point.y_m == 0.0) ||
 point.z_m <= -config_.sensor_height_m + config_.ground_rejection_band_m)
 continue;
 const double range = std::hypot(point.x_m, point.y_m);
 if (!(range > 0.05) || range >= scan.max_range)
 continue;
 double angle = std::atan2(point.y_m, point.x_m);
 if (angle < 0.0)
 angle += 2.0 * kPi;
 const auto bin = static_cast<std::size_t>(std::floor(
 angle / (2.0 * kPi) * cloud.azimuth_samples)) % cloud.azimuth_samples;
 if (!scan.hit[bin] || range < scan.ranges[bin]) {
 scan.ranges[bin] = range;
 scan.hit[bin] = 1;
 scan.intensities[bin] = std::clamp(point.intensity, 0.0, 1.0);
 scan.ray_time_offsets_s[bin] = point.time_offset_s;
 if (!scan.ray_heights_m.empty())
 scan.ray_heights_m[bin] = point.z_m;
 }
 }
 return scan;
}

void OnlineSlamMapBuilder::update_elevation(std::size_t cell_index,
 double elevation_m) {
 if (!config_.publish_elevation_layer || !std::isfinite(elevation_m) ||
 cell_index >= map_.occupancy.size())
 return;
 if (map_.elevation_m.size() != map_.occupancy.size()) {
 map_.elevation_m.assign(map_.occupancy.size(),
 std::numeric_limits<float>::quiet_NaN());
 map_.elevation_sigma_m.assign(map_.occupancy.size(),
 std::numeric_limits<float>::quiet_NaN());
 elevation_.reset(map_.occupancy.size());
 }
 const auto estimate = elevation_.fuse(cell_index, elevation_m);
 if (estimate.valid) {
 map_.elevation_m[cell_index] = static_cast<float>(estimate.height_m);
 map_.elevation_sigma_m[cell_index] = static_cast<float>(estimate.sigma_m);
 }
}

double OnlineSlamMapBuilder::match_score(const Scan& scan,
 const EstimatedState& motion, Vec2 pose, double heading,
 std::size_t* matched_returns) const {
 const std::size_t stride = std::max<std::size_t>(
 1, (scan.ranges.size() + config_.maximum_matching_returns - 1) /
 config_.maximum_matching_returns);
 double score = 0.0;
 std::size_t compared = 0;
 std::size_t matched = 0;
 for (std::size_t index = 0; index < scan.ranges.size(); index += stride) {
 if (!scan.hit[index] || !std::isfinite(scan.ranges[index]) ||
 scan.ranges[index] <= 0.05 || scan.ranges[index] >= scan.max_range)
 continue;
 const double range = std::min(scan.ranges[index], config_.maximum_range_m);
 const double time_offset_s = ray_time_offset_s(scan, index);
 const double ray_heading = heading + motion.angular_velocity_radps * time_offset_s;
 const auto body_delta = rotate({motion.linear_velocity_mps * time_offset_s,
 motion.lateral_velocity_mps * time_offset_s}, heading);
 const auto sensor = sensor_position(
 {pose.x + body_delta.x, pose.y + body_delta.y}, ray_heading);
 const double angle = ray_heading + scan.angle_min_rad +
 scan.angle_increment_rad * static_cast<double>(index) + config_.sensor_yaw_rad;
 const Vec2 endpoint{sensor.x + range * std::cos(angle),
 sensor.y + range * std::sin(angle)};
 const auto cell = runtime::world_to_grid(map_, endpoint);
 if (!cell) {
 score -= 0.6;
 ++compared;
 continue;
 }
 const auto cell_index = static_cast<std::size_t>(cell->y) * map_.width + cell->x;
 const int value = map_.occupancy[cell_index];
 if (value < 0) {
 score += 0.05; // unknown is weak evidence, never a match
 } else if (value >= 50) {
 score += 1.0;
 ++matched;
 } else {
 score -= 0.35;
 }
 ++compared;
 }
 if (matched_returns)
 *matched_returns = matched;
 return compared == 0 ? 0.0 : score / static_cast<double>(compared);
}

void OnlineSlamMapBuilder::integrate_scan(const Scan& scan,
 const EstimatedState& motion) {
 const double max_range = std::min(scan.max_range, config_.maximum_range_m);
 const double ray_step = std::max(0.02, config_.resolution_m * 0.5);
 for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
 if (!std::isfinite(scan.ranges[index]) || scan.ranges[index] <= 0.05)
 continue;
 const bool hit = scan.hit[index] && scan.ranges[index] < scan.max_range;
 const double range = std::min(scan.ranges[index], max_range);
 const double time_offset_s = ray_time_offset_s(scan, index);
 const double ray_heading = heading_rad_ + motion.angular_velocity_radps * time_offset_s;
 const auto body_delta = rotate({motion.linear_velocity_mps * time_offset_s,
 motion.lateral_velocity_mps * time_offset_s}, heading_rad_);
 const auto sensor = sensor_position(
 {pose_.x + body_delta.x, pose_.y + body_delta.y}, ray_heading);
 const double angle = ray_heading + scan.angle_min_rad +
 scan.angle_increment_rad * static_cast<double>(index) + config_.sensor_yaw_rad;
 const int subdivisions = std::max(1, static_cast<int>(std::ceil(range / ray_step)));
 const int free_subdivisions = hit ? std::max(0, subdivisions - 1) : subdivisions;
 for (int step = 1; step <= free_subdivisions; ++step) {
 const double distance = range * static_cast<double>(step) /
 static_cast<double>(subdivisions);
 const Vec2 point{sensor.x + distance * std::cos(angle),
 sensor.y + distance * std::sin(angle)};
 const auto cell = runtime::world_to_grid(map_, point);
 if (!cell)
 break;
 const auto cell_index = static_cast<std::size_t>(cell->y) * map_.width + cell->x;
	 occupancy_.observe_free(cell_index);
 }
 if (hit) {
 const Vec2 endpoint{sensor.x + range * std::cos(angle),
 sensor.y + range * std::sin(angle)};
 const auto cell = runtime::world_to_grid(map_, endpoint);
 if (cell) {
 const auto cell_index = static_cast<std::size_t>(cell->y) * map_.width + cell->x;
	 occupancy_.observe_occupied(cell_index);
	 if (occupancy_.value(cell_index) >= 50 && index < scan.ray_heights_m.size() &&
	  std::isfinite(scan.ray_heights_m[index]))
	  update_elevation(cell_index, config_.sensor_height_m +
	  scan.ray_heights_m[index]);
 }
 }
 for (std::size_t index = 0; index < map_.occupancy.size(); ++index)
 map_.occupancy[index] = occupancy_.value(index);
 }
}

MapBuilderUpdate OnlineSlamMapBuilder::make_update(
 MappingState state, double stamp_s, std::string diagnostic) const {
 MapBuilderUpdate output;
 output.state = state;
 output.map_revision = revision_;
 output.diagnostic = std::move(diagnostic);
 output.pose.position = pose_;
 output.pose.heading_rad = heading_rad_;
 output.pose.position_sigma_m = std::max(0.02, config_.resolution_m * 2.0);
 output.pose.heading_sigma_rad = std::max(0.02, config_.heading_step_rad * 2.0);
 output.pose.header = {stamp_s, 0, "map", 0.0,
 kMeasurementSchemaVersion, "robot.online-slam"};
 auto map = std::make_shared<runtime::MapSnapshot>();
 map->map_id = config_.map_id;
 map->revision = revision_;
 map->frame_id = "map";
 map->sample_time = runtime::TimePoint::from_seconds(stamp_s);
 map->provenance = {"slam", "robot.online-slam", std::to_string(revision_)};
 map->grid = map_;
 output.published_map = std::move(map);
 return output;
}

MapBuilderUpdate OnlineSlamMapBuilder::update(
 const ObservationFrame& observations, const EstimatedState& prior,
 runtime::TimePoint sample_time) {
 const double stamp_s = sample_time.seconds();
 if (!valid_config())
 return make_update(MappingState::Degraded, stamp_s, "invalid online SLAM configuration");
 if (!finite_pose(prior) || !std::isfinite(stamp_s) || stamp_s < 0.0 ||
 (last_stamp_s_ >= 0.0 && stamp_s <= last_stamp_s_ + 1e-9)) {
 return make_update(MappingState::Degraded, stamp_s, "stale or invalid SLAM state");
 }
 std::optional<Scan> projected_cloud;
 const Scan* scan_ptr = nullptr;
 if (observations.lidar_point_cloud) {
 projected_cloud = project_cloud(*observations.lidar_point_cloud);
 if (projected_cloud)
 scan_ptr = &*projected_cloud;
 } else if (observations.lidar) {
 scan_ptr = &*observations.lidar;
 }
 if (!scan_ptr || !finite_scan(*scan_ptr) ||
 scan_ptr->header.stamp_s > stamp_s + 1e-6 ||
 stamp_s - scan_ptr->header.stamp_s > 0.25) {
 return make_update(MappingState::Degraded, stamp_s, "missing or stale LiDAR observation");
 }
 const auto& scan = *scan_ptr;
 if (!initialized_) {
 anchor_world_position_ = prior.position;
 anchor_world_heading_rad_ = prior.heading_rad;
 pose_ = {};
 heading_rad_ = 0.0;
 initialized_ = true;
 } else {
 pose_ = world_to_local(prior.position);
 heading_rad_ = wrap_angle(prior.heading_rad - anchor_world_heading_rad_);
 }

 std::string diagnostic = "mapping";
 if (known_cells() > 0) {
 const Vec2 predicted_pose = pose_;
 const double predicted_heading = heading_rad_;
 const int translation_steps = static_cast<int>(std::ceil(
 config_.translation_search_m / config_.translation_step_m));
 const int heading_steps = static_cast<int>(std::ceil(
 config_.heading_search_rad / config_.heading_step_rad));
 double best_score = match_score(scan, prior, predicted_pose, predicted_heading, nullptr);
 double second_score = -std::numeric_limits<double>::infinity();
 Vec2 best_pose = predicted_pose;
 double best_heading = predicted_heading;
 for (int y = -translation_steps; y <= translation_steps; ++y) {
 for (int x = -translation_steps; x <= translation_steps; ++x) {
 for (int h = -heading_steps; h <= heading_steps; ++h) {
 const Vec2 candidate{predicted_pose.x + x * config_.translation_step_m,
 predicted_pose.y + y * config_.translation_step_m};
 const double candidate_heading = wrap_angle(
 predicted_heading + h * config_.heading_step_rad);
 const double score = match_score(scan, prior, candidate, candidate_heading, nullptr);
 if (score > best_score) {
 second_score = best_score;
 best_score = score;
 best_pose = candidate;
 best_heading = candidate_heading;
 } else if (score > second_score) {
 second_score = score;
 }
 }
 }
 }
 if (best_score >= config_.minimum_match_score &&
 (!std::isfinite(second_score) || best_score - second_score >= config_.minimum_match_margin)) {
 pose_ = best_pose;
 heading_rad_ = best_heading;
 diagnostic = "scan-matched mapping";
 } else {
 diagnostic = "odometry prior retained; scan match weak";
 }
 }
 integrate_scan(scan, prior);
 last_stamp_s_ = stamp_s;
 ++revision_;
 return make_update(MappingState::Mapping, stamp_s, std::move(diagnostic));
}

} // namespace sim::autonomy
