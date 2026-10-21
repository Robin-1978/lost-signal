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
 return scan.n_rays > 0 && scan.ranges.size() == static_cast<std::size_t>(scan.n_rays) &&
 scan.hit.size() == scan.ranges.size() && std::isfinite(scan.max_range) &&
 scan.max_range > 0.0 && std::isfinite(scan.angle_min_rad) &&
 std::isfinite(scan.angle_increment_rad);
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
 // Never clear a previously observed obstacle with a later ray.
 if (map_.occupancy[cell_index] < 50)
 map_.occupancy[cell_index] = 0;
 }
 if (hit) {
 const Vec2 endpoint{sensor.x + range * std::cos(angle),
 sensor.y + range * std::sin(angle)};
 const auto cell = runtime::world_to_grid(map_, endpoint);
 if (cell) {
 const auto cell_index = static_cast<std::size_t>(cell->y) * map_.width + cell->x;
 map_.occupancy[cell_index] = 100;
 }
 }
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
 if (!observations.lidar || !finite_scan(*observations.lidar) ||
 observations.lidar->header.stamp_s > stamp_s + 1e-6 ||
 stamp_s - observations.lidar->header.stamp_s > 0.25) {
 return make_update(MappingState::Degraded, stamp_s, "missing or stale LiDAR observation");
 }
 const auto& scan = *observations.lidar;
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
