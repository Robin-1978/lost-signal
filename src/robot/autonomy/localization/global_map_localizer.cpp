#include "robot/autonomy/localization/global_map_localizer.hpp"

#include "robot/core/math.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

namespace sim {
namespace {

struct Return {
 double range{0.0};
 double angle{0.0};
};

struct Candidate {
 Vec2 position;
 double heading{0.0};
 double score{-std::numeric_limits<double>::infinity()};
 double overlap{0.0};
};

bool finite_pose(Vec2 position, double heading) {
 return std::isfinite(position.x) && std::isfinite(position.y) &&
 std::isfinite(heading);
}

Vec2 rotate(Vec2 point, double heading) {
 const double c = std::cos(heading);
 const double s = std::sin(heading);
 return {c * point.x - s * point.y, s * point.x + c * point.y};
}

Vec2 sensor_world_position(Vec2 base, double heading,
 const GlobalMapLocalizationConfig& config) {
 const auto offset = rotate({config.sensor_x_m, config.sensor_y_m}, heading);
 return {base.x + offset.x, base.y + offset.y};
}

std::optional<double> expected_range(const runtime::MapGrid& grid,
 Vec2 sensor, double beam_angle, double max_range) {
 // A coarse ray cast is deliberately used only for global hypothesis search.
 // The accepted pose is refined by the continuous local matcher. Sampling at
 // half a map cell prevents a thin occupied cell from being skipped.
 const double step = std::max(0.05, grid.resolution_m);
 for (double distance = step; distance < max_range; distance += step) {
 const Vec2 point{sensor.x + distance * std::cos(beam_angle),
 sensor.y + distance * std::sin(beam_angle)};
 const auto cell = runtime::world_to_grid(grid, point);
 if (!cell)
 return std::nullopt;
 const auto index = static_cast<std::size_t>(cell->y) * grid.width + cell->x;
 if (index < grid.occupancy.size() && grid.occupancy[index] >= 50)
 return distance;
 }
 return max_range;
}

bool separated(const Candidate& first, const Candidate& second,
 const GlobalMapLocalizationConfig& config) {
 return std::hypot(first.position.x - second.position.x,
 second.position.y - first.position.y) >= config.ambiguity_distance_m ||
 std::abs(wrap_angle(first.heading - second.heading)) >=
 config.ambiguity_heading_rad;
}

void retain_candidate(std::vector<Candidate>& candidates, Candidate candidate) {
 candidates.push_back(candidate);
 std::sort(candidates.begin(), candidates.end(), [](const auto& left, const auto& right) {
 return left.score > right.score;
 });
 constexpr std::size_t kRetained = 16;
 if (candidates.size() > kRetained)
 candidates.resize(kRetained);
}

} // namespace

autonomy::LocalizationResult GlobalMapLocalizer::localize(
 const Scan& scan, const Vec2& odom_position, double odom_heading_rad,
 const runtime::MapSnapshot& map, double timestamp_s) const {
 autonomy::LocalizationResult result;
 result.pose.header = {timestamp_s, 0, "map", 0.0,
 kMeasurementSchemaVersion, "robot.global-known-map-lidar"};
 if (!finite_pose(odom_position, odom_heading_rad)) {
 result.state = autonomy::LocalizationState::Degraded;
 result.rejection_reason = "invalid odometry prior";
 return result;
 }
 if (scan.n_rays <= 0 || scan.ranges.size() != static_cast<std::size_t>(scan.n_rays) ||
 scan.hit.size() != scan.ranges.size() || !(scan.max_range > 0.0) ||
 !std::isfinite(scan.header.stamp_s) ||
 std::abs(scan.header.stamp_s - timestamp_s) > 0.05) {
 result.state = autonomy::LocalizationState::Degraded;
 result.rejection_reason = "invalid or stale scan";
 return result;
 }
 if (!map.grid || map.frame_id != "map" || map.grid->width == 0 ||
 map.grid->height == 0 || !(map.grid->resolution_m > 0.0) ||
 map.grid->occupancy.size() !=
 static_cast<std::size_t>(map.grid->width) * map.grid->height) {
 result.state = autonomy::LocalizationState::Degraded;
 result.rejection_reason = "map snapshot unavailable or invalid";
 return result;
 }
 if (!(config_.grid_step_m > 0.0) || !(config_.heading_step_rad > 0.0) ||
 !(config_.minimum_overlap >= 0.0 && config_.minimum_overlap <= 1.0) ||
 !(config_.minimum_score_margin >= 0.0) || config_.minimum_returns == 0 ||
 config_.maximum_returns < config_.minimum_returns ||
 config_.maximum_evaluations == 0) {
 result.state = autonomy::LocalizationState::Degraded;
 result.rejection_reason = "invalid global matcher configuration";
 return result;
 }

 std::vector<Return> returns;
 returns.reserve(std::min<std::size_t>(scan.ranges.size(), config_.maximum_returns));
 const std::size_t stride = std::max<std::size_t>(
 1, (scan.ranges.size() + config_.maximum_returns - 1) /
 config_.maximum_returns);
 for (std::size_t index = 0; index < scan.ranges.size(); index += stride) {
 const double range = scan.ranges[index];
 if (!scan.hit[index] || !std::isfinite(range) || range <= 0.05 ||
 range >= scan.max_range)
 continue;
 returns.push_back({range, scan.angle_min_rad +
 scan.angle_increment_rad * static_cast<double>(index)});
 }
 result.usable_returns = returns.size();
 if (returns.size() < config_.minimum_returns) {
 result.state = autonomy::LocalizationState::Degraded;
 result.rejection_reason = "insufficient geometric returns";
 return result;
 }

 const auto& grid = *map.grid;
 const auto grid_stride = std::max<std::uint32_t>(1, static_cast<std::uint32_t>(
 std::ceil(config_.grid_step_m / grid.resolution_m)));
 const auto heading_count = std::max<std::size_t>(1, static_cast<std::size_t>(
 std::ceil((2.0 * kPi) / config_.heading_step_rad)));
 // Global recovery only needs a stable coarse signature. Keep the full
 // return count for diagnostics, but cap expensive ray comparisons to a
 // deterministic angularly distributed subset.
 std::vector<Return> scoring_returns;
 const std::size_t scoring_stride = std::max<std::size_t>(
 1, (returns.size() + 47) / 48);
 for (std::size_t index = 0; index < returns.size(); index += scoring_stride)
 scoring_returns.push_back(returns[index]);

 std::vector<Candidate> candidates;
 candidates.reserve(16);
 std::size_t evaluations = 0;
 for (std::uint32_t y = 0; y < grid.height; y += grid_stride) {
 for (std::uint32_t x = 0; x < grid.width; x += grid_stride) {
 const auto position = runtime::grid_cell_center(grid, {x, y});
 const auto cell_index = static_cast<std::size_t>(y) * grid.width + x;
 if (cell_index >= grid.occupancy.size() || grid.occupancy[cell_index] >= 50)
 continue;
 for (std::size_t heading_index = 0; heading_index < heading_count; ++heading_index) {
 if (++evaluations > config_.maximum_evaluations) {
 result.state = autonomy::LocalizationState::Degraded;
 result.rejection_reason = "global search budget exceeded";
 return result;
 }
 const double heading = wrap_angle(
 -kPi + static_cast<double>(heading_index) * config_.heading_step_rad);
 const auto sensor_position = sensor_world_position(position, heading, config_);
 double score = 0.0;
 std::size_t matches = 0;
 for (const auto& point : scoring_returns) {
 const double beam_angle = heading + point.angle + config_.sensor_yaw_rad;
 const auto expected = expected_range(grid, sensor_position, beam_angle,
 scan.max_range);
 if (!expected) {
 // A ray leaving the map before the measured return is inconsistent with
 // the candidate. This is stronger evidence than merely scoring the hit
 // endpoint, which is ambiguous in repetitive corridors.
 score -= 1.0;
 continue;
 }
 const double error = std::abs(*expected - point.range);
 score += std::exp(-error / std::max(0.05, grid.resolution_m * 2.0));
 if (*expected < scan.max_range && error <= std::max(0.12,
 grid.resolution_m * 2.0))
 ++matches;
 }
 score /= static_cast<double>(scoring_returns.size());
 retain_candidate(candidates, {position, heading, score,
 static_cast<double>(matches) / static_cast<double>(scoring_returns.size())});
 }
 }
 }
 if (candidates.empty() || candidates.front().overlap < config_.minimum_overlap) {
 result.state = autonomy::LocalizationState::Degraded;
 result.rejection_reason = "global map overlap below acceptance gate";
 return result;
 }
 const Candidate& best = candidates.front();
 const Candidate* second = nullptr;
 for (std::size_t index = 1; index < candidates.size(); ++index) {
 if (separated(best, candidates[index], config_)) {
 second = &candidates[index];
 break;
 }
 }
 if (second && best.score - second->score < config_.minimum_score_margin) {
 result.state = autonomy::LocalizationState::Degraded;
 result.rejection_reason = "global map match is geometrically ambiguous";
 return result;
 }
 result.accepted = true;
 result.state = autonomy::LocalizationState::Localized;
 result.score = best.score;
 result.overlap = best.overlap;
 result.pose.position = best.position;
 result.pose.heading_rad = best.heading;
 result.pose.position_sigma_m = std::max(0.05, grid.resolution_m * 1.5);
 result.pose.heading_sigma_rad = std::max(0.05, config_.heading_step_rad * 1.5);
 result.pose.header = {timestamp_s, 0, "map", 0.0,
 kMeasurementSchemaVersion, "robot.global-known-map-lidar"};
 return result;
}

autonomy::LocalizationResult GlobalMapLocalizer::localize_unknown_start(
 const Scan& scan, const runtime::MapSnapshot& map, double timestamp_s) const {
 return localize(scan, {}, 0.0, map, timestamp_s);
}

} // namespace sim
