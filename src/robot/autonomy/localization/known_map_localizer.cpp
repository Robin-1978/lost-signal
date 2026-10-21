#include "robot/autonomy/localization/known_map_localizer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace sim {
namespace {

bool finite_pose(Vec2 p, double heading) {
 return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(heading);
}

Vec2 rotate(Vec2 p, double heading) {
 const double c = std::cos(heading);
 const double s = std::sin(heading);
 return {c * p.x - s * p.y, s * p.x + c * p.y};
}

Vec2 sensor_world_position(Vec2 base, double base_heading,
 const KnownMapLocalizationConfig& config) {
 const auto offset = rotate({config.sensor_x_m, config.sensor_y_m}, base_heading);
 return {base.x + offset.x, base.y + offset.y};
}

double endpoint_score(const runtime::MapGrid& grid, Vec2 point) {
 const double c = std::cos(grid.origin_yaw_rad);
 const double s = std::sin(grid.origin_yaw_rad);
 const double dx = point.x - grid.origin_m.x;
 const double dy = point.y - grid.origin_m.y;
 const int cell_x = static_cast<int>(std::floor((c * dx + s * dy) /
 grid.resolution_m));
 const int cell_y = static_cast<int>(std::floor((-s * dx + c * dy) /
 grid.resolution_m));
 if (cell_x < 0 || cell_y < 0 ||
 cell_x >= static_cast<int>(grid.width) || cell_y >= static_cast<int>(grid.height))
 return -0.25; // outside map is evidence against this pose.
 const auto index = static_cast<std::size_t>(cell_y) * grid.width +
 static_cast<std::size_t>(cell_x);
 if (index >= grid.occupancy.size())
 return -0.25;
 const int value = static_cast<int>(grid.occupancy[index]);
 if (value < 0)
 return 0.0; // unknown must not become false certainty.
 return value >= 50 ? 1.0 : -0.20;
}

} // namespace

KnownMapLocalizationResult KnownMapLocalizer::localize(
 const Scan& scan, const Vec2& odom_position, double odom_heading_rad,
 const runtime::MapSnapshot& map, double timestamp_s) const {
 KnownMapLocalizationResult result;
 result.state = autonomy::LocalizationState::Unlocalized;
 result.pose.header = {timestamp_s, 0, "map", 0.0,
 kMeasurementSchemaVersion, "robot.known-map-lidar"};
 if (!finite_pose(odom_position, odom_heading_rad)) {
 result.rejection_reason = "invalid odometry prior";
 result.state = autonomy::LocalizationState::Degraded;
 return result;
 }
 if (scan.n_rays <= 0 || scan.ranges.size() != static_cast<std::size_t>(scan.n_rays) ||
 scan.hit.size() != scan.ranges.size() || !(scan.max_range > 0.0) ||
 !std::isfinite(scan.header.stamp_s) ||
 std::abs(scan.header.stamp_s - timestamp_s) > 0.05) {
 result.rejection_reason = "invalid or stale scan";
 result.state = autonomy::LocalizationState::Degraded;
 return result;
 }
 if (!map.grid || map.frame_id != "map" || map.grid->width == 0 ||
 map.grid->height == 0 || !(map.grid->resolution_m > 0.0) ||
 map.grid->occupancy.size() !=
 static_cast<std::size_t>(map.grid->width) * map.grid->height) {
 result.rejection_reason = "map snapshot unavailable or invalid";
 result.state = autonomy::LocalizationState::Degraded;
 return result;
 }
 if (!(config_.search_translation_m >= 0.0) || !(config_.search_heading_rad >= 0.0) ||
 !(config_.translation_step_m > 0.0) || !(config_.heading_step_rad > 0.0) ||
 !(config_.minimum_score_margin >= 0.0) || config_.maximum_returns < config_.minimum_returns ||
 !(config_.ambiguity_distance_m > 0.0) ||
 !(config_.ambiguity_heading_rad > 0.0)) {
 result.rejection_reason = "invalid matcher configuration";
 result.state = autonomy::LocalizationState::Degraded;
 return result;
 }

 struct Return { double range; double angle; };
 std::vector<Return> returns;
 returns.reserve(scan.ranges.size());
 for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
 const double range = scan.ranges[i];
 if (!scan.hit[i] || !std::isfinite(range) || range <= 0.05 || range >= scan.max_range)
 continue;
 returns.push_back({range, scan.angle_min_rad +
 scan.angle_increment_rad * static_cast<double>(i)});
 }
 result.usable_returns = returns.size();
 if (returns.size() < config_.minimum_returns) {
 result.rejection_reason = "insufficient geometric returns";
 result.state = autonomy::LocalizationState::Degraded;
 return result;
 }
 // A spinning scan may contain 360/800 returns. A bounded matcher uses a
 // deterministic angular subset; the full scan remains available to the
 // perception and mapping providers. This keeps localization cost bounded as
 // sensor resolution increases and is the same coarse-signature principle as
 // the global recovery matcher.
 if (returns.size() > config_.maximum_returns) {
 const std::size_t stride =
 (returns.size() + config_.maximum_returns - 1) / config_.maximum_returns;
 std::vector<Return> selected;
 selected.reserve(config_.maximum_returns);
 for (std::size_t index = 0; index < returns.size() &&
 selected.size() < config_.maximum_returns; index += stride)
 selected.push_back(returns[index]);
 returns = std::move(selected);
 }

 const auto& grid = *map.grid;
 struct Hypothesis {
 Vec2 position;
 double heading_rad{0.0};
 double score{-std::numeric_limits<double>::infinity()};
 double overlap{0.0};
 };
 const auto same_hypothesis = [&](const Hypothesis& left, const Hypothesis& right) {
 return std::hypot(left.position.x - right.position.x,
 left.position.y - right.position.y) < config_.ambiguity_distance_m &&
 std::abs(wrap_angle(left.heading_rad - right.heading_rad)) <
 config_.ambiguity_heading_rad;
 };
 // The endpoint score is quantised: adjacent samples of one physical solution
 // routinely tie exactly. Keeping every tied sample would label a well
 // observed pose as ambiguous, so the search retains separated hypotheses and
 // measures the margin between distinct peaks.
 constexpr std::size_t kMaximumHypotheses = 24;
 std::vector<Hypothesis> hypotheses;
 hypotheses.reserve(kMaximumHypotheses);
 const int translation_steps = static_cast<int>(std::ceil(
 config_.search_translation_m / config_.translation_step_m));
 const int heading_steps = static_cast<int>(std::ceil(
 config_.search_heading_rad / config_.heading_step_rad));
 for (int iy = -translation_steps; iy <= translation_steps; ++iy) {
 for (int ix = -translation_steps; ix <= translation_steps; ++ix) {
 for (int ih = -heading_steps; ih <= heading_steps; ++ih) {
 const Vec2 candidate{odom_position.x + ix * config_.translation_step_m,
 odom_position.y + iy * config_.translation_step_m};
 const double heading = wrap_angle(odom_heading_rad + ih * config_.heading_step_rad);
 const Vec2 sensor_position = sensor_world_position(candidate, heading, config_);
 double score = 0.0;
 std::size_t known = 0;
 std::size_t matches = 0;
 for (const auto& point : returns) {
 const auto local = rotate({point.range * std::cos(point.angle + config_.sensor_yaw_rad),
 point.range * std::sin(point.angle + config_.sensor_yaw_rad)}, heading);
 const double value = endpoint_score(grid,
 {sensor_position.x + local.x, sensor_position.y + local.y});
 score += value;
 if (value != 0.0) {
 ++known;
 if (value > 0.0)
 ++matches;
 }
 }
 if (known == 0)
 continue;
 const double overlap = static_cast<double>(matches) /
 static_cast<double>(returns.size());
 score /= static_cast<double>(returns.size());
 const Hypothesis hypothesis{candidate, heading, score, overlap};
 const auto existing = std::find_if(hypotheses.begin(), hypotheses.end(),
 [&](const auto& value) { return same_hypothesis(value, hypothesis); });
 if (existing != hypotheses.end()) {
 if (hypothesis.score > existing->score)
 *existing = hypothesis;
 continue;
 }
 if (hypotheses.size() < kMaximumHypotheses) {
 hypotheses.push_back(hypothesis);
 continue;
 }
 const auto weakest = std::min_element(hypotheses.begin(), hypotheses.end(),
 [](const auto& left, const auto& right) {
 return left.score < right.score;
 });
 if (hypothesis.score > weakest->score)
 *weakest = hypothesis;
 }
 }
 }
 if (hypotheses.empty()) {
 result.rejection_reason = "map overlap below acceptance gate";
 result.state = autonomy::LocalizationState::Degraded;
 return result;
 }
 // Deterministic ordering, including exact ties, so a decision never depends
 // on evaluation order.
 std::sort(hypotheses.begin(), hypotheses.end(),
 [](const auto& left, const auto& right) {
 if (left.score != right.score)
 return left.score > right.score;
 if (left.position.x != right.position.x)
 return left.position.x < right.position.x;
 if (left.position.y != right.position.y)
 return left.position.y < right.position.y;
 return left.heading_rad < right.heading_rad;
 });
 const Hypothesis& best = hypotheses.front();
 const double second_score = hypotheses.size() > 1
 ? hypotheses[1].score
 : -std::numeric_limits<double>::infinity();
 if (!std::isfinite(best.score) || best.overlap < config_.minimum_overlap) {
 result.rejection_reason = "map overlap below acceptance gate";
 result.state = autonomy::LocalizationState::Degraded;
 return result;
 }
 // A nearly tied score is a degeneracy signal. Do not turn a repetitive
 // corridor into a confident map correction.
 if (std::isfinite(second_score) &&
 best.score - second_score < config_.minimum_score_margin) {
 result.score = best.score;
 result.overlap = best.overlap;
 result.score_margin = best.score - second_score;
 result.rejection_reason = "map match is geometrically ambiguous";
 result.state = autonomy::LocalizationState::Degraded;
 return result;
 }
 result.accepted = true;
 result.state = autonomy::LocalizationState::Localized;
 result.score = best.score;
 result.overlap = best.overlap;
 result.score_margin = std::isfinite(second_score) ? best.score - second_score : 1.0;
 result.pose.position = best.position;
 result.pose.heading_rad = best.heading_rad;
 result.pose.position_sigma_m = std::max(0.02, grid.resolution_m * 0.75);
 result.pose.heading_sigma_rad = std::max(0.01, config_.heading_step_rad * 1.5);
 result.pose.quality_score = std::clamp((best.score + 0.2) / 1.2, 0.0, 1.0);
 result.pose.quality_overlap = std::clamp(best.overlap, 0.0, 1.0);
 result.pose.quality_margin = std::clamp(result.score_margin, 0.0, 1.0);
 result.pose.header = {timestamp_s, 0, "map", 0.0,
 kMeasurementSchemaVersion, "robot.known-map-lidar"};
 return result;
}

} // namespace sim
