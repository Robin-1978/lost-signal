#include "robot/autonomy/localization/global_map_localizer.hpp"

#include "robot/autonomy/localization/scan_evidence.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace sim {
namespace {

using localization::Beam;

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

std::optional<double> expected_range(const runtime::MapGrid& grid,
 Vec2 sensor, double beam_angle, double max_range) {
 // A coarse ray cast used only for the final, bounded refinement. Sampling at
 // half a map cell prevents a thin occupied cell from being skipped.
 const double step = std::max(0.05, grid.resolution_m * 0.5);
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

struct RaycastEvidence {
 double score{-std::numeric_limits<double>::infinity()};
 double overlap{0.0};
};

// Validates a place hypothesis against free space as well as endpoints. The
// coarse passes cannot afford this test at every pose, but the few surviving
// hypotheses must answer the stronger question: would the map actually produce
// this scan from this pose?
RaycastEvidence raycast_evidence(const runtime::MapGrid& grid,
 const std::vector<Beam>& beams, Vec2 base, double heading,
 const GlobalMapLocalizationConfig& config, double max_range) {
 RaycastEvidence evidence;
 if (beams.empty())
 return evidence;
 const auto sensor = localization::sensor_position(base, heading,
 config.sensor_x_m, config.sensor_y_m);
 double score = 0.0;
 std::size_t matches = 0;
 for (const auto& beam : beams) {
 const double beam_angle = heading + beam.angle + config.sensor_yaw_rad;
 const auto expected = expected_range(grid, sensor, beam_angle, max_range);
 if (!expected) {
 // A ray leaving the map before the measured return is inconsistent with
 // the candidate. This is stronger evidence than the hit endpoint alone,
 // which is ambiguous in repetitive corridors.
 score -= 1.0;
 continue;
 }
 // Quantise to the raster resolution before scoring. Ray marching along
 // two geometrically equivalent directions accumulates slightly different
 // sub-cell rounding; that numerical difference must not decide whether a
 // symmetric map is ambiguous. The tolerance below is still the stricter
 // evidence gate for counting a return as matched.
 const double error = std::abs(*expected - beam.range);
 const double quantum = std::max(0.05, grid.resolution_m);
 const double quantised_error = std::round(error / quantum) * quantum;
 score += std::exp(-quantised_error / std::max(0.05, grid.resolution_m * 2.0));
 if (*expected < max_range &&
 error <= std::max(0.12, grid.resolution_m * 2.0))
 ++matches;
 }
 evidence.score = score / static_cast<double>(beams.size());
 evidence.overlap = static_cast<double>(matches) / static_cast<double>(beams.size());
 return evidence;
}

// A coarse occupancy raster with a distance-to-nearest-obstacle field. It turns
// the first place search into O(beams) lookups per pose instead of a ray cast
// across the scan range for every hypothesis.
struct DistanceField {
 std::uint32_t width{0};
 std::uint32_t height{0};
 double resolution_m{0.0};
 Vec2 origin_m;
 double origin_yaw_rad{0.0};
 std::vector<std::uint16_t> distance_cm;
 std::vector<std::uint8_t> free_cell;

 std::optional<std::size_t> index_of(Vec2 world) const {
 const double c = std::cos(origin_yaw_rad);
 const double s = std::sin(origin_yaw_rad);
 const double dx = world.x - origin_m.x;
 const double dy = world.y - origin_m.y;
 const double lx = c * dx + s * dy;
 const double ly = -s * dx + c * dy;
 const auto x = static_cast<std::int64_t>(std::floor(lx / resolution_m));
 const auto y = static_cast<std::int64_t>(std::floor(ly / resolution_m));
 if (x < 0 || y < 0 || x >= static_cast<std::int64_t>(width) ||
 y >= static_cast<std::int64_t>(height))
 return std::nullopt;
 return static_cast<std::size_t>(y) * width + static_cast<std::size_t>(x);
 }

 Vec2 centre(std::uint32_t x, std::uint32_t y) const {
 const double lx = (static_cast<double>(x) + 0.5) * resolution_m;
 const double ly = (static_cast<double>(y) + 0.5) * resolution_m;
 const double c = std::cos(origin_yaw_rad);
 const double s = std::sin(origin_yaw_rad);
 return {origin_m.x + c * lx - s * ly, origin_m.y + s * lx + c * ly};
 }

 bool cell_centre(Vec2 world, std::uint32_t* x, std::uint32_t* y) const {
 const double c = std::cos(origin_yaw_rad);
 const double s = std::sin(origin_yaw_rad);
 const double dx = world.x - origin_m.x;
 const double dy = world.y - origin_m.y;
 const double lx = c * dx + s * dy;
 const double ly = -s * dx + c * dy;
 const auto cx = static_cast<std::int64_t>(std::floor(lx / resolution_m));
 const auto cy = static_cast<std::int64_t>(std::floor(ly / resolution_m));
 if (cx < 0 || cy < 0 || cx >= static_cast<std::int64_t>(width) ||
 cy >= static_cast<std::int64_t>(height))
 return false;
 *x = static_cast<std::uint32_t>(cx);
 *y = static_cast<std::uint32_t>(cy);
 return true;
 }
};

// Builds the coarse raster and a chamfer distance transform. Both passes are
// deterministic and independent of evaluation order, so a repeated cold start
// on the same map revision produces the same field and the same ranking.
DistanceField build_distance_field(const runtime::MapGrid& grid,
 double requested_resolution) {
 DistanceField field;
 field.resolution_m = std::max(requested_resolution, grid.resolution_m);
 field.origin_m = grid.origin_m;
 field.origin_yaw_rad = grid.origin_yaw_rad;
 field.width = static_cast<std::uint32_t>(std::ceil(
 static_cast<double>(grid.width) * grid.resolution_m / field.resolution_m));
 field.height = static_cast<std::uint32_t>(std::ceil(
 static_cast<double>(grid.height) * grid.resolution_m / field.resolution_m));
 if (field.width == 0 || field.height == 0)
 return field;
 const std::size_t cells = static_cast<std::size_t>(field.width) * field.height;
 std::vector<std::uint8_t> occupied(cells, 0);
 std::vector<std::uint8_t> known(cells, 0);
 for (std::uint32_t y = 0; y < grid.height; ++y) {
 for (std::uint32_t x = 0; x < grid.width; ++x) {
 const auto source = static_cast<std::size_t>(y) * grid.width + x;
 if (source >= grid.occupancy.size())
 continue;
 const auto cx = static_cast<std::uint32_t>(std::floor(
 (static_cast<double>(x) + 0.5) * grid.resolution_m / field.resolution_m));
 const auto cy = static_cast<std::uint32_t>(std::floor(
 (static_cast<double>(y) + 0.5) * grid.resolution_m / field.resolution_m));
 if (cx >= field.width || cy >= field.height)
 continue;
 const auto target = static_cast<std::size_t>(cy) * field.width + cx;
 const int value = static_cast<int>(grid.occupancy[source]);
 if (value >= 50)
 occupied[target] = 1;
 if (value >= 0)
 known[target] = 1;
 }
 }
 constexpr float kInfinity = 1.0e9F;
 std::vector<float> distance(cells, kInfinity);
 field.free_cell.assign(cells, 0);
 for (std::size_t index = 0; index < cells; ++index) {
 if (occupied[index])
 distance[index] = 0.0F;
 else if (known[index])
 field.free_cell[index] = 1;
 }
 constexpr float kDiagonal = 1.41421356F;
 for (std::uint32_t y = 0; y < field.height; ++y) {
 for (std::uint32_t x = 0; x < field.width; ++x) {
 const auto index = static_cast<std::size_t>(y) * field.width + x;
 float value = distance[index];
 if (x > 0)
 value = std::min(value, distance[index - 1] + 1.0F);
 if (y > 0) {
 value = std::min(value, distance[index - field.width] + 1.0F);
 if (x > 0)
 value = std::min(value, distance[index - field.width - 1] + kDiagonal);
 if (x + 1 < field.width)
 value = std::min(value, distance[index - field.width + 1] + kDiagonal);
 }
 distance[index] = value;
 }
 }
 for (std::int64_t y = static_cast<std::int64_t>(field.height) - 1; y >= 0; --y) {
 for (std::int64_t x = static_cast<std::int64_t>(field.width) - 1; x >= 0; --x) {
 const auto index = static_cast<std::size_t>(y) * field.width + x;
 float value = distance[index];
 if (x + 1 < static_cast<std::int64_t>(field.width))
 value = std::min(value, distance[index + 1] + 1.0F);
 if (y + 1 < static_cast<std::int64_t>(field.height)) {
 value = std::min(value, distance[index + field.width] + 1.0F);
 if (x + 1 < static_cast<std::int64_t>(field.width))
 value = std::min(value, distance[index + field.width + 1] + kDiagonal);
 if (x > 0)
 value = std::min(value, distance[index + field.width - 1] + kDiagonal);
 }
 distance[index] = value;
 }
 }
 field.distance_cm.resize(cells);
 for (std::size_t index = 0; index < cells; ++index) {
 const double metres = static_cast<double>(distance[index]) * field.resolution_m;
 field.distance_cm[index] = static_cast<std::uint16_t>(std::clamp(
 metres * 100.0, 0.0, 65535.0));
 }
 return field;
}

// 256 buckets of 2 cm up to 5.1 m. Beyond that an endpoint is treated as
// unsupported rather than as a strong negative, because a legitimate scan may
// fall outside a partially surveyed coarse cell.
std::vector<float> build_lookup(double sigma_m) {
 std::vector<float> lookup(256, 0.0F);
 const double sigma = std::max(0.05, sigma_m);
 for (std::size_t index = 0; index < lookup.size(); ++index) {
 const double distance = 0.02 * static_cast<double>(index);
 const double ratio = distance / sigma;
 lookup[index] = static_cast<float>(std::exp(-ratio * ratio));
 }
 lookup.back() = -0.05F;
 return lookup;
}

void retain_candidate(std::vector<Candidate>& candidates, Candidate candidate,
 std::size_t limit, double minimum_distance_m, double minimum_heading_rad) {
 for (auto& existing : candidates) {
 const bool same_place =
 std::hypot(existing.position.x - candidate.position.x,
 existing.position.y - candidate.position.y) < minimum_distance_m &&
 std::abs(wrap_angle(existing.heading - candidate.heading)) <
 minimum_heading_rad;
 if (same_place) {
 if (candidate.score > existing.score)
 existing = candidate;
 return;
 }
 }
 if (candidates.size() < limit) {
 candidates.push_back(candidate);
 return;
 }
 const auto weakest = std::min_element(candidates.begin(), candidates.end(),
 [](const auto& left, const auto& right) {
 return left.score < right.score;
 });
 if (candidate.score > weakest->score)
 *weakest = candidate;
}

bool separated(const Candidate& first, const Candidate& second,
 const GlobalMapLocalizationConfig& config) {
 return std::hypot(first.position.x - second.position.x,
 first.position.y - second.position.y) >= config.ambiguity_distance_m ||
 std::abs(wrap_angle(first.heading - second.heading)) >=
 config.ambiguity_heading_rad;
}

} // namespace

autonomy::LocalizationResult GlobalMapLocalizer::localize(
 const Scan& scan, const Vec2& odom_position, double odom_heading_rad,
 const runtime::MapSnapshot& map, double timestamp_s) const {
 autonomy::LocalizationResult result;
 result.pose.header = {timestamp_s, 0, "map", 0.0,
 kMeasurementSchemaVersion, "robot.global-known-map-lidar"};
 const auto reject = [&](std::string reason) {
 result.accepted = false;
 result.state = autonomy::LocalizationState::Degraded;
 result.rejection_reason = std::move(reason);
 return result;
 };
 if (!finite_pose(odom_position, odom_heading_rad))
 return reject("invalid odometry prior");
 if (scan.n_rays <= 0 || scan.ranges.size() != static_cast<std::size_t>(scan.n_rays) ||
 scan.hit.size() != scan.ranges.size() || !(scan.max_range > 0.0) ||
 !std::isfinite(scan.header.stamp_s) ||
 std::abs(scan.header.stamp_s - timestamp_s) > 0.05)
 return reject("invalid or stale scan");
 if (!map.grid || map.frame_id != "map" || map.grid->width == 0 ||
 map.grid->height == 0 || !(map.grid->resolution_m > 0.0) ||
 map.grid->occupancy.size() !=
 static_cast<std::size_t>(map.grid->width) * map.grid->height)
 return reject("map snapshot unavailable or invalid");
 if (!(config_.coarse_field_resolution_m > 0.0) ||
 !(config_.coarse_position_step_m > 0.0) ||
 !(config_.coarse_heading_step_rad > 0.0) ||
 !(config_.coarse_score_sigma_m > 0.0) ||
 config_.coarse_returns == 0 || config_.coarse_hypotheses == 0 ||
 !(config_.refine_translation_m >= 0.0) ||
 !(config_.refine_translation_step_m > 0.0) ||
 !(config_.refine_heading_step_rad > 0.0) ||
 !(config_.final_translation_step_m > 0.0) ||
 !(config_.final_heading_step_rad > 0.0) ||
 !(config_.minimum_overlap >= 0.0 && config_.minimum_overlap <= 1.0) ||
 !(config_.minimum_score_margin >= 0.0) || config_.minimum_returns == 0 ||
 !(config_.equivalent_support_overlap_tolerance >= 0.0 &&
 config_.equivalent_support_overlap_tolerance <= 1.0) ||
 !(config_.equivalent_support_margin >= 0.0) ||
 config_.maximum_returns < config_.minimum_returns ||
 config_.maximum_evaluations == 0)
 return reject("invalid global matcher configuration");

 std::size_t hit_returns = 0;
 const auto all_beams = localization::collect_beams(scan, 0, &hit_returns);
 result.usable_returns = hit_returns;
 if (hit_returns < config_.minimum_returns)
 return reject("insufficient geometric returns");

 const auto& grid = *map.grid;
 const auto field = build_distance_field(grid, config_.coarse_field_resolution_m);
 if (field.width == 0 || field.height == 0)
 return reject("map snapshot unavailable or invalid");

 std::vector<Beam> coarse_beams;
 {
 std::size_t hits = 0;
 for (const auto& beam : all_beams)
 if (beam.hit)
 ++hits;
 const std::size_t stride = std::max<std::size_t>(
 1, (hits + config_.coarse_returns - 1) / config_.coarse_returns);
 std::size_t seen = 0;
 for (const auto& beam : all_beams) {
 if (!beam.hit)
 continue;
 if (seen++ % stride != 0)
 continue;
 coarse_beams.push_back(beam);
 if (coarse_beams.size() >= config_.coarse_returns)
 break;
 }
 }
 if (coarse_beams.empty())
 return reject("insufficient geometric returns");

 const std::size_t heading_count = std::max<std::size_t>(1, static_cast<std::size_t>(
 std::ceil(2.0 * kPi / config_.coarse_heading_step_rad)));
 std::vector<Vec2> directions;
 directions.reserve(heading_count * coarse_beams.size());
 for (std::size_t heading_index = 0; heading_index < heading_count; ++heading_index) {
 const double heading = wrap_angle(-kPi + static_cast<double>(heading_index) *
 config_.coarse_heading_step_rad);
 for (const auto& beam : coarse_beams) {
 const double angle = heading + beam.angle + config_.sensor_yaw_rad;
 directions.push_back({std::cos(angle), std::sin(angle)});
 }
 }
 const auto lookup = build_lookup(config_.coarse_score_sigma_m);
 const auto evaluate_coarse = [&](Vec2 base, double heading,
 const Vec2* heading_directions) {
 const double c = std::cos(heading);
 const double s = std::sin(heading);
 const Vec2 mount{config_.sensor_x_m, config_.sensor_y_m};
 const Vec2 mount_world{c * mount.x - s * mount.y, s * mount.x + c * mount.y};
 const Vec2 sensor_origin{base.x + mount_world.x, base.y + mount_world.y};
 double score = 0.0;
 std::size_t compared = 0;
 for (std::size_t index = 0; index < coarse_beams.size(); ++index) {
 const auto& direction = heading_directions[index];
 const Vec2 endpoint{sensor_origin.x + coarse_beams[index].range * direction.x,
 sensor_origin.y + coarse_beams[index].range * direction.y};
 const auto cell = field.index_of(endpoint);
 if (!cell) {
 score -= 0.25;
 ++compared;
 continue;
 }
 const double distance_cm = field.distance_cm[*cell];
 const auto bucket = static_cast<std::size_t>(distance_cm / 2.0);
 score += lookup[std::min(bucket, lookup.size() - 1)];
 ++compared;
 }
 return compared == 0 ? 0.0 : score / static_cast<double>(compared);
 };

 // First pass: every coarse place on the map, at a bounded heading resolution.
 std::vector<Candidate> coarse;
 coarse.reserve(config_.coarse_hypotheses);
 std::size_t evaluations = 0;
 const auto position_stride = std::max<std::uint32_t>(1, static_cast<std::uint32_t>(
 std::ceil(config_.coarse_position_step_m / field.resolution_m)));
 for (std::uint32_t y = 0; y < field.height; y += position_stride) {
 for (std::uint32_t x = 0; x < field.width; x += position_stride) {
 const auto cell_index = static_cast<std::size_t>(y) * field.width + x;
 if (!field.free_cell[cell_index])
 continue;
 const Vec2 base = field.centre(x, y);
 for (std::size_t heading_index = 0; heading_index < heading_count;
 ++heading_index) {
 if (++evaluations > config_.maximum_evaluations)
 return reject("global search budget exceeded");
 const double heading = wrap_angle(-kPi + static_cast<double>(heading_index) *
 config_.coarse_heading_step_rad);
 const auto score = evaluate_coarse(base, heading,
 &directions[heading_index * coarse_beams.size()]);
 retain_candidate(coarse, {base, heading, score, 0.0},
 config_.coarse_hypotheses, 1.5, 0.45);
 }
 }
 }
 if (coarse.empty())
 return reject("map overlap below acceptance gate");

 // Second pass: refine each place hypothesis on the distance field. Position
 // steps are finer than the field raster so the result is not limited to the
 // coarse cell centre.
 std::vector<Candidate> refined;
 refined.reserve(coarse.size());
 const int refine_steps = static_cast<int>(std::ceil(
 config_.refine_translation_m / config_.refine_translation_step_m));
 const int refine_heading_steps = static_cast<int>(std::ceil(
 config_.coarse_heading_step_rad / config_.refine_heading_step_rad));
 for (const auto& seed : coarse) {
 Candidate best = seed;
 best.score = -std::numeric_limits<double>::infinity();
 std::vector<Vec2> local_directions(coarse_beams.size());
 for (int ih = -refine_heading_steps; ih <= refine_heading_steps; ++ih) {
 const double heading = wrap_angle(
 seed.heading + ih * config_.refine_heading_step_rad);
 for (std::size_t index = 0; index < coarse_beams.size(); ++index) {
 const double angle = heading + coarse_beams[index].angle + config_.sensor_yaw_rad;
 local_directions[index] = {std::cos(angle), std::sin(angle)};
 }
 for (int iy = -refine_steps; iy <= refine_steps; ++iy) {
 for (int ix = -refine_steps; ix <= refine_steps; ++ix) {
 const Vec2 base{seed.position.x + ix * config_.refine_translation_step_m,
 seed.position.y + iy * config_.refine_translation_step_m};
 const auto score = evaluate_coarse(base, heading, local_directions.data());
 if (score > best.score) {
 best.position = base;
 best.heading = heading;
 best.score = score;
 }
 }
 }
 }
 refined.push_back(best);
 }

 // Third pass: validate the surviving place hypotheses against the full
 // raster, including free space along every sampled ray. This is the same
 // evidence contract the continuous local matcher applies to its final pose.
 std::vector<Beam> final_beams;
 {
 std::size_t seen = 0;
 const auto stride = std::max<std::size_t>(1,
 (hit_returns + config_.maximum_returns - 1) / config_.maximum_returns);
 for (const auto& beam : all_beams) {
 if (!beam.hit)
 continue;
 if (seen++ % stride != 0)
 continue;
 final_beams.push_back(beam);
 if (final_beams.size() >= config_.maximum_returns)
 break;
 }
 }
 const int final_steps = static_cast<int>(std::ceil(
 config_.final_translation_m / config_.final_translation_step_m));
 const int final_heading_steps = static_cast<int>(std::ceil(
 config_.final_heading_rad / config_.final_heading_step_rad));
 std::vector<Candidate> final_candidates;
 final_candidates.reserve(refined.size());
 for (const auto& seed : refined) {
 Candidate best;
 best.score = -std::numeric_limits<double>::infinity();
 best.position = seed.position;
 best.heading = seed.heading;
 for (int iy = -final_steps; iy <= final_steps; ++iy) {
 for (int ix = -final_steps; ix <= final_steps; ++ix) {
 const Vec2 base{seed.position.x + ix * config_.final_translation_step_m,
 seed.position.y + iy * config_.final_translation_step_m};
 for (int ih = -final_heading_steps; ih <= final_heading_steps; ++ih) {
 const double heading = wrap_angle(
 seed.heading + ih * config_.final_heading_step_rad);
 const auto evidence = raycast_evidence(grid, final_beams, base, heading,
 config_, scan.max_range);
 if (evidence.score > best.score) {
 best.position = base;
 best.heading = heading;
 best.score = evidence.score;
 best.overlap = evidence.overlap;
 }
 }
 }
 }
 if (!std::isfinite(best.score))
 continue;
 retain_candidate(final_candidates, best, config_.coarse_hypotheses,
 config_.ambiguity_distance_m, config_.ambiguity_heading_rad);
 }
 if (final_candidates.empty())
 return reject("map overlap below acceptance gate");
 std::sort(final_candidates.begin(), final_candidates.end(),
 [](const auto& left, const auto& right) {
 if (left.score != right.score)
 return left.score > right.score;
 if (left.position.x != right.position.x)
 return left.position.x < right.position.x;
 if (left.position.y != right.position.y)
 return left.position.y < right.position.y;
 return left.heading < right.heading;
 });
 const Candidate& best = final_candidates.front();
 const Candidate* second = nullptr;
 for (std::size_t index = 1; index < final_candidates.size(); ++index) {
 if (separated(best, final_candidates[index], config_)) {
 second = &final_candidates[index];
 break;
 }
 }
 result.score = best.score;
 result.overlap = best.overlap;
 if (!std::isfinite(best.score) || best.overlap < config_.minimum_overlap)
 return reject("global map overlap below acceptance gate");
 result.score_margin = second ? best.score - second->score : 1.0;
 // A second hypothesis with essentially the same occupied-return support is
 // an unobservable pose even when endpoint quantisation gives it a small
 // numerical score lead. Treat that case as ambiguous; otherwise a symmetric
 // hall can publish one arbitrary quarter-turn as a global fix.
 const bool equivalent_support = second &&
 second->overlap >= best.overlap - config_.equivalent_support_overlap_tolerance;
 if (second && (best.score - second->score < config_.minimum_score_margin ||
 (equivalent_support &&
 best.score - second->score < config_.equivalent_support_margin)))
 return reject("global map match is geometrically ambiguous");

 result.accepted = true;
 result.state = autonomy::LocalizationState::Localized;
 result.pose.position = best.position;
 result.pose.heading_rad = best.heading;
 result.pose.position_sigma_m = std::max(0.05, grid.resolution_m * 1.5);
 result.pose.heading_sigma_rad = std::max(0.02, config_.final_heading_step_rad * 1.5);
 result.pose.quality_score = std::clamp((best.score + 0.2) / 1.2, 0.0, 1.0);
 result.pose.quality_overlap = std::clamp(best.overlap, 0.0, 1.0);
 result.pose.quality_margin = std::clamp(result.score_margin, 0.0, 1.0);
 result.pose.header = {timestamp_s, 0, "map", 0.0,
 kMeasurementSchemaVersion, "robot.global-known-map-lidar"};
 return result;
}

autonomy::LocalizationResult GlobalMapLocalizer::localize_unknown_start(
 const Scan& scan, const runtime::MapSnapshot& map, double timestamp_s) const {
 return localize(scan, {}, 0.0, map, timestamp_s);
}

} // namespace sim
