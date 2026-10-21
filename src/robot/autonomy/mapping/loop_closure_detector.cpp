#include "robot/autonomy/mapping/loop_closure_detector.hpp"

#include "robot/core/math.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace sim::autonomy {
namespace {

Vec2 rotate(Vec2 point, double heading) {
 const double c = std::cos(heading);
 const double s = std::sin(heading);
 return {c * point.x - s * point.y, s * point.x + c * point.y};
}

double ray_time_offset_s(const Scan& scan, std::size_t index) {
 if (index < scan.ray_time_offsets_s.size() &&
 std::isfinite(scan.ray_time_offsets_s[index]))
 return scan.ray_time_offsets_s[index];
 if (scan.time_increment_s > 0.0 && std::isfinite(scan.time_increment_s))
 return -scan.time_increment_s *
 static_cast<double>(scan.n_rays - 1 - static_cast<int>(index));
 if (scan.scan_period_s > 0.0 && std::isfinite(scan.scan_period_s) &&
 scan.n_rays > 1)
 return -scan.scan_period_s *
 static_cast<double>(scan.n_rays - 1 - static_cast<int>(index)) /
 static_cast<double>(scan.n_rays - 1);
 return 0.0;
}

// Endpoints of one scan expressed in its own keyframe frame, including the
// per-return motion correction. This is the same de-skew model the occupancy
// raster uses, so a verified loop closure measures the geometry that was
// actually mapped.
std::vector<Vec2> scan_endpoints(const MappingSessionKeyframe& keyframe,
 std::size_t maximum_returns) {
 const auto& scan = keyframe.scan;
 const auto& motion = keyframe.motion;
 std::vector<Vec2> endpoints;
 endpoints.reserve(scan.ranges.size());
 for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
 if (!scan.hit[index] || !std::isfinite(scan.ranges[index]) ||
 scan.ranges[index] <= 0.05 || scan.ranges[index] >= scan.max_range)
 continue;
 const double offset_s = ray_time_offset_s(scan, index);
 const double ray_heading = motion.angular_velocity_radps * offset_s;
 const Vec2 body_delta{motion.linear_velocity_mps * offset_s,
 motion.lateral_velocity_mps * offset_s};
 const double angle = ray_heading + scan.angle_min_rad +
 scan.angle_increment_rad * static_cast<double>(index);
 const double range = scan.ranges[index];
 endpoints.push_back({body_delta.x + range * std::cos(angle),
 body_delta.y + range * std::sin(angle)});
 }
 if (maximum_returns > 0 && endpoints.size() > maximum_returns) {
 const std::size_t stride =
 (endpoints.size() + maximum_returns - 1) / maximum_returns;
 std::vector<Vec2> selected;
 selected.reserve(maximum_returns);
 for (std::size_t index = 0; index < endpoints.size() &&
 selected.size() < maximum_returns; index += stride)
 selected.push_back(endpoints[index]);
 endpoints = std::move(selected);
 }
 return endpoints;
}

// Uniform grid over one keyframe scan for nearest-neighbour lookups.
class EndpointIndex {
 public:
 void build(const std::vector<Vec2>& points, double resolution) {
 cells_.clear();
 points_ = points;
 resolution_ = std::max(0.05, resolution);
 if (points.empty()) {
 width_ = 0;
 height_ = 0;
 return;
 }
 minimum_ = points.front();
 Vec2 maximum = points.front();
 for (const auto& point : points) {
 minimum_.x = std::min(minimum_.x, point.x);
 minimum_.y = std::min(minimum_.y, point.y);
 maximum.x = std::max(maximum.x, point.x);
 maximum.y = std::max(maximum.y, point.y);
 }
 width_ = static_cast<int>(std::floor((maximum.x - minimum_.x) / resolution_)) + 1;
 height_ = static_cast<int>(std::floor((maximum.y - minimum_.y) / resolution_)) + 1;
 width_ = std::clamp(width_, 1, 512);
 height_ = std::clamp(height_, 1, 512);
 cells_.assign(static_cast<std::size_t>(width_) * height_, {});
 for (std::size_t index = 0; index < points.size(); ++index) {
 const auto cell = cell_of(points[index]);
 cells_[static_cast<std::size_t>(cell.second) * width_ + cell.first]
 .push_back(index);
 }
 }

 double nearest(Vec2 point) const {
 if (cells_.empty())
 return std::numeric_limits<double>::infinity();
 const auto cell = cell_of(point);
 double best = std::numeric_limits<double>::infinity();
 for (int dy = -1; dy <= 1; ++dy) {
 for (int dx = -1; dx <= 1; ++dx) {
 const int x = cell.first + dx;
 const int y = cell.second + dy;
 if (x < 0 || y < 0 || x >= width_ || y >= height_)
 continue;
 for (const auto index : cells_[static_cast<std::size_t>(y) * width_ + x]) {
 const auto& candidate = points_[index];
 best = std::min(best, std::hypot(candidate.x - point.x,
 candidate.y - point.y));
 }
 }
 }
 return best;
 }

 private:
 std::pair<int, int> cell_of(Vec2 point) const {
 return {std::clamp(static_cast<int>(std::floor((point.x - minimum_.x) / resolution_)),
 0, width_ - 1),
 std::clamp(static_cast<int>(std::floor((point.y - minimum_.y) / resolution_)),
 0, height_ - 1)};
 }

 std::vector<Vec2> points_;
 std::vector<std::vector<std::size_t>> cells_;
 Vec2 minimum_;
 double resolution_{0.5};
 int width_{0};
 int height_{0};
};

struct Match {
 Pose2 relative;
 double score{-std::numeric_limits<double>::infinity()};
 double overlap{0.0};
};

} // namespace

std::optional<LoopClosureCandidate> LoopClosureDetector::detect(
 const std::vector<MappingSessionKeyframe>& keyframes) const {
 if (keyframes.size() < 2 || config_.maximum_returns == 0 ||
 !(config_.endpoint_tolerance_m > 0.0) ||
 !(config_.translation_step_m > 0.0) ||
 !(config_.heading_step_rad > 0.0) ||
 !(config_.search_translation_m >= 0.0) ||
 !(config_.search_heading_rad >= 0.0))
 return std::nullopt;
 const auto& query = keyframes.back();
 const auto query_points = scan_endpoints(query, config_.maximum_returns);
 if (query_points.size() < 12)
 return std::nullopt;

 std::optional<LoopClosureCandidate> best_candidate;
 double best_rank = -std::numeric_limits<double>::infinity();
 std::size_t considered = 0;
 for (std::size_t index = keyframes.size() - 1; index-- > 0;) {
 const auto& keyframe = keyframes[index];
 if (!std::isfinite(keyframe.graph.stamp_s) ||
 query.graph.stamp_s - keyframe.graph.stamp_s <
 config_.minimum_temporal_separation_s)
 continue;
 if (pose_translation_error(keyframe.graph.initial_pose,
 query.graph.initial_pose) > config_.candidate_search_distance_m)
 continue;
 if (considered++ >= config_.maximum_candidates)
 break;
 const auto reference_points = scan_endpoints(keyframe, config_.maximum_returns);
 if (reference_points.size() < 12)
 continue;
 EndpointIndex index_points;
 index_points.build(reference_points, config_.index_cell_m);
 const auto predicted = relative_pose(keyframe.graph.initial_pose,
 query.graph.initial_pose);
 const int translation_steps = static_cast<int>(std::ceil(
 config_.search_translation_m / config_.translation_step_m));
 const int heading_steps = static_cast<int>(std::ceil(
 config_.search_heading_rad / config_.heading_step_rad));
 std::vector<Match> matches;
 matches.reserve(static_cast<std::size_t>(2 * translation_steps + 1) *
 static_cast<std::size_t>(2 * translation_steps + 1) *
 static_cast<std::size_t>(2 * heading_steps + 1));
 for (int iy = -translation_steps; iy <= translation_steps; ++iy) {
 for (int ix = -translation_steps; ix <= translation_steps; ++ix) {
 for (int ih = -heading_steps; ih <= heading_steps; ++ih) {
 const Pose2 candidate{
 {predicted.position.x + ix * config_.translation_step_m,
 predicted.position.y + iy * config_.translation_step_m},
 wrap_angle(predicted.heading_rad + ih * config_.heading_step_rad)};
 double score = 0.0;
 std::size_t matched = 0;
 for (const auto& point : query_points) {
 const auto transformed = rotate(point, candidate.heading_rad);
 const Vec2 mapped{transformed.x + candidate.position.x,
 transformed.y + candidate.position.y};
 const double distance = index_points.nearest(mapped);
 if (distance <= config_.endpoint_tolerance_m) {
 ++matched;
 score += 1.0 - distance / config_.endpoint_tolerance_m;
 }
 }
 score /= static_cast<double>(query_points.size());
 matches.push_back({candidate, score,
 static_cast<double>(matched) / static_cast<double>(query_points.size())});
 }
 }
 }
 if (matches.empty())
 continue;
 // Two-stage verification: the forward score ranks every candidate cheaply,
 // then the strongest distinct hypotheses are checked in the reverse
 // direction. A one-sided match can align part of a scan against unrelated
 // geometry; requiring both scans to explain each other rejects that case.
 EndpointIndex query_index;
 query_index.build(query_points, config_.index_cell_m);
 const auto inverse = [](const Pose2& pose, Vec2 point) {
 const Vec2 shifted{point.x - pose.position.x, point.y - pose.position.y};
 const double c = std::cos(pose.heading_rad);
 const double s = std::sin(pose.heading_rad);
 return Vec2{c * shifted.x + s * shifted.y,
 -s * shifted.x + c * shifted.y};
 };
 constexpr std::size_t kReverseChecks = 8;
 std::size_t reverse_checked = 0;
 for (std::size_t candidate_index = 0;
 candidate_index < matches.size() && reverse_checked < kReverseChecks;
 ++candidate_index) {
 auto& match = matches[candidate_index];
 bool duplicate = false;
 for (std::size_t previous = 0; previous < candidate_index; ++previous) {
 if (std::hypot(matches[previous].relative.position.x -
 match.relative.position.x,
 matches[previous].relative.position.y -
 match.relative.position.y) < config_.ambiguity_distance_m &&
 std::abs(wrap_angle(matches[previous].relative.heading_rad -
 match.relative.heading_rad)) <
 config_.ambiguity_heading_rad) {
 duplicate = true;
 break;
 }
 }
 if (duplicate)
 continue;
 ++reverse_checked;
 double reverse_score = 0.0;
 std::size_t reverse_matches = 0;
 for (const auto& point : reference_points) {
 const double distance = query_index.nearest(inverse(match.relative, point));
 if (distance <= config_.endpoint_tolerance_m) {
 ++reverse_matches;
 reverse_score += 1.0 - distance / config_.endpoint_tolerance_m;
 }
 }
 reverse_score /= static_cast<double>(reference_points.size());
 const double reverse_overlap = static_cast<double>(reverse_matches) /
 static_cast<double>(reference_points.size());
 match.overlap = std::min(match.overlap, reverse_overlap);
 match.score = 0.5 * (match.score + reverse_score);
 }
 // Coarse-to-fine: refine the strongest distinct hypotheses so a wide drift
 // window does not cost final accuracy.
 const auto score_at = [&](const Pose2& candidate, double* overlap) {
 double score = 0.0;
 std::size_t matched = 0;
 for (const auto& point : query_points) {
 const auto transformed = rotate(point, candidate.heading_rad);
 const Vec2 mapped{transformed.x + candidate.position.x,
 transformed.y + candidate.position.y};
 const double distance = index_points.nearest(mapped);
 if (distance <= config_.endpoint_tolerance_m) {
 ++matched;
 score += 1.0 - distance / config_.endpoint_tolerance_m;
 }
 }
 if (overlap)
 *overlap = static_cast<double>(matched) /
 static_cast<double>(query_points.size());
 return score / static_cast<double>(query_points.size());
 };
 const int refine_steps = static_cast<int>(std::ceil(
 config_.refine_translation_m / config_.refine_translation_step_m));
 const int refine_heading_steps = static_cast<int>(std::ceil(
 config_.refine_heading_rad / config_.refine_heading_step_rad));
 std::vector<bool> refined(matches.size(), false);
 std::size_t refined_count = 0;
 for (std::size_t candidate_index = 0;
 candidate_index < matches.size() &&
 refined_count < config_.refined_hypotheses; ++candidate_index) {
 auto& match = matches[candidate_index];
 if (refined[candidate_index])
 continue;
 refined[candidate_index] = true;
 ++refined_count;
 Match best = match;
 for (int iy = -refine_steps; iy <= refine_steps; ++iy) {
 for (int ix = -refine_steps; ix <= refine_steps; ++ix) {
 for (int ih = -refine_heading_steps; ih <= refine_heading_steps; ++ih) {
 const Pose2 fine{
 {match.relative.position.x + ix * config_.refine_translation_step_m,
 match.relative.position.y + iy * config_.refine_translation_step_m},
 wrap_angle(match.relative.heading_rad +
 ih * config_.refine_heading_step_rad)};
 double fine_overlap = 0.0;
 const double fine_score = score_at(fine, &fine_overlap);
 if (fine_score > best.score) {
 best = {fine, fine_score, fine_overlap};
 }
 }
 }
 }
 match = best;
 }
 // Deterministic ordering, including exact ties.
 std::sort(matches.begin(), matches.end(), [](const auto& left, const auto& right) {
 if (left.score != right.score)
 return left.score > right.score;
 if (left.relative.position.x != right.relative.position.x)
 return left.relative.position.x < right.relative.position.x;
 if (left.relative.position.y != right.relative.position.y)
 return left.relative.position.y < right.relative.position.y;
 return left.relative.heading_rad < right.relative.heading_rad;
 });
 const Match& best = matches.front();
 const Match* second = nullptr;
 for (std::size_t other = 1; other < matches.size(); ++other) {
 const auto& candidate = matches[other];
 if (std::hypot(candidate.relative.position.x - best.relative.position.x,
 candidate.relative.position.y - best.relative.position.y) >=
 config_.ambiguity_distance_m ||
 std::abs(wrap_angle(candidate.relative.heading_rad -
 best.relative.heading_rad)) >= config_.ambiguity_heading_rad) {
 second = &candidate;
 break;
 }
 }
 if (!std::isfinite(best.score))
 continue;
 const double rank = best.score * best.overlap;
 if (rank <= best_rank)
 continue;
 best_rank = rank;
 LoopClosureCandidate candidate;
 candidate.from_keyframe = keyframe.graph.id;
 candidate.to_keyframe = query.graph.id;
 candidate.evidence.match_score = best.score;
 candidate.evidence.overlap = best.overlap;
 candidate.evidence.score_margin =
 second ? best.score - second->score : best.score;
 candidate.evidence.measured_relative_pose = best.relative;
 candidate.evidence.information = pose_graph_information(0.15, 0.05);
 candidate.diagnostic = "scan-verified loop candidate";
 best_candidate = std::move(candidate);
 }
 return best_candidate;
}

} // namespace sim::autonomy
