#include "robot/autonomy/localization/global_search.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <new>
#include <optional>
#include <vector>

#include "robot/core/geometry.hpp"
#include "robot/core/transform.hpp"

namespace lost_signal::robot::autonomy {

namespace {

// Runner-ups for deferred startup disambiguation: refined[0] is the
// published pose; up to 3 following distinct hypotheses ride along
// (best-first, already sorted). Bounded and deterministic.
void fill_runner_ups(GlobalSearchResult& result,
                     const std::vector<ScanMatchResult>& refined) {
  result.runner_ups.clear();
  for (std::size_t i = 1; i < refined.size() && result.runner_ups.size() < 3;
       ++i) {
    GlobalSearchResult::RunnerUp ru;
    ru.x_m = refined[i].x_m;
    ru.y_m = refined[i].y_m;
    ru.yaw_rad = refined[i].yaw_rad;
    ru.score = refined[i].score;
    result.runner_ups.push_back(ru);
  }
}

// Expected-range raycast evidence (master's final-stage check, ported):
// would the published survey actually produce this scan from the candidate
// pose? Rays march through unknown cells (only occupied ends a ray); a ray
// that leaves the raster before the measured return is inconsistent with
// the candidate — stronger than the endpoint alone, which ties along
// repetitive walls. Inputs are ROBOT-side only (candidate pose, prepared
// beams, survey snapshot, TF mount): no simulator truth enters here.
// Sensor-pose math mirrors ScanMatcher::Batch::rot_for exactly (body
// attitude composed with the mount; the 2D raster takes the x/y
// projection), so the two scoring paths cannot disagree on geometry.
struct RaycastEvidence {
  double score = -std::numeric_limits<double>::infinity();
  bool has_evidence = false;
  double overlap = 0.0;
};

std::optional<double> expected_range_cs(const iface::MapSnapshot& map,
                                        double sx, double sy, double dx,
                                        double dy, double max_range) {
  const double step = std::max(0.05, map.resolution_m * 0.5);
  for (double d = step; d < max_range; d += step) {
    const double wx = sx + d * dx;
    const double wy = sy + d * dy;
    const int gx = static_cast<int>(
        std::floor((wx - map.origin_x_m) / map.resolution_m));
    const int gy = static_cast<int>(
        std::floor((wy - map.origin_y_m) / map.resolution_m));
    if (gx < 0 || gy < 0 || gx >= static_cast<int>(map.width) ||
        gy >= static_cast<int>(map.height)) {
      return std::nullopt;
    }
    if (map.occupancy[static_cast<std::size_t>(gy) * map.width + gx] >= 50) {
      return d;
    }
  }
  return max_range;
}

RaycastEvidence raycast_evidence(
    const iface::MapSnapshot& map, const std::vector<ScanBeam>& beams,
    double base_x_m, double base_y_m, double yaw_rad, double body_roll_rad,
    double body_pitch_rad, const core::RigidTransform& base_from_lidar,
    double max_range_m) {
  RaycastEvidence out;
  if (beams.empty() || !(max_range_m > 0.0)) return out;
  const core::Quaternion body_q =
      core::quaternion_from_rpy(body_roll_rad, body_pitch_rad, yaw_rad);
  const core::Quaternion sensor_q =
      core::quaternion_multiply(body_q, base_from_lidar.rotation);
  const core::Vec3 mount_off =
      core::rotate_vector(body_q, base_from_lidar.translation_m);
  const double sx = base_x_m + mount_off.x;
  const double sy = base_y_m + mount_off.y;
  double score = 0.0;
  std::size_t matches = 0;
  for (const auto& beam : beams) {
    const core::Vec3 dir = core::rotate_vector(
        sensor_q, {std::cos(beam.angle), std::sin(beam.angle), 0.0});
    const double az = std::atan2(dir.y, dir.x);
    const double expected =
        expected_range_cs(map, sx, sy, std::cos(az), std::sin(az),
                          max_range_m)
            .value_or(-1.0);
    if (!(expected >= 0.0)) {
      score -= 1.0;
      continue;
    }
    // Quantise to the raster resolution before scoring: ray marching along
    // two geometrically equivalent directions accumulates slightly
    // different sub-cell rounding; that numerical difference must not
    // decide whether a symmetric map is ambiguous.
    const double error = std::abs(expected - beam.range);
    const double quantum = std::max(0.05, map.resolution_m);
    const double quantised_error =
        std::round(error / quantum) * quantum;
    score += std::exp(-quantised_error / std::max(0.05, 2.0 * map.resolution_m));
    if (expected < max_range_m &&
        error <= std::max(0.12, 2.0 * map.resolution_m)) {
      ++matches;
    }
  }
  out.has_evidence = true;
  out.overlap =
      static_cast<double>(matches) / static_cast<double>(beams.size());
  out.score = score / static_cast<double>(beams.size());
  return out;
}

}  // namespace

struct GlobalSearcher::PreparedMap {
 explicit PreparedMap(const iface::MapSnapshot& map)
 : map_id(map.map_id), revision(map.revision), width(map.width),
 height(map.height), resolution_m(map.resolution_m),
 origin_x_m(map.origin_x_m), origin_y_m(map.origin_y_m) {
 const std::size_t prefix_width = static_cast<std::size_t>(width) + 1;
 const std::size_t prefix_height = static_cast<std::size_t>(height) + 1;
 valid = prefix_width <= std::numeric_limits<std::size_t>::max() /
 prefix_height;
 }

 bool allocate() {
 if (!valid) return false;
 if (occupied_prefix != nullptr && distance_cm != nullptr) return true;
 const std::size_t prefix_width = static_cast<std::size_t>(width) + 1;
 const std::size_t prefix_height = static_cast<std::size_t>(height) + 1;
 // Allocate storage once, then touch it only inside the cooperative prepare
 // phases. Value-initialising multi-million-cell vectors here caused the
 // control tick that started recovery to absorb two hidden full-raster
 // writes. The dynamic arrays are intentionally uninitialised: forward
 // preparation assigns every distance cell, and the summed-area recurrence
 // only needs its top row and left column seeded to zero.
 const std::size_t prefix_count = prefix_width * prefix_height;
 const std::size_t cell_count = static_cast<std::size_t>(width) * height;
 occupied_prefix.reset(new (std::nothrow) std::uint32_t[prefix_count]);
 distance_cm.reset(new (std::nothrow) std::uint16_t[cell_count]);
 if (occupied_prefix == nullptr || distance_cm == nullptr) {
 occupied_prefix.reset();
 distance_cm.reset();
 return false;
 }
 std::fill_n(occupied_prefix.get(), prefix_width, 0U);
 for (std::size_t y = 1; y < prefix_height; ++y) {
 occupied_prefix[y * prefix_width] = 0U;
 }
 const auto bounded_centimetres = [](double value_m) {
 return static_cast<std::uint16_t>(std::clamp(
 std::lround(value_m * 100.0), 1L,
 static_cast<long>(std::numeric_limits<std::uint16_t>::max())));
 };
 distance_orthogonal_cm = bounded_centimetres(resolution_m);
 distance_diagonal_cm = std::max(
 distance_orthogonal_cm,
 bounded_centimetres(resolution_m * std::sqrt(2.0)));
 return true;
 }

 bool matches(const iface::MapSnapshot& map) const {
 if (width != map.width || height != map.height ||
 resolution_m != map.resolution_m || origin_x_m != map.origin_x_m ||
 origin_y_m != map.origin_y_m) {
 return false;
 }
 if (!map_id.empty() && !map.map_id.empty()) {
 // IMapProvider's contract makes a named id/revision immutable. Geometry is
 // still checked above so malformed test fixtures cannot reuse an
 // incompatible allocation; changing occupancy under the same named key is
 // a provider contract violation and must publish a new revision.
 return map_id == map.map_id && revision == map.revision;
 }
 // Anonymous snapshots cannot provide a stable O(1) immutable identity.
 // Disable reuse instead of hashing every cell synchronously in begin(),
 // which would bypass the cooperative per-tick preparation budget.
 return false;
 }

 std::string map_id;
 std::uint64_t revision = 0;
 std::uint32_t width = 0;
 std::uint32_t height = 0;
 double resolution_m = 0.0;
 double origin_x_m = 0.0;
 double origin_y_m = 0.0;
 bool valid = false;
 bool forward_complete = false;
 bool backward_complete = false;
 std::size_t prepare_cursor = 0;
 std::unique_ptr<std::uint32_t[]> occupied_prefix;
 std::unique_ptr<std::uint16_t[]> distance_cm;
 std::uint16_t distance_orthogonal_cm = 1;
 std::uint16_t distance_diagonal_cm = 1;
};

struct GlobalSearcher::WorkState {
 enum class Phase {
 kPrepareForward,
 kPrepareBackward,
 kCoarse,
 kRefine,
 kFinal,
 kDone
 };
 struct Coarse {
 double x = 0.0, y = 0.0, heading = 0.0, score = 0.0, overlap = 0.0;
 };

 WorkState(const GlobalSearchConfig& cfg,
 const core::RigidTransform& mount_value,
 std::shared_ptr<PreparedMap> prepared_value,
 const std::vector<float>& range_values, double angle_min,
 double angle_increment, const iface::MapSnapshot& map_value,
 core::MonotonicNs stamp, double roll, double pitch, double z,
 bool bound_position, double bound_x_m, double bound_y_m,
 double bound_radius_m, bool bound_heading,
 double bound_heading_rad, double bound_heading_radius_rad)
 : config(cfg), mount(mount_value), prepared(std::move(prepared_value)),
 ranges(range_values), map(map_value),
 angle_min_rad(angle_min), angle_increment_rad(angle_increment),
 stamp_ns(stamp), body_roll_rad(roll), body_pitch_rad(pitch), body_z_m(z) {
 result.stamp_ns = stamp;
 if (ranges.empty() || !(angle_increment_rad > 0.0)) {
 finish("invalid scan");
 return;
 }
 if (!map.has_raster()) {
 finish("map snapshot has no raster");
 return;
 }
 if (prepared == nullptr) {
 finish("map index allocation failed");
 return;
 }
 if (!prepared->valid) {
 finish("map dimensions overflow observability index");
 return;
 }
 if (config.preparation_cells_per_cycle == 0 ||
 !(config.coarse_position_step_m > 0.0) ||
 !(config.coarse_heading_step_rad > 0.0) ||
 !(config.coarse_score_sigma_m > 0.0) || config.coarse_beams == 0 ||
 config.coarse_hypotheses == 0 ||
 !(config.coarse_hypothesis_distance_m > 0.0) ||
 !(config.coarse_hypothesis_heading_rad > 0.0) ||
 !(config.refine_translation_m >= 0.0) ||
 !(config.refine_translation_step_m > 0.0) ||
 !(config.refine_heading_rad >= 0.0) ||
 !(config.refine_heading_step_rad > 0.0) ||
 !(config.global_ambiguity_distance_m > 0.0) ||
 !(config.global_ambiguity_heading_rad > 0.0) ||
 !(config.equivalent_support_score_margin >= 0.0) ||
 !(config.final_translation_m >= 0.0) ||
 !(config.final_translation_step_m > 0.0) ||
 !(config.final_heading_rad >= 0.0) ||
 !(config.final_heading_step_rad > 0.0) ||
 config.final_max_returns == 0) {
 finish("invalid global search configuration");
 return;
 }
 ScanMatcherConfig coarse_cfg;
 coarse_cfg.max_range_m = config.max_range_m;
 coarse_cfg.minimum_returns = config.minimum_returns;
 coarse_matcher = ScanMatcher(coarse_cfg);
 coarse_matcher.set_lidar_mount(mount);
 const std::vector<ScanBeam> full = coarse_matcher.prepare_beams(
 ranges, angle_min_rad, angle_increment_rad, ranges.size());
 for (const auto& beam : full) {
 if (beam.hit) ++result.usable_returns;
 }
 {
  // Effective support is the configured baseline return count tempered by
  // the scan's angular second moment.  It is a property of the frozen
  // ROBOT-side measurement and is independent of the candidate pose.
  double c2 = 0.0;
  double s2 = 0.0;
  for (const auto& beam : full) {
   if (!beam.hit) continue;
   c2 += std::cos(2.0 * beam.angle);
   s2 += std::sin(2.0 * beam.angle);
  }
  const double angular = result.usable_returns >= 2
      ? std::clamp(1.0 - std::hypot(c2, s2) /
                             static_cast<double>(result.usable_returns),
                   0.0, 1.0)
      : 0.0;
  result.geometric_support = angular *
      static_cast<double>(result.usable_returns) /
      static_cast<double>(std::max<std::size_t>(1, config.minimum_returns));
 }
 if (result.usable_returns < config.minimum_returns) {
 finish("insufficient geometric returns");
 return;
 }
 if (!prepared->allocate()) {
 finish("map index allocation failed");
 return;
 }
 const std::size_t coarse_stride = std::max<std::size_t>(
 1, (result.usable_returns + config.coarse_beams - 1) /
 config.coarse_beams);
 std::size_t seen_hits = 0;
 for (const auto& beam : full) {
 if (!beam.hit) continue;
 if (seen_hits++ % coarse_stride != 0) continue;
 beams.push_back(beam);
 if (beams.size() >= config.coarse_beams) break;
 }
 n_headings = std::max(1, static_cast<int>(std::ceil(
 2.0 * core::kPi / config.coarse_heading_step_rad)));
 coarse_dx.resize(static_cast<std::size_t>(n_headings) * beams.size());
 coarse_dy.resize(static_cast<std::size_t>(n_headings) * beams.size());
 coarse_mount_x.resize(static_cast<std::size_t>(n_headings));
 coarse_mount_y.resize(static_cast<std::size_t>(n_headings));
 for (int ih = 0; ih < n_headings; ++ih) {
 const double heading = core::wrap_angle(
 -core::kPi + ih * config.coarse_heading_step_rad);
 const core::Quaternion body_q =
 core::quaternion_from_rpy(body_roll_rad, body_pitch_rad, heading);
 const core::Quaternion sensor_q =
 core::quaternion_multiply(body_q, mount.rotation);
 const core::Vec3 mount_off =
 core::rotate_vector(body_q, mount.translation_m);
 coarse_mount_x[static_cast<std::size_t>(ih)] = mount_off.x;
 coarse_mount_y[static_cast<std::size_t>(ih)] = mount_off.y;
 for (std::size_t beam_index = 0; beam_index < beams.size(); ++beam_index) {
 const ScanBeam& beam = beams[beam_index];
 const core::Vec3 direction = core::rotate_vector(
 sensor_q, {std::cos(beam.angle), std::sin(beam.angle), 0.0});
 const std::size_t index =
 static_cast<std::size_t>(ih) * beams.size() + beam_index;
 coarse_dx[index] = static_cast<float>(direction.x);
 coarse_dy[index] = static_cast<float>(direction.y);
 }
 }
 cell_stride = std::max(1, static_cast<int>(std::round(
 config.coarse_position_step_m / map.resolution_m)));
 coarse_max_gx = static_cast<int>(map.width);
 coarse_max_gy = static_cast<int>(map.height);
 if (bound_position) {
 if (!std::isfinite(bound_x_m) || !std::isfinite(bound_y_m) ||
 !(bound_radius_m > 0.0)) {
 finish("invalid bounded search region");
 return;
 }
 position_bounded = true;
 bound_center_x_m = bound_x_m;
 bound_center_y_m = bound_y_m;
 bound_radius_m_ = bound_radius_m;
 const auto lower_grid = [&](double world, double origin, int limit) {
 const int raw = static_cast<int>(std::floor(
 (world - origin) / map.resolution_m));
 const int clamped = std::clamp(raw, 0, limit - 1);
 return ((clamped + cell_stride - 1) / cell_stride) * cell_stride;
 };
 const auto upper_grid_exclusive = [&](double world, double origin,
 int limit) {
 const int raw = static_cast<int>(std::floor(
 (world - origin) / map.resolution_m));
 const int clamped = std::clamp(raw, 0, limit - 1);
 return std::min(limit,
 (clamped / cell_stride + 1) * cell_stride);
 };
 coarse_min_gx = lower_grid(bound_x_m - bound_radius_m,
 map.origin_x_m, static_cast<int>(map.width));
 coarse_max_gx = upper_grid_exclusive(bound_x_m + bound_radius_m,
 map.origin_x_m, static_cast<int>(map.width));
 coarse_min_gy = lower_grid(bound_y_m - bound_radius_m,
 map.origin_y_m, static_cast<int>(map.height));
 coarse_max_gy = upper_grid_exclusive(bound_y_m + bound_radius_m,
 map.origin_y_m, static_cast<int>(map.height));
 coarse_gx = coarse_min_gx;
 coarse_gy = coarse_min_gy;
 }
 if (bound_heading) {
 if (!std::isfinite(bound_heading_rad) ||
 !(bound_heading_radius_rad > 0.0) ||
 !std::isfinite(bound_heading_radius_rad)) {
 finish("invalid bounded heading region");
 return;
 }
 heading_bounded = true;
 bound_heading_rad_ = core::wrap_angle(bound_heading_rad);
 bound_heading_radius_rad_ = std::min(core::kPi, bound_heading_radius_rad);
 }
 coarse_lookup.resize(256);
 for (std::size_t i = 0; i < coarse_lookup.size(); ++i) {
 const double distance_m = 0.02 * static_cast<double>(i);
 const double ratio = distance_m / config.coarse_score_sigma_m;
 coarse_lookup[i] = static_cast<float>(std::exp(-ratio * ratio));
 }
 coarse_lookup.back() = -0.05F;
 // Candidate poses are base poses while scan endpoints originate at the
 // lidar. Include the horizontal mount offset so pruning remains exact for
 // non-central sensors: a return cannot touch occupied geometry outside
 // this bound.
 const double mount_radius_m =
 std::hypot(mount.translation_m.x, mount.translation_m.y);
 observable_radius_cells = std::max(
 1, static_cast<int>(std::ceil(
 config.max_range_m + mount_radius_m) / map.resolution_m));
 survivors.reserve(config.coarse_hypotheses);
 if (prepared->backward_complete) {
 phase = Phase::kCoarse;
 } else if (prepared->forward_complete) {
 phase = Phase::kPrepareBackward;
 }
 }

 void finish(const char* reason) {
 result.rejection_reason = reason;
 phase = Phase::kDone;
 }

 bool heading_allowed(double heading_rad) const {
 return !heading_bounded ||
 std::abs(core::wrap_angle(heading_rad - bound_heading_rad_)) <=
 bound_heading_radius_rad_;
 }

 void consider_coarse(const Coarse& candidate) {
 const auto same = [&](const Coarse& value) {
 return std::hypot(value.x - candidate.x, value.y - candidate.y) <
 config.coarse_hypothesis_distance_m &&
 std::abs(core::wrap_angle(value.heading - candidate.heading)) <
 config.coarse_hypothesis_heading_rad;
 };
 const auto existing = std::find_if(survivors.begin(), survivors.end(), same);
 if (existing != survivors.end()) {
 if (candidate.score > existing->score) *existing = candidate;
 return;
 }
 if (survivors.size() < config.coarse_hypotheses) {
 survivors.push_back(candidate);
 return;
 }
 const auto weakest = std::min_element(
 survivors.begin(), survivors.end(),
 [](const Coarse& a, const Coarse& b) { return a.score < b.score; });
 if (candidate.score > weakest->score) *weakest = candidate;
 }

 bool occupied_near(int x, int y) const {
 const int x0 = std::max(0, x - observable_radius_cells);
 const int y0 = std::max(0, y - observable_radius_cells);
 const int x1 = std::min(
 static_cast<int>(map.width) - 1, x + observable_radius_cells);
 const int y1 = std::min(
 static_cast<int>(map.height) - 1, y + observable_radius_cells);
 const std::size_t stride =
 static_cast<std::size_t>(map.width) + 1;
 const std::size_t left = static_cast<std::size_t>(x0);
 const std::size_t top = static_cast<std::size_t>(y0);
 const std::size_t right = static_cast<std::size_t>(x1) + 1;
 const std::size_t bottom = static_cast<std::size_t>(y1) + 1;
 const std::uint32_t count =
 prepared->occupied_prefix[bottom * stride + right] -
 prepared->occupied_prefix[top * stride + right] -
 prepared->occupied_prefix[bottom * stride + left] +
 prepared->occupied_prefix[top * stride + left];
 return count > 0;
 }

 double score_coarse(int heading_index, double base_x, double base_y) const {
 const std::size_t heading = static_cast<std::size_t>(heading_index);
 const double sensor_x = base_x + coarse_mount_x[heading];
 const double sensor_y = base_y + coarse_mount_y[heading];
 double score = 0.0;
 for (std::size_t beam_index = 0; beam_index < beams.size(); ++beam_index) {
 const std::size_t direction = heading * beams.size() + beam_index;
 const double wx = sensor_x +
 beams[beam_index].range * coarse_dx[direction];
 const double wy = sensor_y +
 beams[beam_index].range * coarse_dy[direction];
 const int gx = static_cast<int>(
 std::floor((wx - map.origin_x_m) / map.resolution_m));
 const int gy = static_cast<int>(
 std::floor((wy - map.origin_y_m) / map.resolution_m));
 if (gx < 0 || gy < 0 || gx >= static_cast<int>(map.width) ||
 gy >= static_cast<int>(map.height)) {
 score -= 0.25;
 continue;
 }
 const std::uint16_t distance = prepared->distance_cm[
 static_cast<std::size_t>(gy) * map.width + static_cast<std::size_t>(gx)];
 const std::size_t bucket = std::min<std::size_t>(
 distance / 2, coarse_lookup.size() - 1);
 score += coarse_lookup[bucket];
 }
 return score / static_cast<double>(beams.size());
 }

 void begin_refine() {
 result.coarse_evaluations = evaluations;
 if (survivors.empty()) {
 finish("no coarse hypothesis with map evidence");
 return;
 }
 std::sort(survivors.begin(), survivors.end(),
 [](const Coarse& a, const Coarse& b) {
 if (a.score != b.score) return a.score > b.score;
 if (a.x != b.x) return a.x < b.x;
 if (a.y != b.y) return a.y < b.y;
 return a.heading < b.heading;
 });
 ScanMatcherConfig refine_cfg;
 refine_cfg.search_translation_m = config.refine_translation_m;
 refine_cfg.search_heading_rad = config.refine_heading_rad;
 refine_cfg.translation_step_m = config.refine_translation_step_m;
 refine_cfg.heading_step_rad = config.refine_heading_step_rad;
 refine_cfg.minimum_overlap = config.minimum_overlap;
 refine_cfg.ambiguity_distance_m = config.refine_ambiguity_distance_m;
 refine_cfg.ambiguity_heading_rad = config.refine_ambiguity_heading_rad;
 refine_cfg.max_range_m = config.max_range_m;
 refine_cfg.minimum_returns = config.minimum_returns;
 refine_cfg.maximum_returns =
 std::max(config.refine_max_returns, config.minimum_returns);
 refiner = ScanMatcher(refine_cfg);
 refiner.set_lidar_mount(mount);
 refine_beams = refiner.prepare_beams(
 ranges, angle_min_rad, angle_increment_rad,
 refine_cfg.maximum_returns);
 refine_batch = std::make_unique<ScanMatcher::Batch>(
 refiner, refine_beams, body_roll_rad, body_pitch_rad);
 refine_steps = static_cast<int>(std::ceil(
 config.refine_translation_m / config.refine_translation_step_m));
 refine_heading_steps = static_cast<int>(std::ceil(
 config.refine_heading_rad / config.refine_heading_step_rad));
 reset_refine_seed();
 phase = Phase::kRefine;
 }

 void reset_refine_seed() {
 refine_iy = -refine_steps;
 refine_ix = -refine_steps;
 refine_ih = -refine_heading_steps;
 refine_hypotheses.clear();
 }

 void consider_refine(double x, double y, double heading,
 const PoseScore& pose) {
 ScanMatchResult candidate;
 candidate.x_m = x;
 candidate.y_m = y;
 candidate.yaw_rad = heading;
 candidate.score = pose.score;
 candidate.overlap = pose.overlap;
 candidate.geometric_support = pose.angular_observability *
     static_cast<double>(pose.usable_returns) /
     static_cast<double>(std::max<std::size_t>(1, config.minimum_returns));
 const auto same = [&](const ScanMatchResult& value) {
 return std::hypot(value.x_m - candidate.x_m,
 value.y_m - candidate.y_m) < config.refine_ambiguity_distance_m &&
 std::abs(core::wrap_angle(value.yaw_rad - candidate.yaw_rad)) <
 config.refine_ambiguity_heading_rad;
 };
 const auto existing = std::find_if(
 refine_hypotheses.begin(), refine_hypotheses.end(), same);
 if (existing != refine_hypotheses.end()) {
 if (candidate.score > existing->score) *existing = candidate;
 return;
 }
 constexpr std::size_t kMaxHypotheses = kMaximumScanMatchHypotheses;
 if (refine_hypotheses.size() < kMaxHypotheses) {
 refine_hypotheses.push_back(candidate);
 return;
 }
 const auto weakest = std::min_element(
 refine_hypotheses.begin(), refine_hypotheses.end(),
 [](const ScanMatchResult& a, const ScanMatchResult& b) {
 return a.score < b.score;
 });
 if (candidate.score > weakest->score) *weakest = candidate;
 }

 void complete_refine_seed() {
 ++result.refined_poses;
 if (!refine_hypotheses.empty()) {
 std::sort(refine_hypotheses.begin(), refine_hypotheses.end(),
 [](const ScanMatchResult& a, const ScanMatchResult& b) {
 if (a.score != b.score) return a.score > b.score;
 if (a.x_m != b.x_m) return a.x_m < b.x_m;
 if (a.y_m != b.y_m) return a.y_m < b.y_m;
 return a.yaw_rad < b.yaw_rad;
 });
 ScanMatchResult candidate = refine_hypotheses.front();
 const double second_score = refine_hypotheses.size() > 1
 ? refine_hypotheses[1].score
 : -std::numeric_limits<double>::infinity();
 if (std::isfinite(candidate.score) &&
 candidate.overlap >= config.minimum_overlap) {
 candidate.accepted = true;
 candidate.position_sigma_m = std::max(0.02, map.resolution_m * 0.75);
 candidate.heading_sigma_rad = std::max(
 0.01, config.refine_heading_step_rad * 1.5);
 candidate.score_margin = std::isfinite(second_score)
 ? candidate.score - second_score : 1.0;
 candidate.usable_returns = refine_batch->usable_returns();
 retain_refined(candidate);
 }
 }
 ++refine_index;
 reset_refine_seed();
 }

 void retain_refined(const ScanMatchResult& candidate) {
 const auto same = [&](const ScanMatchResult& value) {
 return std::hypot(value.x_m - candidate.x_m,
 value.y_m - candidate.y_m) < config.global_ambiguity_distance_m &&
 std::abs(core::wrap_angle(value.yaw_rad - candidate.yaw_rad)) <
 config.global_ambiguity_heading_rad;
 };
 const auto existing = std::find_if(refined.begin(), refined.end(), same);
 if (existing != refined.end()) {
 if (candidate.score > existing->score) *existing = candidate;
 } else {
 refined.push_back(candidate);
 }
 }

 void begin_final() {
 if (refined.empty()) {
 finish("no refined hypothesis accepted");
 return;
 }
 const std::vector<ScanBeam> full = coarse_matcher.prepare_beams(
 ranges, angle_min_rad, angle_increment_rad, ranges.size());
 std::size_t hits = 0;
 for (const auto& beam : full) {
 if (beam.hit) ++hits;
 }
 const std::size_t stride = std::max<std::size_t>(
 1, (hits + config.final_max_returns - 1) / config.final_max_returns);
 std::size_t seen = 0;
 for (const auto& beam : full) {
 if (!beam.hit) continue;
 if (seen++ % stride != 0) continue;
 final_beams.push_back(beam);
 if (final_beams.size() >= config.final_max_returns) break;
 }
 final_steps = static_cast<int>(std::ceil(
 config.final_translation_m / config.final_translation_step_m));
 final_heading_steps = static_cast<int>(std::ceil(
 config.final_heading_rad / config.final_heading_step_rad));
 final_iy = -final_steps;
 final_ix = -final_steps;
 final_ih = -final_heading_steps;
 reset_final_best();
 phase = Phase::kFinal;
 }

 void reset_final_best() {
 final_best = ScanMatchResult{};
 final_best.accepted = false;
 final_best.score = -std::numeric_limits<double>::infinity();
 }

 void retain_checked() {
 if (!final_best.accepted) return;
 const auto same = [&](const ScanMatchResult& value) {
 return std::hypot(value.x_m - final_best.x_m,
 value.y_m - final_best.y_m) < config.global_ambiguity_distance_m &&
 std::abs(core::wrap_angle(value.yaw_rad - final_best.yaw_rad)) <
 config.global_ambiguity_heading_rad;
 };
 const auto existing = std::find_if(checked.begin(), checked.end(), same);
 if (existing != checked.end()) {
 if (final_best.score > existing->score) *existing = final_best;
 } else {
 checked.push_back(final_best);
 }
 }

 void complete_final() {
 if (checked.empty()) {
 finish("no raycast-validated hypothesis accepted");
 return;
 }
 std::sort(checked.begin(), checked.end(),
 [](const ScanMatchResult& a, const ScanMatchResult& b) {
 if (a.score != b.score) return a.score > b.score;
 if (a.x_m != b.x_m) return a.x_m < b.x_m;
 if (a.y_m != b.y_m) return a.y_m < b.y_m;
 return a.yaw_rad < b.yaw_rad;
 });
 const ScanMatchResult& best = checked.front();
 result.x_m = best.x_m;
 result.y_m = best.y_m;
 result.yaw_rad = best.yaw_rad;
 result.position_sigma_m = best.position_sigma_m;
 result.heading_sigma_rad = best.heading_sigma_rad;
 result.score = best.score;
 result.overlap = best.overlap;
 result.geometric_support = best.geometric_support;
 if (!std::isfinite(best.score) || best.overlap < config.minimum_overlap) {
 finish("global map overlap below acceptance gate");
 return;
 }
 if (checked.size() > 1) {
 const double second_score = checked[1].score;
 const double second_overlap = checked[1].overlap;
 if (best.score - second_score < config.minimum_score_margin) {
 result.score_margin = best.score - second_score;
 fill_runner_ups(result, checked);
 finish("global match is geometrically ambiguous");
 return;
 }
 if (best.overlap - second_overlap <
 config.equivalent_support_overlap_tolerance &&
 best.score - second_score < config.equivalent_support_score_margin) {
 result.score_margin = best.score - second_score;
 fill_runner_ups(result, checked);
 finish("global match has equivalent support");
 return;
 }
 }
 result.accepted = true;
 result.rejection_reason = "accepted";
 result.score_margin =
 checked.size() > 1 ? best.score - checked[1].score : 1.0;
 fill_runner_ups(result, checked);
 phase = Phase::kDone;
 }

 GlobalSearchConfig config;
 core::RigidTransform mount;
 std::shared_ptr<PreparedMap> prepared;
 std::vector<float> ranges;
 const iface::MapSnapshot& map;
 double angle_min_rad = 0.0;
 double angle_increment_rad = 0.0;
 core::MonotonicNs stamp_ns = 0;
 double body_roll_rad = 0.0, body_pitch_rad = 0.0, body_z_m = 0.0;
 GlobalSearchResult result;
 Phase phase = Phase::kPrepareForward;
 std::vector<float> coarse_lookup;
 int observable_radius_cells = 1;
 ScanMatcher coarse_matcher;
 ScanMatcher refiner;
 std::vector<ScanBeam> beams;
 std::vector<float> coarse_dx, coarse_dy;
 std::vector<double> coarse_mount_x, coarse_mount_y;
 int n_headings = 1, cell_stride = 1;
 int coarse_gx = 0, coarse_gy = 0, coarse_ih = 0;
 int coarse_min_gx = 0, coarse_min_gy = 0;
 int coarse_max_gx = 0, coarse_max_gy = 0;
 bool position_bounded = false;
 bool heading_bounded = false;
 double bound_center_x_m = 0.0, bound_center_y_m = 0.0;
 double bound_radius_m_ = 0.0;
 double bound_heading_rad_ = 0.0;
 double bound_heading_radius_rad_ = 0.0;
 std::size_t evaluations = 0;
 std::vector<Coarse> survivors;
 std::size_t refine_index = 0;
 std::vector<ScanBeam> refine_beams;
 std::unique_ptr<ScanMatcher::Batch> refine_batch;
 int refine_steps = 0, refine_heading_steps = 0;
 int refine_iy = 0, refine_ix = 0, refine_ih = 0;
 std::vector<ScanMatchResult> refine_hypotheses;
 std::vector<ScanMatchResult> refined;
 std::vector<ScanBeam> final_beams;
 int final_steps = 0, final_heading_steps = 0;
 std::size_t final_index = 0;
 int final_iy = 0, final_ix = 0, final_ih = 0;
 ScanMatchResult final_best;
 std::vector<ScanMatchResult> checked;
};

GlobalSearcher::GlobalSearcher(GlobalSearchConfig config) : config_(config) {}

GlobalSearcher::~GlobalSearcher() = default;
GlobalSearcher::GlobalSearcher(GlobalSearcher&&) noexcept = default;
GlobalSearcher& GlobalSearcher::operator=(GlobalSearcher&&) noexcept = default;

void GlobalSearcher::begin(const std::vector<float>& ranges,
 double angle_min_rad, double angle_increment_rad,
 const iface::MapSnapshot& map, core::MonotonicNs stamp_ns,
 double body_roll_rad, double body_pitch_rad, double body_z_m) {
 if (prepared_map_ == nullptr || !prepared_map_->matches(map)) {
 try {
 prepared_map_ = std::make_shared<PreparedMap>(map);
 } catch (const std::bad_alloc&) {
 prepared_map_.reset();
 }
 }
 work_ = std::make_unique<WorkState>(
 config_, base_from_lidar_, prepared_map_, ranges, angle_min_rad,
 angle_increment_rad, map,
 stamp_ns, body_roll_rad, body_pitch_rad, body_z_m,
 false, 0.0, 0.0, 0.0, false, 0.0, 0.0);
}

void GlobalSearcher::begin_near(
 const std::vector<float>& ranges, double angle_min_rad,
 double angle_increment_rad, const iface::MapSnapshot& map,
 core::MonotonicNs stamp_ns, double center_x_m, double center_y_m,
 double radius_m, double center_heading_rad, double heading_radius_rad,
 double body_roll_rad, double body_pitch_rad,
 double body_z_m) {
 if (prepared_map_ == nullptr || !prepared_map_->matches(map)) {
 try {
 prepared_map_ = std::make_shared<PreparedMap>(map);
 } catch (const std::bad_alloc&) {
 prepared_map_.reset();
 }
 }
 work_ = std::make_unique<WorkState>(
 config_, base_from_lidar_, prepared_map_, ranges, angle_min_rad,
 angle_increment_rad, map,
 stamp_ns, body_roll_rad, body_pitch_rad, body_z_m,
 true, center_x_m, center_y_m, radius_m,
 true, center_heading_rad, heading_radius_rad);
}

bool GlobalSearcher::active() const { return work_ != nullptr; }

bool GlobalSearcher::active_map_matches(
 const iface::MapSnapshot& map) const {
 if (work_ == nullptr) return false;
 // WorkState owns no raster copy: even an equivalent replacement object
 // cannot extend the lifetime of the reference captured by begin().
 if (&work_->map != &map || prepared_map_ == nullptr) return false;
 if (!map.map_id.empty()) return prepared_map_->matches(map);
 // Anonymous fixtures have no revision identity. They may be resumed only
 // while the same immutable object and geometry remain alive.
 return prepared_map_->width == map.width &&
 prepared_map_->height == map.height &&
 prepared_map_->resolution_m == map.resolution_m &&
 prepared_map_->origin_x_m == map.origin_x_m &&
 prepared_map_->origin_y_m == map.origin_y_m;
}

void GlobalSearcher::cancel() { work_.reset(); }

std::optional<GlobalSearchResult> GlobalSearcher::advance(
 std::size_t work_budget) {
 if (work_ == nullptr) return std::nullopt;
 WorkState& w = *work_;
 std::size_t budget = std::max<std::size_t>(1, work_budget);
 const auto spend = [&](std::size_t cost) {
 budget = budget > cost ? budget - cost : 0;
 };
 while (budget > 0 && w.phase != WorkState::Phase::kDone) {
 if (w.phase == WorkState::Phase::kPrepareForward) {
 const std::size_t cell_count =
 static_cast<std::size_t>(w.map.width) * w.map.height;
 const std::size_t end = w.prepared->prepare_cursor + std::min(
 w.config.preparation_cells_per_cycle,
 cell_count - w.prepared->prepare_cursor);
 const std::size_t prefix_width =
 static_cast<std::size_t>(w.map.width) + 1;
 for (; w.prepared->prepare_cursor < end;
 ++w.prepared->prepare_cursor, ++w.result.map_preparation_cells) {
 const std::size_t x = w.prepared->prepare_cursor % w.map.width;
 const std::size_t y = w.prepared->prepare_cursor / w.map.width;
 const std::size_t prefix = (y + 1) * prefix_width + (x + 1);
 const std::uint32_t occupied =
 w.map.occupancy[w.prepared->prepare_cursor] >= 50 ? 1U : 0U;
 w.prepared->occupied_prefix[prefix] = occupied +
 w.prepared->occupied_prefix[prefix - 1] +
 w.prepared->occupied_prefix[prefix - prefix_width] -
 w.prepared->occupied_prefix[prefix - prefix_width - 1];
 const auto add_distance = [](std::uint16_t value, std::uint16_t step) {
 const std::uint16_t infinity =
 std::numeric_limits<std::uint16_t>::max();
 return value > static_cast<std::uint16_t>(infinity - step)
 ? infinity
 : static_cast<std::uint16_t>(value + step);
 };
 std::uint16_t distance = occupied != 0
 ? 0
 : std::numeric_limits<std::uint16_t>::max();
 if (occupied == 0) {
 if (x > 0) {
 distance = std::min(distance, add_distance(
 w.prepared->distance_cm[w.prepared->prepare_cursor - 1],
 w.prepared->distance_orthogonal_cm));
 }
 if (y > 0) {
 distance = std::min(distance, add_distance(
 w.prepared->distance_cm[w.prepared->prepare_cursor - w.map.width],
 w.prepared->distance_orthogonal_cm));
 if (x > 0) {
 distance = std::min(distance, add_distance(
 w.prepared->distance_cm[
 w.prepared->prepare_cursor - w.map.width - 1],
 w.prepared->distance_diagonal_cm));
 }
 if (x + 1 < w.map.width) {
 distance = std::min(distance, add_distance(
 w.prepared->distance_cm[
 w.prepared->prepare_cursor - w.map.width + 1],
 w.prepared->distance_diagonal_cm));
 }
 }
 }
 w.prepared->distance_cm[w.prepared->prepare_cursor] = distance;
 }
 if (w.prepared->prepare_cursor < cell_count) return std::nullopt;
 w.prepared->forward_complete = true;
 w.phase = WorkState::Phase::kPrepareBackward;
 continue;
 }
 if (w.phase == WorkState::Phase::kPrepareBackward) {
 const std::size_t cell_count =
 static_cast<std::size_t>(w.map.width) * w.map.height;
 if (w.prepared->prepare_cursor > cell_count) {
 w.prepared->prepare_cursor = cell_count;
 }
 const std::size_t begin = w.prepared->prepare_cursor - std::min(
 w.config.preparation_cells_per_cycle, w.prepared->prepare_cursor);
 const auto add_distance = [](std::uint16_t value, std::uint16_t step) {
 const std::uint16_t infinity =
 std::numeric_limits<std::uint16_t>::max();
 return value > static_cast<std::uint16_t>(infinity - step)
 ? infinity
 : static_cast<std::uint16_t>(value + step);
 };
 while (w.prepared->prepare_cursor > begin) {
 --w.prepared->prepare_cursor;
 ++w.result.map_preparation_cells;
 const std::size_t x = w.prepared->prepare_cursor % w.map.width;
 const std::size_t y = w.prepared->prepare_cursor / w.map.width;
 std::uint16_t& distance =
 w.prepared->distance_cm[w.prepared->prepare_cursor];
 if (x + 1 < w.map.width) {
 distance = std::min(distance, add_distance(
 w.prepared->distance_cm[w.prepared->prepare_cursor + 1],
 w.prepared->distance_orthogonal_cm));
 }
 if (y + 1 < w.map.height) {
 distance = std::min(distance, add_distance(
 w.prepared->distance_cm[w.prepared->prepare_cursor + w.map.width],
 w.prepared->distance_orthogonal_cm));
 if (x > 0) {
 distance = std::min(distance, add_distance(
 w.prepared->distance_cm[
 w.prepared->prepare_cursor + w.map.width - 1],
 w.prepared->distance_diagonal_cm));
 }
 if (x + 1 < w.map.width) {
 distance = std::min(distance, add_distance(
 w.prepared->distance_cm[
 w.prepared->prepare_cursor + w.map.width + 1],
 w.prepared->distance_diagonal_cm));
 }
 }
 }
 if (w.prepared->prepare_cursor > 0) return std::nullopt;
 w.prepared->backward_complete = true;
 w.phase = WorkState::Phase::kCoarse;
 continue;
 }
 if (w.phase == WorkState::Phase::kCoarse) {
 if (w.coarse_gy >= w.coarse_max_gy) {
 w.begin_refine();
 continue;
 }
 if (w.coarse_gx >= w.coarse_max_gx) {
 w.coarse_gx = w.coarse_min_gx;
 w.coarse_gy += w.cell_stride;
 w.coarse_ih = 0;
 continue;
 }
 const double cx = w.map.origin_x_m +
 (w.coarse_gx + 0.5) * w.map.resolution_m;
 const double cy = w.map.origin_y_m +
 (w.coarse_gy + 0.5) * w.map.resolution_m;
 if (w.position_bounded &&
 std::hypot(cx - w.bound_center_x_m, cy - w.bound_center_y_m) >
 w.bound_radius_m_) {
 w.coarse_gx += w.cell_stride;
 w.coarse_ih = 0;
 spend(1);
 continue;
 }
 const std::size_t cell_index = static_cast<std::size_t>(w.coarse_gy) *
 w.map.width + static_cast<std::size_t>(w.coarse_gx);
 if (w.map.occupancy[cell_index] >= 50) {
 w.coarse_gx += w.cell_stride;
 w.coarse_ih = 0;
 spend(1);
 continue;
 }
 if (!w.occupied_near(w.coarse_gx, w.coarse_gy)) {
 ++w.result.coarse_positions_skipped_unobservable;
 w.coarse_gx += w.cell_stride;
 w.coarse_ih = 0;
 spend(1);
 continue;
 }
 if (w.coarse_ih == 0) ++w.result.coarse_positions_considered;
 const double heading = core::wrap_angle(
 -core::kPi + w.coarse_ih * w.config.coarse_heading_step_rad);
 if (!w.heading_allowed(heading)) {
 ++w.coarse_ih;
 if (w.coarse_ih >= w.n_headings) {
 w.coarse_ih = 0;
 w.coarse_gx += w.cell_stride;
 }
 spend(1);
 continue;
 }
 ++w.evaluations;
 const double score = w.score_coarse(w.coarse_ih, cx, cy);
 w.consider_coarse({cx, cy, heading, score, 0.0});
 ++w.coarse_ih;
 if (w.coarse_ih >= w.n_headings) {
 w.coarse_ih = 0;
 w.coarse_gx += w.cell_stride;
 }
 spend(1);
 continue;
 }
 if (w.phase == WorkState::Phase::kRefine) {
 if (w.refine_index >= w.survivors.size()) {
 w.begin_final();
 continue;
 }
 const WorkState::Coarse& seed = w.survivors[w.refine_index];
 const double cx = seed.x +
 w.refine_ix * w.config.refine_translation_step_m;
 const double cy = seed.y +
 w.refine_iy * w.config.refine_translation_step_m;
 const double heading = core::wrap_angle(
 seed.heading + w.refine_ih * w.config.refine_heading_step_rad);
 if (w.heading_allowed(heading)) {
 const PoseScore pose = w.refine_batch->score(
 static_cast<std::size_t>(w.refine_ih + w.refine_heading_steps),
 cx, cy, heading, w.map, w.body_z_m);
 if (pose.has_evidence) w.consider_refine(cx, cy, heading, pose);
 }
 ++w.refine_ih;
 if (w.refine_ih > w.refine_heading_steps) {
 w.refine_ih = -w.refine_heading_steps;
 ++w.refine_ix;
 if (w.refine_ix > w.refine_steps) {
 w.refine_ix = -w.refine_steps;
 ++w.refine_iy;
 if (w.refine_iy > w.refine_steps) w.complete_refine_seed();
 }
 }
 spend(1);
 continue;
 }
 if (w.final_index >= w.refined.size()) {
 w.complete_final();
 continue;
 }
 const ScanMatchResult& seed = w.refined[w.final_index];
 const double cx = seed.x_m +
 w.final_ix * w.config.final_translation_step_m;
 const double cy = seed.y_m +
 w.final_iy * w.config.final_translation_step_m;
 const double heading = core::wrap_angle(
 seed.yaw_rad + w.final_ih * w.config.final_heading_step_rad);
 if (w.heading_allowed(heading)) {
 const RaycastEvidence ev = raycast_evidence(
 w.map, w.final_beams, cx, cy, heading, w.body_roll_rad,
 w.body_pitch_rad, w.mount, w.config.max_range_m);
 if (ev.has_evidence && ev.score > w.final_best.score) {
 w.final_best.accepted = true;
 w.final_best.x_m = cx;
 w.final_best.y_m = cy;
 w.final_best.yaw_rad = heading;
 w.final_best.score = ev.score;
 w.final_best.overlap = ev.overlap;
 w.final_best.position_sigma_m = seed.position_sigma_m;
 w.final_best.heading_sigma_rad = seed.heading_sigma_rad;
 w.final_best.geometric_support = seed.geometric_support;
 }
 }
 ++w.final_ih;
 if (w.final_ih > w.final_heading_steps) {
 w.final_ih = -w.final_heading_steps;
 ++w.final_ix;
 if (w.final_ix > w.final_steps) {
 w.final_ix = -w.final_steps;
 ++w.final_iy;
 if (w.final_iy > w.final_steps) {
 w.retain_checked();
 ++w.final_index;
 w.final_iy = -w.final_steps;
 w.final_ix = -w.final_steps;
 w.final_ih = -w.final_heading_steps;
 w.reset_final_best();
 }
 }
 }
 // A raycast candidate costs roughly an order of magnitude more than a
 // coarse endpoint score. Weighted accounting keeps each 20 ms ROBOT tick
 // bounded without changing which candidates are evaluated.
 spend(32);
 }
 if (w.phase != WorkState::Phase::kDone) return std::nullopt;
 GlobalSearchResult result = std::move(w.result);
 work_.reset();
 return result;
}

GlobalSearchResult GlobalSearcher::search(
 const std::vector<float>& ranges, double angle_min_rad,
 double angle_increment_rad, const iface::MapSnapshot& map,
 core::MonotonicNs stamp_ns, double body_roll_rad, double body_pitch_rad,
 double body_z_m) const {
 GlobalSearcher complete(config_);
 complete.set_lidar_mount(base_from_lidar_);
 complete.prepared_map_ = prepared_map_;
 complete.begin(ranges, angle_min_rad, angle_increment_rad, map, stamp_ns,
 body_roll_rad, body_pitch_rad, body_z_m);
 for (;;) {
 const auto result = complete.advance(
 std::numeric_limits<std::size_t>::max() / 4);
 if (result.has_value()) {
 prepared_map_ = complete.prepared_map_;
 return *result;
 }
 }
}

}  // namespace lost_signal::robot::autonomy
