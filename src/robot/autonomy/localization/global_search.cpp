#include "robot/autonomy/localization/global_search.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
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

struct GlobalSearcher::WorkState {
 enum class Phase { kCoarse, kRefine, kFinal, kDone };
 struct Coarse {
 double x = 0.0, y = 0.0, heading = 0.0, score = 0.0, overlap = 0.0;
 };

 WorkState(const GlobalSearchConfig& cfg,
 const core::RigidTransform& mount_value,
 const std::vector<float>& range_values, double angle_min,
 double angle_increment, const iface::MapSnapshot& map_value,
 core::MonotonicNs stamp, double roll, double pitch, double z)
 : config(cfg), mount(mount_value), ranges(range_values), map(map_value),
 angle_min_rad(angle_min), angle_increment_rad(angle_increment),
 stamp_ns(stamp), body_roll_rad(roll), body_pitch_rad(pitch), body_z_m(z) {
 if (ranges.empty() || !(angle_increment_rad > 0.0)) {
 finish("invalid scan");
 return;
 }
 if (!map.has_raster()) {
 finish("map snapshot has no raster");
 return;
 }
 if (!(config.coarse_position_step_m > 0.0) ||
 !(config.coarse_heading_step_rad > 0.0) || config.coarse_beams == 0 ||
 config.coarse_hypotheses == 0 ||
 !(config.refine_translation_m >= 0.0) ||
 !(config.refine_translation_step_m > 0.0) ||
 !(config.refine_heading_rad >= 0.0) ||
 !(config.refine_heading_step_rad > 0.0) ||
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
 beams = coarse_matcher.prepare_beams(
 ranges, angle_min_rad, angle_increment_rad, config.coarse_beams);
 const std::vector<ScanBeam> full = coarse_matcher.prepare_beams(
 ranges, angle_min_rad, angle_increment_rad, ranges.size());
 for (const auto& beam : full) {
 if (beam.hit) ++result.usable_returns;
 }
 if (result.usable_returns < config.minimum_returns) {
 finish("insufficient geometric returns");
 return;
 }
 batch = std::make_unique<ScanMatcher::Batch>(
 coarse_matcher, beams, body_roll_rad, body_pitch_rad);
 n_headings = std::max(1, static_cast<int>(std::ceil(
 2.0 * core::kPi / config.coarse_heading_step_rad)));
 cell_stride = std::max(1, static_cast<int>(std::round(
 config.coarse_position_step_m / map.resolution_m)));
 survivors.reserve(config.coarse_hypotheses);
 }

 void finish(const char* reason) {
 result.rejection_reason = reason;
 phase = Phase::kDone;
 }

 void consider_coarse(const Coarse& candidate) {
 const auto same = [&](const Coarse& value) {
 return std::hypot(value.x - candidate.x, value.y - candidate.y) <
 0.5 * config.coarse_position_step_m &&
 std::abs(core::wrap_angle(value.heading - candidate.heading)) <
 0.5 * config.coarse_heading_step_rad;
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
 refine_cfg.minimum_score_margin = config.refine_score_margin;
 refine_cfg.max_range_m = config.max_range_m;
 refine_cfg.minimum_returns = config.minimum_returns;
 refine_cfg.maximum_returns =
 std::max(config.refine_max_returns, config.minimum_returns);
 refiner = ScanMatcher(refine_cfg);
 refiner.set_lidar_mount(mount);
 phase = Phase::kRefine;
 }

 void retain_refined(const ScanMatchResult& candidate) {
 const auto same = [&](const ScanMatchResult& value) {
 return std::hypot(value.x_m - candidate.x_m,
 value.y_m - candidate.y_m) < config.refine_ambiguity_distance_m &&
 std::abs(core::wrap_angle(value.yaw_rad - candidate.yaw_rad)) <
 config.refine_ambiguity_heading_rad;
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
 value.y_m - final_best.y_m) < config.refine_ambiguity_distance_m &&
 std::abs(core::wrap_angle(value.yaw_rad - final_best.yaw_rad)) <
 config.refine_ambiguity_heading_rad;
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
 config.equivalent_support_overlap_tolerance) {
 result.score_margin = best.score - second_score;
 fill_runner_ups(result, checked);
 finish("global match has equivalent support");
 return;
 }
 }
 result.accepted = true;
 result.score_margin =
 checked.size() > 1 ? best.score - checked[1].score : 1.0;
 fill_runner_ups(result, checked);
 phase = Phase::kDone;
 }

 GlobalSearchConfig config;
 core::RigidTransform mount;
 std::vector<float> ranges;
 iface::MapSnapshot map;
 double angle_min_rad = 0.0;
 double angle_increment_rad = 0.0;
 core::MonotonicNs stamp_ns = 0;
 double body_roll_rad = 0.0, body_pitch_rad = 0.0, body_z_m = 0.0;
 GlobalSearchResult result;
 Phase phase = Phase::kCoarse;
 ScanMatcher coarse_matcher;
 ScanMatcher refiner;
 std::vector<ScanBeam> beams;
 std::unique_ptr<ScanMatcher::Batch> batch;
 int n_headings = 1, cell_stride = 1;
 int coarse_gx = 0, coarse_gy = 0, coarse_ih = 0;
 std::size_t evaluations = 0;
 std::vector<Coarse> survivors;
 std::size_t refine_index = 0;
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
 work_ = std::make_unique<WorkState>(
 config_, base_from_lidar_, ranges, angle_min_rad, angle_increment_rad, map,
 stamp_ns, body_roll_rad, body_pitch_rad, body_z_m);
}

bool GlobalSearcher::active() const { return work_ != nullptr; }

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
 if (w.phase == WorkState::Phase::kCoarse) {
 if (w.coarse_gy >= static_cast<int>(w.map.height)) {
 w.begin_refine();
 continue;
 }
 if (w.coarse_gx >= static_cast<int>(w.map.width)) {
 w.coarse_gx = 0;
 w.coarse_gy += w.cell_stride;
 w.coarse_ih = 0;
 continue;
 }
 const std::size_t cell_index = static_cast<std::size_t>(w.coarse_gy) *
 w.map.width + static_cast<std::size_t>(w.coarse_gx);
 if (w.map.occupancy[cell_index] >= 50) {
 w.coarse_gx += w.cell_stride;
 w.coarse_ih = 0;
 continue;
 }
 if (++w.evaluations > w.config.maximum_evaluations) {
 w.result.coarse_evaluations = w.evaluations;
 w.finish("evaluation budget exceeded");
 continue;
 }
 const double cx = w.map.origin_x_m +
 (w.coarse_gx + 0.5) * w.map.resolution_m;
 const double cy = w.map.origin_y_m +
 (w.coarse_gy + 0.5) * w.map.resolution_m;
 const double heading = core::wrap_angle(
 -core::kPi + w.coarse_ih * w.config.coarse_heading_step_rad);
 const PoseScore pose = w.batch->score(
 static_cast<std::size_t>(w.coarse_ih), cx, cy, heading, w.map,
 w.body_z_m);
 if (pose.has_evidence) {
 w.consider_coarse({cx, cy, heading, pose.score, pose.overlap});
 }
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
 const WorkState::Coarse& seed = w.survivors[w.refine_index++];
 ++w.result.refined_poses;
 const ScanMatchResult refined = w.refiner.localize(
 w.ranges, w.angle_min_rad, w.angle_increment_rad, seed.x, seed.y,
 seed.heading, w.map, w.stamp_ns, w.body_roll_rad, w.body_pitch_rad,
 w.body_z_m);
 if (refined.accepted) w.retain_refined(refined);
 spend(512);
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
 spend(8);
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
 complete.begin(ranges, angle_min_rad, angle_increment_rad, map, stamp_ns,
 body_roll_rad, body_pitch_rad, body_z_m);
 for (;;) {
 const auto result = complete.advance(
 std::numeric_limits<std::size_t>::max() / 4);
 if (result.has_value()) return *result;
 }
}

}  // namespace lost_signal::robot::autonomy
