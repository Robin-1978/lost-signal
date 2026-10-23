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

GlobalSearcher::GlobalSearcher(GlobalSearchConfig config) : config_(config) {}

GlobalSearchResult GlobalSearcher::search(
    const std::vector<float>& ranges, double angle_min_rad,
    double angle_increment_rad, const iface::MapSnapshot& map,
    core::MonotonicNs stamp_ns, double body_roll_rad, double body_pitch_rad,
    double body_z_m) const {
  GlobalSearchResult result;
  (void)stamp_ns;
  if (ranges.empty() || !(angle_increment_rad > 0.0)) {
    result.rejection_reason = "invalid scan";
    return result;
  }
  if (!map.has_raster()) {
    result.rejection_reason = "map snapshot has no raster";
    return result;
  }
  if (!(config_.coarse_position_step_m > 0.0) ||
      !(config_.coarse_heading_step_rad > 0.0) ||
      config_.coarse_beams == 0 || config_.coarse_hypotheses == 0 ||
      !(config_.refine_translation_m >= 0.0) ||
      !(config_.refine_translation_step_m > 0.0) ||
      !(config_.refine_heading_rad >= 0.0) ||
      !(config_.refine_heading_step_rad > 0.0) ||
      !(config_.final_translation_m >= 0.0) ||
      !(config_.final_translation_step_m > 0.0) ||
      !(config_.final_heading_rad >= 0.0) ||
      !(config_.final_heading_step_rad > 0.0) ||
      config_.final_max_returns == 0) {
    result.rejection_reason = "invalid global search configuration";
    return result;
  }

  ScanMatcherConfig coarse_cfg;
  coarse_cfg.max_range_m = config_.max_range_m;
  coarse_cfg.minimum_returns = config_.minimum_returns;
  ScanMatcher coarse_matcher(coarse_cfg);
  coarse_matcher.set_lidar_mount(base_from_lidar_);
  // Coarse ranking keeps the calibrated mixed subset (hits plus max-range
  // fill, strided); the sparse-scan guard below counts full-scan hits, so
  // a 24-beam slice low on returns cannot veto a scan that has evidence.
  const std::vector<ScanBeam> beams = coarse_matcher.prepare_beams(
      ranges, angle_min_rad, angle_increment_rad, config_.coarse_beams);
  {
    const std::vector<ScanBeam> full = coarse_matcher.prepare_beams(
        ranges, angle_min_rad, angle_increment_rad, ranges.size());
    std::size_t hits = 0;
    for (const auto& beam : full) {
      if (beam.hit) ++hits;
    }
    result.usable_returns = hits;
    if (hits < config_.minimum_returns) {
      result.rejection_reason = "insufficient geometric returns";
      return result;
    }
  }

  // Coarse pass over free raster cells. Row-major order with ascending
  // headings keeps the search deterministic; ties break by (x, y, heading).
  struct Coarse {
    double x = 0.0, y = 0.0, heading = 0.0, score = 0.0, overlap = 0.0;
  };
  std::vector<Coarse> survivors;
  survivors.reserve(config_.coarse_hypotheses);
  const auto consider = [&](const Coarse& c) {
    const auto same = [&](const Coarse& v) {
      return std::hypot(v.x - c.x, v.y - c.y) < 0.5 * config_.coarse_position_step_m &&
             std::abs(core::wrap_angle(v.heading - c.heading)) <
                 0.5 * config_.coarse_heading_step_rad;
    };
    const auto existing =
        std::find_if(survivors.begin(), survivors.end(), same);
    if (existing != survivors.end()) {
      if (c.score > existing->score) *existing = c;
      return;
    }
    if (survivors.size() < config_.coarse_hypotheses) {
      survivors.push_back(c);
      return;
    }
    const auto weakest =
        std::min_element(survivors.begin(), survivors.end(),
                         [](const Coarse& a, const Coarse& b) {
                           return a.score < b.score;
                         });
    if (c.score > weakest->score) *weakest = c;
  };

  const int n_headings = std::max(
      1, static_cast<int>(std::ceil(2.0 * core::kPi / config_.coarse_heading_step_rad)));
  std::size_t evaluations = 0;
  const double step = config_.coarse_position_step_m;
  const int gx0 = 0, gy0 = 0;
  const int gx1 = static_cast<int>(map.width), gy1 = static_cast<int>(map.height);
  // Batched scoring (slot = ih — stable per heading, traversal order
  // untouched so survivor ranking cannot change).
  ScanMatcher::Batch batch(coarse_matcher, beams, body_roll_rad,
                           body_pitch_rad);
  // Stride in cells so visited centers sit ~step apart regardless of raster.
  const int cell_stride =
      std::max(1, static_cast<int>(std::round(step / map.resolution_m)));
  for (int gy = gy0; gy < gy1; gy += cell_stride) {
    for (int gx = gx0; gx < gx1; gx += cell_stride) {
      const int8_t cell =
          map.occupancy[static_cast<std::size_t>(gy) * map.width + gx];
      if (cell < 0 || cell >= 50) continue;  // hypotheses live in free space
      // Free-only seeding is load-bearing, not incidental. Unknown seeding
      // was tried: on sparse unknown surveys a lone wrong aisle refines
      // alone and the overlap gate (even raycast-validated) cannot tell
      // "explains one landmark" from the true pose — loop_closure proved
      // it with teleports. Unknown recovery needs the recall half first
      // (denser coarse grid / more survivors so twins compete and the
      // ambiguity gates engage), plus the SLAM-tracking path. Until then
      // global search stays a kidnap-recovery path for free maps only.
      const double cx = map.origin_x_m + (gx + 0.5) * map.resolution_m;
      const double cy = map.origin_y_m + (gy + 0.5) * map.resolution_m;
      for (int ih = 0; ih < n_headings; ++ih) {
        if (++evaluations > config_.maximum_evaluations) {
          result.coarse_evaluations = evaluations;
          result.rejection_reason = "evaluation budget exceeded";
          return result;
        }
        const double heading =
            core::wrap_angle(-core::kPi + ih * config_.coarse_heading_step_rad);
        const PoseScore pose = batch.score(static_cast<std::size_t>(ih), cx,
                                           cy, heading, map, body_z_m);
        if (!pose.has_evidence) continue;
        consider({cx, cy, heading, pose.score, pose.overlap});
      }
    }
  }
  result.coarse_evaluations = evaluations;
  if (survivors.empty()) {
    result.rejection_reason = "no coarse hypothesis with map evidence";
    return result;
  }

  // Refine each survivor with the bounded local matcher, best-first.
  // NOTE (measured): no pre-refine dedupe here by design. Coarse survivors
  // live on a 1.0 m x 0.35 rad grid, so any two are farther apart than one
  // ambiguity cell — and the twin check below NEEDS multiple refined
  // runner-ups to compare margins against. Merging early would only weaken
  // ambiguity protection while saving nothing (verified: 16 in, 16 out on
  // wall, T-junction and 50 m maps alike).
  std::sort(survivors.begin(), survivors.end(),
            [](const Coarse& a, const Coarse& b) {
              if (a.score != b.score) return a.score > b.score;
              if (a.x != b.x) return a.x < b.x;
              if (a.y != b.y) return a.y < b.y;
              return a.heading < b.heading;
            });
  ScanMatcherConfig refine_cfg;
  refine_cfg.search_translation_m = config_.refine_translation_m;
  refine_cfg.search_heading_rad = config_.refine_heading_rad;
  refine_cfg.translation_step_m = config_.refine_translation_step_m;
  refine_cfg.heading_step_rad = config_.refine_heading_step_rad;
  refine_cfg.minimum_overlap = config_.minimum_overlap;
  refine_cfg.minimum_score_margin = config_.refine_score_margin;
  refine_cfg.max_range_m = config_.max_range_m;
  refine_cfg.minimum_returns = config_.minimum_returns;
  refine_cfg.maximum_returns =
      std::max(config_.refine_max_returns, config_.minimum_returns);
  ScanMatcher refiner(refine_cfg);
  refiner.set_lidar_mount(base_from_lidar_);

  std::vector<ScanMatchResult> refined;
  for (const auto& s : survivors) {
    ++result.refined_poses;
    ScanMatchResult r =
        refiner.localize(ranges, angle_min_rad, angle_increment_rad, s.x,
                         s.y, s.heading, map, stamp_ns, body_roll_rad,
                         body_pitch_rad, body_z_m);
    if (!r.accepted) continue;
    // Survivors refining to the same physical pose are one hypothesis;
    // without this merge they would false-trigger the twin check below.
    const auto same = [&](const ScanMatchResult& v) {
      return std::hypot(v.x_m - r.x_m, v.y_m - r.y_m) <
                 config_.refine_ambiguity_distance_m &&
             std::abs(core::wrap_angle(v.yaw_rad - r.yaw_rad)) <
                 config_.refine_ambiguity_heading_rad;
    };
    const auto existing =
        std::find_if(refined.begin(), refined.end(), same);
    if (existing != refined.end()) {
      if (r.score > existing->score) *existing = r;
      continue;
    }
    refined.push_back(r);
  }
  if (refined.empty()) {
    result.rejection_reason = "no refined hypothesis accepted";
    return result;
  }
  // Final validation pass (master's third stage): every refined survivor
  // is re-searched in a tight window with expected-range raycast evidence
  // (would the map produce this scan from this pose?). Endpoint ranking
  // degrades gracefully on unknown cells, so without this stage a sparse
  // ambiguous scan refines alone and publishes a confident fix at the
  // wrong aisle. Hits-only beam budget, stride-downsampled.
  std::vector<ScanBeam> final_beams;
  {
    const std::vector<ScanBeam> full = coarse_matcher.prepare_beams(
        ranges, angle_min_rad, angle_increment_rad, ranges.size());
    std::size_t hits = 0;
    for (const auto& beam : full) {
      if (beam.hit) ++hits;
    }
    const std::size_t stride = std::max<std::size_t>(
        1, (hits + config_.final_max_returns - 1) / config_.final_max_returns);
    std::size_t seen = 0;
    for (const auto& beam : full) {
      if (!beam.hit) continue;
      if (seen++ % stride != 0) continue;
      final_beams.push_back(beam);
      if (final_beams.size() >= config_.final_max_returns) break;
    }
  }
  std::vector<ScanMatchResult> checked;
  const int final_steps = static_cast<int>(std::ceil(
      config_.final_translation_m / config_.final_translation_step_m));
  const int final_heading_steps = static_cast<int>(std::ceil(
      config_.final_heading_rad / config_.final_heading_step_rad));
  for (const auto& s : refined) {
    ScanMatchResult best;
    best.accepted = false;
    best.score = -std::numeric_limits<double>::infinity();
    for (int iy = -final_steps; iy <= final_steps; ++iy) {
      for (int ix = -final_steps; ix <= final_steps; ++ix) {
        for (int ih = -final_heading_steps; ih <= final_heading_steps;
             ++ih) {
          const double cx =
              s.x_m + ix * config_.final_translation_step_m;
          const double cy =
              s.y_m + iy * config_.final_translation_step_m;
          const double heading = core::wrap_angle(
              s.yaw_rad + ih * config_.final_heading_step_rad);
          const RaycastEvidence ev = raycast_evidence(
              map, final_beams, cx, cy, heading, body_roll_rad,
              body_pitch_rad, base_from_lidar_, config_.max_range_m);
          if (!ev.has_evidence) continue;
          if (ev.score > best.score) {
            best.accepted = true;
            best.x_m = cx;
            best.y_m = cy;
            best.yaw_rad = heading;
            best.score = ev.score;
            best.overlap = ev.overlap;
            // Sigmas ride from the refine window (unchanged blending
            // semantics downstream); score/overlap/pose are raycast.
            best.position_sigma_m = s.position_sigma_m;
            best.heading_sigma_rad = s.heading_sigma_rad;
          }
        }
      }
    }
    if (!best.accepted) continue;
    // Same physical pose, one hypothesis (same merge as the refine loop
    // above; kept at the tighter refine ambiguity instead of master's
    // 0.75 m / 0.70 rad final dedupe, i.e. more conservative — near-twins
    // compete here and the margin gates below decide).
    const auto same = [&](const ScanMatchResult& v) {
      return std::hypot(v.x_m - best.x_m, v.y_m - best.y_m) <
                 config_.refine_ambiguity_distance_m &&
             std::abs(core::wrap_angle(v.yaw_rad - best.yaw_rad)) <
                 config_.refine_ambiguity_heading_rad;
    };
    const auto existing =
        std::find_if(checked.begin(), checked.end(), same);
    if (existing != checked.end()) {
      if (best.score > existing->score) *existing = best;
      continue;
    }
    checked.push_back(best);
  }
  if (checked.empty()) {
    result.rejection_reason = "no raycast-validated hypothesis accepted";
    return result;
  }
  std::sort(checked.begin(), checked.end(),
            [](const ScanMatchResult& a, const ScanMatchResult& b) {
              if (a.score != b.score) return a.score > b.score;
              if (a.x_m != b.x_m) return a.x_m < b.x_m;
              if (a.y_m != b.y_m) return a.y_m < b.y_m;
              return a.yaw_rad < b.yaw_rad;
            });
  const ScanMatchResult& best = checked.front();
  // Overlap applies to a lone winner too (master): a single surviving
  // hypothesis that explains too little of the scan is unobservable, not
  // a confident fix. Without this, one wrong aisle refining alone with a
  // mediocre raycast score would publish a teleport with margin 1.0.
  if (!std::isfinite(best.score) ||
      best.overlap < config_.minimum_overlap) {
    result.score = best.score;
    result.overlap = best.overlap;
    result.rejection_reason = "global map overlap below acceptance gate";
    return result;
  }
  // Ambiguity across validated hypotheses: two surviving places explaining
  // the scan nearly equally well is unobservable, not confidence.
  if (checked.size() > 1) {
    const double second_score = checked[1].score;
    const double second_overlap = checked[1].overlap;
    if (best.score - second_score < config_.minimum_score_margin) {
      result.score = best.score;
      result.overlap = best.overlap;
      result.score_margin = best.score - second_score;
      result.rejection_reason = "global match is geometrically ambiguous";
      fill_runner_ups(result, checked);
      return result;
    }
    if (best.overlap - second_overlap <
        config_.equivalent_support_overlap_tolerance) {
      result.score = best.score;
      result.overlap = best.overlap;
      result.score_margin = best.score - second_score;
      result.rejection_reason = "global match has equivalent support";
      fill_runner_ups(result, checked);
      return result;
    }
  }
  result.accepted = true;
  result.x_m = best.x_m;
  result.y_m = best.y_m;
  result.yaw_rad = best.yaw_rad;
  result.position_sigma_m = best.position_sigma_m;
  result.heading_sigma_rad = best.heading_sigma_rad;
  result.score = best.score;
  result.overlap = best.overlap;
  result.score_margin =
      checked.size() > 1 ? best.score - checked[1].score : 1.0;
  fill_runner_ups(result, checked);
  return result;
}

}  // namespace lost_signal::robot::autonomy
