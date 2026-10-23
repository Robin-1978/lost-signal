#include "robot/autonomy/localization/global_search.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "robot/core/geometry.hpp"

namespace lost_signal::robot::autonomy {

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
      config_.coarse_beams == 0 || config_.coarse_hypotheses == 0) {
    result.rejection_reason = "invalid global search configuration";
    return result;
  }

  ScanMatcherConfig coarse_cfg;
  coarse_cfg.max_range_m = config_.max_range_m;
  coarse_cfg.minimum_returns = config_.minimum_returns;
  ScanMatcher coarse_matcher(coarse_cfg);
  coarse_matcher.set_lidar_mount(base_from_lidar_);
  const std::vector<ScanBeam> beams = coarse_matcher.prepare_beams(
      ranges, angle_min_rad, angle_increment_rad, config_.coarse_beams);
  std::size_t hit_returns = 0;
  for (const auto& beam : beams) {
    if (beam.hit) ++hit_returns;
  }
  result.usable_returns = hit_returns;
  if (hit_returns < config_.minimum_returns) {
    result.rejection_reason = "insufficient geometric returns";
    return result;
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
  // Stride in cells so visited centers sit ~step apart regardless of raster.
  const int cell_stride =
      std::max(1, static_cast<int>(std::round(step / map.resolution_m)));
  for (int gy = gy0; gy < gy1; gy += cell_stride) {
    for (int gx = gx0; gx < gx1; gx += cell_stride) {
      const int8_t cell =
          map.occupancy[static_cast<std::size_t>(gy) * map.width + gx];
      if (cell < 0 || cell >= 50) continue;  // hypotheses live in free space
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
        const PoseScore pose = coarse_matcher.score_pose(
            cx, cy, heading, beams, map, body_roll_rad, body_pitch_rad,
            body_z_m);
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
  std::sort(refined.begin(), refined.end(),
            [](const ScanMatchResult& a, const ScanMatchResult& b) {
              if (a.score != b.score) return a.score > b.score;
              if (a.x_m != b.x_m) return a.x_m < b.x_m;
              if (a.y_m != b.y_m) return a.y_m < b.y_m;
              return a.yaw_rad < b.yaw_rad;
            });
  const ScanMatchResult& best = refined.front();
  // Ambiguity across refined hypotheses: two surviving places explaining
  // the scan nearly equally well is unobservable, not confidence.
  if (refined.size() > 1) {
    const double second_score = refined[1].score;
    const double second_overlap = refined[1].overlap;
    if (best.score - second_score < config_.minimum_score_margin) {
      result.score = best.score;
      result.overlap = best.overlap;
      result.score_margin = best.score - second_score;
      result.rejection_reason = "global match is geometrically ambiguous";
      return result;
    }
    if (best.overlap - second_overlap <
        config_.equivalent_support_overlap_tolerance) {
      result.score = best.score;
      result.overlap = best.overlap;
      result.score_margin = best.score - second_score;
      result.rejection_reason = "global match has equivalent support";
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
      refined.size() > 1 ? best.score - refined[1].score : 1.0;
  return result;
}

}  // namespace lost_signal::robot::autonomy
