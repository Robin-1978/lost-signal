#include "robot/autonomy/localization/scan_matcher.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "robot/core/transform.hpp"

namespace lost_signal::robot::autonomy {

namespace {

struct GridProjector {
  const iface::MapSnapshot& map;
  explicit GridProjector(const iface::MapSnapshot& m) : map(m) {}

  bool project(double wx, double wy, int& gx, int& gy) const {
    gx = static_cast<int>(std::floor((wx - map.origin_x_m) / map.resolution_m));
    gy = static_cast<int>(std::floor((wy - map.origin_y_m) / map.resolution_m));
    return gx >= 0 && gy >= 0 && gx < static_cast<int>(map.width) &&
           gy < static_cast<int>(map.height);
  }
  int at(int gx, int gy) const {
    return map.occupancy[static_cast<std::size_t>(gy) * map.width + gx];
  }
};

double endpoint_score(const GridProjector& projector, double wx, double wy) {
  int gx, gy;
  if (!projector.project(wx, wy, gx, gy)) return -0.25;  // outside: evidence against
  const int value = projector.at(gx, gy);
  if (value < 0) return 0.0;  // unknown must not become false certainty
  return value >= 50 ? 1.0 : -0.20;
}

double free_ray_score_cs(const GridProjector& projector, double sx,
                           double sy, double beam_cos, double beam_sin,
                           double max_range) {
  // No-return beams are weak evidence: probe a few interior points plus the
  // endpoint instead of marching the whole ray per hypothesis.
  const double probes[3] = {
      std::max(0.05, max_range * 0.5),
      std::max(0.05, max_range * 0.8),
      std::max(0.05, max_range - std::max(0.05, projector.map.resolution_m))};
  bool observed_known = false;
  for (double distance : probes) {
    if (!(distance < max_range + 1e-9)) continue;
    int gx, gy;
    if (!projector.project(sx + distance * beam_cos,
                           sy + distance * beam_sin, gx, gy)) {
      return observed_known ? 0.0 : -0.25;
    }
    const int value = projector.at(gx, gy);
    if (value < 0) {
      if (!observed_known) return -0.25;
      continue;
    }
    observed_known = true;
    if (value >= 50) return -1.0;
  }
  return observed_known ? 1.0 : 0.0;
}

double free_ray_score(const GridProjector& projector, double sx, double sy,
                      double beam_angle, double max_range) {
  return free_ray_score_cs(projector, sx, sy, std::cos(beam_angle),
                           std::sin(beam_angle), max_range);
}

}  // namespace

ScanMatcher::ScanMatcher(ScanMatcherConfig config) : config_(config) {}

ScanMatcher::Batch::Batch(const ScanMatcher& matcher,
                           const std::vector<ScanBeam>& beams,
                           double body_roll_rad, double body_pitch_rad)
    : mount_(&matcher.base_from_lidar_),
      roll_rad_(body_roll_rad),
      pitch_rad_(body_pitch_rad) {
  beams_.reserve(beams.size());
  for (const auto& b : beams) {
    BeamT t;
    t.range = b.range;
    t.cb = std::cos(b.angle);
    t.sb = std::sin(b.angle);
    t.hit = b.hit;
    beams_.push_back(t);
    if (b.hit) ++usable_returns_;
  }
}

ScanMatcher::Batch::RotSet& ScanMatcher::Batch::rot_for(std::size_t slot,
                                                        double yaw_rad) {
  if (slot >= rots_.size()) rots_.resize(slot + 1);
  RotSet& r = rots_[slot];
  if (r.ready && r.yaw_rad == yaw_rad) return r;  // stable slot: reuse
  r.yaw_rad = yaw_rad;
  const core::Quaternion body_q =
      core::quaternion_from_rpy(roll_rad_, pitch_rad_, yaw_rad);
  r.sensor_q =
      core::quaternion_multiply(body_q, mount().rotation);
  r.mount_off =
      core::rotate_vector(body_q, mount().translation_m);
  r.dx.assign(beams_.size(), 0.0);
  r.dy.assign(beams_.size(), 0.0);
  r.free_c.assign(beams_.size(), 1.0);
  r.free_s.assign(beams_.size(), 0.0);
  for (std::size_t i = 0; i < beams_.size(); ++i) {
    const core::Vec3 d = core::rotate_vector(
        r.sensor_q, {beams_[i].cb, beams_[i].sb, 0.0});
    r.dx[i] = d.x;
    r.dy[i] = d.y;
    const double az = std::atan2(d.y, d.x);
    r.free_c[i] = std::cos(az);
    r.free_s[i] = std::sin(az);
  }
  r.ready = true;
  return r;
}

PoseScore ScanMatcher::Batch::score(std::size_t slot, double x_m, double y_m,
                                    double yaw_rad,
                                    const iface::MapSnapshot& map,
                                    double body_z_m) {
  (void)body_z_m;  // 2D raster: height rides the sensor pose, not the score
  PoseScore out;
  out.usable_returns = usable_returns_;
  if (beams_.empty()) return out;
  RotSet& r = rot_for(slot, yaw_rad);
  const GridProjector projector(map);
  const double sx = x_m + r.mount_off.x;
  const double sy = y_m + r.mount_off.y;
  double score = 0.0;
  double total_weight = 0.0;
  std::size_t known = 0, matches = 0;
  for (std::size_t i = 0; i < beams_.size(); ++i) {
    const BeamT& beam = beams_[i];
    const double wx = sx + beam.range * r.dx[i];
    const double wy = sy + beam.range * r.dy[i];
    const double value =
        beam.hit ? endpoint_score(projector, wx, wy)
                 : free_ray_score_cs(projector, sx, sy, r.free_c[i],
                                     r.free_s[i], beam.range);
    const double weight = beam.hit ? 1.0 : 0.15;
    score += weight * value;
    total_weight += weight;
    if (value != 0.0) {
      ++known;
      if (value > 0.0 && beam.hit) ++matches;
    }
  }
  if (known == 0 || usable_returns_ == 0) return out;
  out.has_evidence = true;
  out.overlap = static_cast<double>(matches) / static_cast<double>(usable_returns_);
  out.score = score / std::max(1e-9, total_weight);
  return out;
}

std::vector<ScanBeam> ScanMatcher::prepare_beams(
    const std::vector<float>& ranges, double angle_min_rad,
    double angle_increment_rad, std::size_t max_beams) const {
  std::vector<ScanBeam> beams;
  if (ranges.empty() || !(angle_increment_rad > 0.0) || max_beams == 0) {
    return beams;
  }
  beams.reserve(ranges.size());
  for (std::size_t i = 0; i < ranges.size(); ++i) {
    const double range = ranges[i];
    if (!std::isfinite(range) || range <= 0.05) continue;
    const bool hit = range < config_.max_range_m;
    beams.push_back({std::min(range, config_.max_range_m),
                     angle_min_rad + angle_increment_rad * i, hit});
  }
  if (beams.size() > max_beams) {
    const std::size_t stride =
        (beams.size() + max_beams - 1) / max_beams;
    std::vector<ScanBeam> selected;
    selected.reserve(max_beams);
    for (std::size_t i = 0; i < beams.size() && selected.size() < max_beams;
         i += stride) {
      selected.push_back(beams[i]);
    }
    beams = std::move(selected);
  }
  return beams;
}

PoseScore ScanMatcher::score_pose(double x_m, double y_m, double yaw_rad,
                                  const std::vector<ScanBeam>& beams,
                                  const iface::MapSnapshot& map,
                                  double body_roll_rad,
                                  double body_pitch_rad,
                                  double body_z_m) const {
  // Single implementation: one-shot batch scores identically (same ops,
  // same order), so the public path and the batched search path cannot
  // disagree.
  Batch batch(*this, beams, body_roll_rad, body_pitch_rad);
  return batch.score(0, x_m, y_m, yaw_rad, map, body_z_m);
}

ScanMatchResult ScanMatcher::localize(
    const std::vector<float>& ranges, double angle_min_rad,
    double angle_increment_rad, double odom_x_m, double odom_y_m,
    double odom_heading_rad, const iface::MapSnapshot& map,
    core::MonotonicNs stamp_ns, double body_roll_rad, double body_pitch_rad,
    double body_z_m) const {
  ScanMatchResult result;
  (void)stamp_ns;
  if (!std::isfinite(odom_x_m) || !std::isfinite(odom_y_m) ||
      !std::isfinite(odom_heading_rad)) {
    result.rejection_reason = "invalid odometry prior";
    return result;
  }
  if (ranges.empty() || !(angle_increment_rad > 0.0)) {
    result.rejection_reason = "invalid scan";
    return result;
  }
  if (!map.has_raster()) {
    result.rejection_reason = "map snapshot has no raster";
    return result;
  }
  if (!(config_.search_translation_m >= 0.0) ||
      !(config_.search_heading_rad >= 0.0) ||
      !(config_.translation_step_m > 0.0) ||
      !(config_.heading_step_rad > 0.0) ||
      config_.maximum_returns < config_.minimum_returns ||
      !(config_.ambiguity_distance_m > 0.0) ||
      !(config_.ambiguity_heading_rad > 0.0)) {
    result.rejection_reason = "invalid matcher configuration";
    return result;
  }

  std::vector<ScanBeam> beams =
      prepare_beams(ranges, angle_min_rad, angle_increment_rad,
                    config_.maximum_returns);
  std::size_t hit_returns = 0;
  for (const auto& beam : beams) {
    if (beam.hit) ++hit_returns;
  }
  result.usable_returns = hit_returns;
  if (hit_returns < config_.minimum_returns) {
    result.rejection_reason = "insufficient geometric returns";
    return result;
  }

  struct Hypothesis {
    double x = 0.0, y = 0.0, heading = 0.0;
    double score = -std::numeric_limits<double>::infinity();
    double overlap = 0.0;
  };
  const auto same_hypothesis = [&](const Hypothesis& a, const Hypothesis& b) {
    return std::hypot(a.x - b.x, a.y - b.y) < config_.ambiguity_distance_m &&
           std::abs(core::wrap_angle(a.heading - b.heading)) <
               config_.ambiguity_heading_rad;
  };
  constexpr std::size_t kMaxHypotheses = 24;
  std::vector<Hypothesis> hypotheses;
  hypotheses.reserve(kMaxHypotheses);
  const int t_steps = static_cast<int>(
      std::ceil(config_.search_translation_m / config_.translation_step_m));
  const int h_steps = static_cast<int>(
      std::ceil(config_.search_heading_rad / config_.heading_step_rad));
  // Batched scoring: beam trig once per call, rotations once per heading
  // slot (slot = ih + h_steps — stable per heading, traversal order
  // untouched so hypothesis ranking cannot change).
  Batch batch(*this, beams, body_roll_rad, body_pitch_rad);
  for (int iy = -t_steps; iy <= t_steps; ++iy) {
    for (int ix = -t_steps; ix <= t_steps; ++ix) {
      for (int ih = -h_steps; ih <= h_steps; ++ih) {
        const double cx = odom_x_m + ix * config_.translation_step_m;
        const double cy = odom_y_m + iy * config_.translation_step_m;
        const double heading =
            core::wrap_angle(odom_heading_rad + ih * config_.heading_step_rad);
        const PoseScore pose = batch.score(
            static_cast<std::size_t>(ih + h_steps), cx, cy, heading, map,
            body_z_m);
        if (!pose.has_evidence) continue;
        Hypothesis h{cx, cy, heading, pose.score, pose.overlap};
        const auto existing = std::find_if(
            hypotheses.begin(), hypotheses.end(),
            [&](const Hypothesis& v) { return same_hypothesis(v, h); });
        if (existing != hypotheses.end()) {
          if (h.score > existing->score) *existing = h;
          continue;
        }
        if (hypotheses.size() < kMaxHypotheses) {
          hypotheses.push_back(h);
          continue;
        }
        const auto weakest = std::min_element(
            hypotheses.begin(), hypotheses.end(),
            [](const Hypothesis& a, const Hypothesis& b) {
              return a.score < b.score;
            });
        if (h.score > weakest->score) *weakest = h;
      }
    }
  }
  if (hypotheses.empty()) {
    result.rejection_reason = "map overlap below acceptance gate";
    return result;
  }
  std::sort(hypotheses.begin(), hypotheses.end(),
            [](const Hypothesis& a, const Hypothesis& b) {
              if (a.score != b.score) return a.score > b.score;
              if (a.x != b.x) return a.x < b.x;
              if (a.y != b.y) return a.y < b.y;
              return a.heading < b.heading;
            });
  const Hypothesis& best = hypotheses.front();
  const double second_score =
      hypotheses.size() > 1
          ? hypotheses[1].score
          : -std::numeric_limits<double>::infinity();
  if (!std::isfinite(best.score) || best.overlap < config_.minimum_overlap) {
    result.rejection_reason = "map overlap below acceptance gate";
    return result;
  }
  // Nearly tied best hypotheses = degeneracy (repetitive corridor), not
  // confidence. Reject rather than drag the estimate to a wrong aisle.
  if (std::isfinite(second_score) &&
      best.score - second_score < config_.minimum_score_margin) {
    result.score = best.score;
    result.overlap = best.overlap;
    result.score_margin = best.score - second_score;
    result.rejection_reason = "map match is geometrically ambiguous";
    return result;
  }
  result.accepted = true;
  result.x_m = best.x;
  result.y_m = best.y;
  result.yaw_rad = best.heading;
  result.position_sigma_m = std::max(0.02, map.resolution_m * 0.75);
  result.heading_sigma_rad = std::max(0.01, config_.heading_step_rad * 1.5);
  result.score = best.score;
  result.overlap = best.overlap;
  result.score_margin = std::isfinite(second_score) ? best.score - second_score
                                                   : 1.0;
  return result;
}

}  // namespace lost_signal::robot::autonomy
