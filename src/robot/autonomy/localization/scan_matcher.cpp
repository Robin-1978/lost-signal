#include "robot/autonomy/localization/scan_matcher.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
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

namespace {

struct ExpectedRangeEvidence {
 double score = -std::numeric_limits<double>::infinity();
 double overlap = 0.0;
 bool valid = false;
};

std::optional<double> expected_range_cs(const iface::MapSnapshot& map,
 double sx, double sy, double dx, double dy, double max_range) {
 const double step = std::max(0.05, map.resolution_m * 0.5);
 for (double distance = step; distance < max_range; distance += step) {
 const int gx = static_cast<int>(std::floor(
 (sx + distance * dx - map.origin_x_m) / map.resolution_m));
 const int gy = static_cast<int>(std::floor(
 (sy + distance * dy - map.origin_y_m) / map.resolution_m));
 if (gx < 0 || gy < 0 || gx >= static_cast<int>(map.width) ||
 gy >= static_cast<int>(map.height)) {
 return std::nullopt;
 }
 if (map.occupancy[static_cast<std::size_t>(gy) * map.width + gx] >= 50) {
 return distance;
 }
 }
 return max_range;
}

ExpectedRangeEvidence expected_range_evidence(
 const iface::MapSnapshot& map, const std::vector<ScanBeam>& beams,
 double base_x_m, double base_y_m, double yaw_rad, double body_roll_rad,
 double body_pitch_rad, const core::RigidTransform& base_from_lidar,
 double max_range_m) {
 ExpectedRangeEvidence out;
 const core::Quaternion body_q =
 core::quaternion_from_rpy(body_roll_rad, body_pitch_rad, yaw_rad);
 const core::Quaternion sensor_q =
 core::quaternion_multiply(body_q, base_from_lidar.rotation);
 const core::Vec3 mount_off =
 core::rotate_vector(body_q, base_from_lidar.translation_m);
 const double sx = base_x_m + mount_off.x;
 const double sy = base_y_m + mount_off.y;
 double score = 0.0;
 double total_weight = 0.0;
 std::size_t hit_returns = 0;
 std::size_t hit_matches = 0;
 for (const auto& beam : beams) {
 const core::Vec3 direction = core::rotate_vector(
 sensor_q, {std::cos(beam.angle), std::sin(beam.angle), 0.0});
 const double azimuth = std::atan2(direction.y, direction.x);
 const auto expected = expected_range_cs(
 map, sx, sy, std::cos(azimuth), std::sin(azimuth), max_range_m);
 if (!expected.has_value()) {
 // A cropped or partially authored survey says nothing beyond its boundary.
 // Do not let the rectangular map extent manufacture a localization peak.
 continue;
 }
 const double error = std::abs(*expected - beam.range);
 const double quantum = std::max(0.05, map.resolution_m);
 const double quantized_error = std::round(error / quantum) * quantum;
 const double likelihood = std::exp(
 -quantized_error / std::max(0.05, 2.0 * map.resolution_m));
 const double weight = beam.hit ? 1.0 : 0.15;
 score += weight * likelihood;
 total_weight += weight;
 if (beam.hit) {
 ++hit_returns;
 if (*expected < max_range_m &&
 error <= std::max(0.12, 2.0 * map.resolution_m)) {
 ++hit_matches;
 }
 }
 }
 if (hit_returns == 0 || !(total_weight > 0.0)) return out;
 out.valid = true;
 out.score = score / total_weight;
 out.overlap = static_cast<double>(hit_matches) / hit_returns;
 return out;
}

} // namespace

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
 // Endpoint scores are cell-quantized. Candidates within roughly one and a
 // half cells lie on the same physical raster peak even when a configured
 // metric radius was chosen for a finer map. Treating the exact boundary as
 // a second place creates a zero-margin false ambiguity on coarse surveys.
 const double ambiguity_distance_m = config_.expected_range_validation
 ? std::max(config_.ambiguity_distance_m, 1.5 * map.resolution_m)
 : config_.ambiguity_distance_m;
  const auto same_hypothesis = [&](const Hypothesis& a, const Hypothesis& b) {
 const double distance_m = std::hypot(a.x - b.x, a.y - b.y);
 const bool same_position = config_.expected_range_validation
 ? distance_m <= ambiguity_distance_m
 : distance_m < ambiguity_distance_m;
 return same_position &&
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
 const auto rank_hypotheses = [](const Hypothesis& a,
 const Hypothesis& b) {
 if (a.score != b.score) return a.score > b.score;
 if (a.x != b.x) return a.x < b.x;
 if (a.y != b.y) return a.y < b.y;
 return a.heading < b.heading;
 };
 if (config_.expected_range_validation) {
 // Endpoint occupancy is a cheap shortlist, but a thick survey cell creates
 // plateaus: several poses can place every return somewhere inside the same
 // occupied cells. Re-rank only the bounded survivor set by expected ranges
 // so penetration depth and free distance select the physical scan pose.
 for (auto& hypothesis : hypotheses) {
 const ExpectedRangeEvidence evidence = expected_range_evidence(
 map, beams, hypothesis.x, hypothesis.y, hypothesis.heading,
 body_roll_rad, body_pitch_rad, base_from_lidar_, config_.max_range_m);
 if (evidence.valid) {
 hypothesis.score = evidence.score;
 hypothesis.overlap = evidence.overlap;
 } else {
 hypothesis.score = -std::numeric_limits<double>::infinity();
 hypothesis.overlap = 0.0;
 }
 }
 }
 std::sort(hypotheses.begin(), hypotheses.end(), rank_hypotheses);
  const Hypothesis& best = hypotheses.front();
  const double second_score =
      hypotheses.size() > 1
          ? hypotheses[1].score
          : -std::numeric_limits<double>::infinity();
 const auto likelihood_evidence = [&](double x, double y, double heading) {
 PoseScore out;
 if (config_.expected_range_validation) {
 const ExpectedRangeEvidence evidence = expected_range_evidence(
 map, beams, x, y, heading, body_roll_rad, body_pitch_rad,
 base_from_lidar_, config_.max_range_m);
 out.has_evidence = evidence.valid;
 out.score = evidence.score;
 out.overlap = evidence.overlap;
 out.usable_returns = hit_returns;
 return out;
 }
 return batch.score(0, x, y, heading, map, body_z_m);
 };
 const auto likelihood_score = [&](double x, double y, double heading) {
 const PoseScore evidence = likelihood_evidence(x, y, heading);
 return evidence.has_evidence ? evidence.score
 : -std::numeric_limits<double>::infinity();
 };
 const auto populate_pose_information = [&](const Hypothesis& pose) {
 result.x_m = pose.x;
 result.y_m = pose.y;
 result.yaw_rad = pose.heading;
 // Approximate the local negative-log-likelihood Hessian around the winner.
 // Its rank is the measurement's actual observability: a corridor may
 // constrain lateral position and yaw while contributing zero information
 // along its axis. Passing information, rather than an isotropic guessed
 // sigma, prevents one scan from erasing uncertainty in an unobserved
 // direction.
 const double translation_probe_m = std::max(
 config_.translation_step_m, map.resolution_m);
 const double heading_probe_rad = std::max(
 config_.heading_step_rad,
 std::atan2(map.resolution_m, std::max(map.resolution_m,
 config_.max_range_m)));
 const auto cost = [](double score) {
 return std::isfinite(score)
 ? -std::log(std::max(1e-6, score))
 : std::numeric_limits<double>::infinity();
 };
 const double c0 = cost(likelihood_score(pose.x, pose.y, pose.heading));
 const double cxp = cost(likelihood_score(
 pose.x + translation_probe_m, pose.y, pose.heading));
 const double cxm = cost(likelihood_score(
 pose.x - translation_probe_m, pose.y, pose.heading));
 const double cyp = cost(likelihood_score(
 pose.x, pose.y + translation_probe_m, pose.heading));
 const double cym = cost(likelihood_score(
 pose.x, pose.y - translation_probe_m, pose.heading));
 const double cpp = cost(likelihood_score(
 pose.x + translation_probe_m, pose.y + translation_probe_m,
 pose.heading));
 const double cpm = cost(likelihood_score(
 pose.x + translation_probe_m, pose.y - translation_probe_m,
 pose.heading));
 const double cmp = cost(likelihood_score(
 pose.x - translation_probe_m, pose.y + translation_probe_m,
 pose.heading));
 const double cmm = cost(likelihood_score(
 pose.x - translation_probe_m, pose.y - translation_probe_m,
 pose.heading));
 const double chp = cost(likelihood_score(
 pose.x, pose.y, core::wrap_angle(pose.heading + heading_probe_rad)));
 const double chm = cost(likelihood_score(
 pose.x, pose.y, core::wrap_angle(pose.heading - heading_probe_rad)));
 const bool finite_neighborhood = std::isfinite(c0) &&
 std::isfinite(cxp) && std::isfinite(cxm) &&
 std::isfinite(cyp) && std::isfinite(cym) &&
 std::isfinite(cpp) && std::isfinite(cpm) &&
 std::isfinite(cmp) && std::isfinite(cmm) &&
 std::isfinite(chp) && std::isfinite(chm);
 if (finite_neighborhood) {
 const double translation_step2 =
 translation_probe_m * translation_probe_m;
 const double heading_step2 = heading_probe_rad * heading_probe_rad;
 const double evidence_scale =
 static_cast<double>(std::max<std::size_t>(1, hit_returns));
 double ixx = evidence_scale * std::max(
 0.0, (cxp - 2.0 * c0 + cxm) / translation_step2);
 double iyy = evidence_scale * std::max(
 0.0, (cyp - 2.0 * c0 + cym) / translation_step2);
 double ixy = evidence_scale *
 (cpp - cpm - cmp + cmm) / (4.0 * translation_step2);
 const double cross_limit = std::sqrt(std::max(0.0, ixx * iyy));
 ixy = std::clamp(ixy, -cross_limit, cross_limit);
 const double ihh = evidence_scale * std::max(
 0.0, (chp - 2.0 * c0 + chm) / heading_step2);
 result.pose_information = {
 ixx, ixy, 0.0,
 ixy, iyy, 0.0,
 0.0, 0.0, ihh};
 result.pose_information_valid =
 ixx > 1e-9 || iyy > 1e-9 || ihh > 1e-9;
 const double trace = ixx + iyy;
 const double discriminant = std::sqrt(std::max(
 0.0, (ixx - iyy) * (ixx - iyy) + 4.0 * ixy * ixy));
 const double weakest_position_information =
 std::max(0.0, 0.5 * (trace - discriminant));
 result.position_sigma_m = weakest_position_information > 1e-9
 ? 1.0 / std::sqrt(weakest_position_information)
 : std::max(map.resolution_m, config_.search_translation_m);
 result.heading_sigma_rad = ihh > 1e-9
 ? 1.0 / std::sqrt(ihh)
 : std::max(config_.heading_step_rad, config_.search_heading_rad);
 }
 if (!result.pose_information_valid) {
 result.position_sigma_m = std::max(
 map.resolution_m, config_.search_translation_m);
 result.heading_sigma_rad = std::max(
 config_.heading_step_rad, config_.search_heading_rad);
 }
 result.position_sigma_m = std::max(0.02, result.position_sigma_m);
 result.heading_sigma_rad = std::max(0.01, result.heading_sigma_rad);
 };
  if (!std::isfinite(best.score) || best.overlap < config_.minimum_overlap) {
    result.rejection_reason = "map overlap below acceptance gate";
    return result;
  }
 populate_pose_information(best);
  // Nearly tied best hypotheses = degeneracy (repetitive corridor), not
 // a complete pose observation. Keep only information shared by the tied
 // modes; the estimate cannot move along their unobservable separation.
  if (std::isfinite(second_score) &&
      best.score - second_score < config_.minimum_score_margin) {
    result.score = best.score;
    result.overlap = best.overlap;
    result.score_margin = best.score - second_score;
    result.rejection_reason = "map match is geometrically ambiguous";
 const Hypothesis& second = hypotheses[1];
 const double delta[3] = {
 second.x - best.x,
 second.y - best.y,
 core::wrap_angle(second.heading - best.heading)};
 double shared_information_disagreement = 0.0;
 for (int row = 0; row < 3; ++row) {
 for (int column = 0; column < 3; ++column) {
 shared_information_disagreement += delta[row] *
 result.pose_information[static_cast<std::size_t>(row * 3 + column)] *
 delta[column];
 }
 }
 // One standard-error squared is a statistical agreement test, not a tuned
 // distance: tied modes may share only directions in which their measured
 // information agrees. Distinct corners/headings remain a hard rejection.
 constexpr double kOneStandardErrorSquared = 1.0;
 result.partial_observation_valid = result.pose_information_valid &&
 std::isfinite(shared_information_disagreement) &&
 shared_information_disagreement <= kOneStandardErrorSquared;
    return result;
  }
 // A winner on the edge of the bounded search is not necessarily a local
 // optimum. Probe one lattice step farther in every clipped direction. If
 // the score stays within the same ambiguity margin (or improves), the peak
 // lies outside the trusted local window and this scan must not be applied as
 // a sequence of small map-frame corrections. The supervisor's covariance
 // policy can start the wider, temporally confirmed recovery path instead.
 const double searched_translation_m = t_steps * config_.translation_step_m;
 const double searched_heading_rad = h_steps * config_.heading_step_rad;
 const double dx = best.x - odom_x_m;
 const double dy = best.y - odom_y_m;
 const double dh = core::wrap_angle(best.heading - odom_heading_rad);
 bool clipped_peak = false;
 std::array<double, 3> clipped_direction{};
 const auto unresolved_outside = [&](double x, double y, double heading) {
 const double outside_score = likelihood_score(x, y, heading);
 return std::isfinite(outside_score) &&
 best.score - outside_score < config_.minimum_score_margin;
 };
 if (t_steps > 0 &&
 std::abs(dx) + 1e-9 >= searched_translation_m) {
 const double step = std::copysign(config_.translation_step_m, dx);
 if (unresolved_outside(best.x + step, best.y, best.heading)) {
 clipped_peak = true;
 clipped_direction[0] = step;
 }
 }
 if (t_steps > 0 &&
 std::abs(dy) + 1e-9 >= searched_translation_m) {
 const double step = std::copysign(config_.translation_step_m, dy);
 if (unresolved_outside(best.x, best.y + step, best.heading)) {
 clipped_peak = true;
 clipped_direction[1] = step;
 }
 }
 if (h_steps > 0 &&
 std::abs(dh) + 1e-9 >= searched_heading_rad) {
 const double step = std::copysign(config_.heading_step_rad, dh);
 if (unresolved_outside(
 best.x, best.y, core::wrap_angle(best.heading + step))) {
 clipped_peak = true;
 clipped_direction[2] = step;
 }
 }
 if (clipped_peak) {
 result.score = best.score;
 result.overlap = best.overlap;
 result.score_margin = std::isfinite(second_score)
 ? best.score - second_score : 1.0;
 // A bounded search can reach its edge because the likelihood is flat in a
 // truly unobservable direction (for example, translation along a uniform
 // wall). Rejecting that as a contradictory scan makes temporal recovery
 // repeatedly discard an otherwise supported global hypothesis. Accept only
 // the information shared across the ridge when the outward direction lies
 // in the numerical nullspace of the measured Hessian; an observed boundary
 // direction remains a hard rejection and must widen through the supervisor.
 double information_energy = 0.0;
 double cancellation_scale = 0.0;
 for (int row = 0; row < 3; ++row) {
 for (int column = 0; column < 3; ++column) {
 const double term = clipped_direction[static_cast<std::size_t>(row)] *
 result.pose_information[static_cast<std::size_t>(row * 3 + column)] *
 clipped_direction[static_cast<std::size_t>(column)];
 information_energy += term;
 cancellation_scale += std::abs(term);
 }
 }
 const double numerical_rank_tolerance =
 std::sqrt(std::numeric_limits<double>::epsilon());
 const bool boundary_unobservable = result.pose_information_valid &&
 std::isfinite(information_energy) &&
 (cancellation_scale <= std::numeric_limits<double>::epsilon() ||
 std::abs(information_energy) <=
 numerical_rank_tolerance * cancellation_scale);
 if (boundary_unobservable) {
 result.partial_observation_valid = true;
 result.rejection_reason = "map match is geometrically ambiguous";
 return result;
 }
 result.rejection_reason = "map optimum reaches search boundary";
 return result;
 }
 // Odometry is the local search prior, not merely the centre of a lattice.
 // Endpoint rasters contain broad plateaus where a neighbouring cell can
 // win by quantization noise and then walk map->odom in the same direction
 // on every scan. A nonzero correction must improve the likelihood by the
 // same declared margin used to distinguish separate geometric modes. An
 // unchanged prior can still be accepted as a supported no-op observation.
 const PoseScore prior_evidence = likelihood_evidence(
 odom_x_m, odom_y_m, odom_heading_rad);
 const double prior_score = prior_evidence.has_evidence
 ? prior_evidence.score : -std::numeric_limits<double>::infinity();
 const bool shifted_from_prior =
 std::hypot(best.x - odom_x_m, best.y - odom_y_m) > 1e-9 ||
 std::abs(core::wrap_angle(best.heading - odom_heading_rad)) > 1e-9;
 if (config_.require_prior_improvement && shifted_from_prior &&
 std::isfinite(prior_score) &&
 best.score - prior_score < config_.minimum_score_margin) {
 // The scan supports the established prior, but does not justify moving
 // away from it. Preserve the prior as a zero-innovation observation and
 // retain only the local Hessian information measured there. This prevents
 // cell quantization from walking map->odom while still observing, for
 // example, wall-normal position and heading.
 if (prior_evidence.overlap >= config_.minimum_overlap) {
 const Hypothesis supported_prior{
 odom_x_m, odom_y_m, odom_heading_rad,
 prior_score, prior_evidence.overlap};
 populate_pose_information(supported_prior);
 result.accepted = true;
 result.score = prior_score;
 result.overlap = prior_evidence.overlap;
 result.score_margin = std::isfinite(second_score)
 ? best.score - second_score : 1.0;
 result.rejection_reason = "accepted at odometry prior";
 return result;
 }
 result.score = best.score;
 result.overlap = best.overlap;
 result.score_margin = best.score - prior_score;
 result.rejection_reason = "map match does not improve odometry prior";
 return result;
 }
  result.accepted = true;
  result.score = best.score;
  result.overlap = best.overlap;
  result.score_margin = std::isfinite(second_score) ? best.score - second_score
                                                   : 1.0;
 result.rejection_reason = "accepted";
  return result;
}

}  // namespace lost_signal::robot::autonomy
