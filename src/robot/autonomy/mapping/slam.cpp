#include "robot/autonomy/mapping/slam.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>

#include "robot/core/geometry.hpp"
#include "robot/core/transform.hpp"

namespace lost_signal::robot::autonomy {

namespace {
// Providers predating the explicit layout leave the increment at zero:
// keep the legacy full-rotation interpretation instead of collapsing every
// beam onto one angle.
double normalized_scan_increment(double increment, std::size_t n) {
 if (std::isfinite(increment) && increment != 0.0) return increment;
 return 2.0 * core::kPi / static_cast<double>(n > 0 ? n : 1);
}
} // namespace

struct SlamSession::LoopSearchWork {
 struct Hypothesis {
 double x = 0.0;
 double y = 0.0;
 double heading = 0.0;
 double score = -std::numeric_limits<double>::infinity();
 double overlap = 0.0;
 };

 struct Lattice {
 void start(const ScanMatcherConfig& matcher_config,
 const core::RigidTransform& mount,
 const std::vector<float>& ranges, double angle_min_rad,
 double angle_increment_rad, double prior_x_m, double prior_y_m,
 double prior_heading_rad, iface::MapSnapshot search_map,
 double body_roll_rad, double body_pitch_rad, double body_z_m,
 std::optional<Pose2> motion_prior = std::nullopt) {
 config = matcher_config;
 matcher = ScanMatcher(config);
 matcher.set_lidar_mount(mount);
 map = std::move(search_map);
 prior_x = prior_x_m;
 prior_y = prior_y_m;
 prior_heading = prior_heading_rad;
 evidence_prior = motion_prior.value_or(
 Pose2{{prior_x_m, prior_y_m}, prior_heading_rad});
 z_m = body_z_m;
 roll_rad = body_roll_rad;
 pitch_rad = body_pitch_rad;
 beams = matcher.prepare_beams(ranges, angle_min_rad,
 angle_increment_rad, config.maximum_returns);
 std::size_t hits = 0;
 for (const auto& beam : beams) {
 if (beam.hit) ++hits;
 }
 result = ScanMatchResult{};
 result.usable_returns = hits;
 hypotheses.clear();
 complete = false;
 if (!map.has_raster()) {
 result.rejection_reason = "map snapshot has no raster";
 complete = true;
 return;
 }
 if (hits < config.minimum_returns) {
 result.rejection_reason = "insufficient geometric returns";
 complete = true;
 return;
 }
 batch = std::make_unique<ScanMatcher::Batch>(
 matcher, beams, body_roll_rad, body_pitch_rad);
 translation_steps = static_cast<int>(std::ceil(
 config.search_translation_m / config.translation_step_m));
 heading_steps = static_cast<int>(std::ceil(
 config.search_heading_rad / config.heading_step_rad));
 ix = -translation_steps;
 iy = -translation_steps;
 ih = -heading_steps;
 }

 bool advance(std::size_t budget) {
 if (complete) return true;
 while (budget-- > 0 && !complete) {
 const double x = prior_x + ix * config.translation_step_m;
 const double y = prior_y + iy * config.translation_step_m;
 const double heading = core::wrap_angle(
 prior_heading + ih * config.heading_step_rad);
 const PoseScore pose = batch->score(
 static_cast<std::size_t>(ih + heading_steps), x, y, heading,
 map, z_m);
 if (pose.has_evidence) consider(x, y, heading, pose);
 ++ih;
 if (ih > heading_steps) {
 ih = -heading_steps;
 ++ix;
 if (ix > translation_steps) {
 ix = -translation_steps;
 ++iy;
 if (iy > translation_steps) finish();
 }
 }
 }
 return complete;
 }

 void consider(double x, double y, double heading,
 const PoseScore& pose) {
 Hypothesis candidate{x, y, heading, pose.score, pose.overlap};
 const auto same = [&](const Hypothesis& value) {
 const double distance = std::hypot(value.x - x, value.y - y);
 const double heading_error = std::abs(
 core::wrap_angle(value.heading - heading));
 return config.expected_range_validation
 ? distance <= config.ambiguity_distance_m &&
 heading_error <= config.ambiguity_heading_rad
 : distance < config.ambiguity_distance_m &&
 heading_error < config.ambiguity_heading_rad;
 };
 const auto existing = std::find_if(
 hypotheses.begin(), hypotheses.end(), same);
 if (existing != hypotheses.end()) {
 if (candidate.score > existing->score) *existing = candidate;
 return;
 }
 constexpr std::size_t kMaxHypotheses = kMaximumScanMatchHypotheses;
 if (hypotheses.size() < kMaxHypotheses) {
 hypotheses.push_back(candidate);
 return;
 }
 const auto weakest = std::min_element(
 hypotheses.begin(), hypotheses.end(),
 [](const Hypothesis& a, const Hypothesis& b) {
 return a.score < b.score;
 });
 if (candidate.score > weakest->score) *weakest = candidate;
 }

 void finish() {
 complete = true;
 if (hypotheses.empty()) {
 result.rejection_reason = "map overlap below acceptance gate";
 return;
 }
 if (config.expected_range_validation) {
 for (auto& hypothesis : hypotheses) {
 const PoseScore evidence = matcher.score_expected_ranges(
 hypothesis.x, hypothesis.y, hypothesis.heading, beams, map,
 roll_rad, pitch_rad);
 if (evidence.has_evidence) {
 hypothesis.score = evidence.score;
 hypothesis.overlap = evidence.overlap;
 } else {
 hypothesis.score = -std::numeric_limits<double>::infinity();
 hypothesis.overlap = 0.0;
 }
 }
 }
 std::sort(hypotheses.begin(), hypotheses.end(),
 [](const Hypothesis& a, const Hypothesis& b) {
 if (a.score != b.score) return a.score > b.score;
 if (a.x != b.x) return a.x < b.x;
 if (a.y != b.y) return a.y < b.y;
 return a.heading < b.heading;
 });
 const Hypothesis& best = hypotheses.front();
 const double second_score = hypotheses.size() > 1
 ? hypotheses[1].score
 : -std::numeric_limits<double>::infinity();
 result.score = best.score;
 result.overlap = best.overlap;
 result.score_margin = std::isfinite(second_score)
 ? best.score - second_score : 1.0;
 if (!std::isfinite(best.score) ||
 best.overlap < config.minimum_overlap) {
 result.rejection_reason = "map overlap below acceptance gate";
 return;
 }
 // Dense loop refinement searches around the coarse endpoint winner, but
 // its motion prior remains the graph prediction. Endpoint rasterization
 // can move the coarse centre within one occupied band; expected ranges may
 // refine away from the graph only when they improve likelihood by the
 // matcher-declared separated-mode margin. This is the same contract as the
 // one-shot ScanMatcher, implemented here because this lattice is sliced
 // across control ticks.
 const auto evidence_at = [&](double x, double y, double heading) {
 PoseScore evidence;
 if (config.expected_range_validation) {
 evidence = matcher.score_expected_ranges(
 x, y, heading, beams, map, roll_rad, pitch_rad);
 } else {
 evidence = batch->score(0, x, y, heading, map, z_m);
 }
 return evidence;
 };
 const bool shifted_from_prior =
 std::hypot(best.x - evidence_prior.position.x,
 best.y - evidence_prior.position.y) > 1e-9 ||
 std::abs(core::wrap_angle(
 best.heading - evidence_prior.heading_rad)) > 1e-9;
 if (config.require_prior_improvement && shifted_from_prior) {
 const PoseScore prior_evidence = evidence_at(
 evidence_prior.position.x, evidence_prior.position.y,
 evidence_prior.heading_rad);
 if (prior_evidence.has_evidence &&
 best.score - prior_evidence.score < config.minimum_score_margin) {
 if (prior_evidence.overlap < config.minimum_overlap) {
 result.score = best.score;
 result.overlap = best.overlap;
 result.score_margin = best.score - prior_evidence.score;
 result.rejection_reason =
 "map match does not improve motion prior";
 return;
 }
 result.accepted = true;
 result.x_m = evidence_prior.position.x;
 result.y_m = evidence_prior.position.y;
 result.yaw_rad = evidence_prior.heading_rad;
 result.score = prior_evidence.score;
 result.overlap = prior_evidence.overlap;
 result.score_margin = std::isfinite(second_score)
 ? best.score - second_score : 1.0;
 result.position_sigma_m = std::max(0.02, map.resolution_m * 0.75);
 result.heading_sigma_rad = std::max(
 0.01, config.heading_step_rad * 1.5);
 result.rejection_reason = "accepted at motion prior";
 return;
 }
 }
 if (std::isfinite(second_score) &&
 result.score_margin < config.minimum_score_margin) {
 result.rejection_reason = "map match is geometrically ambiguous";
 return;
 }
 result.accepted = true;
 result.x_m = best.x;
 result.y_m = best.y;
 result.yaw_rad = best.heading;
 result.position_sigma_m = std::max(0.02, map.resolution_m * 0.75);
 result.heading_sigma_rad = std::max(
 0.01, config.heading_step_rad * 1.5);
 result.rejection_reason = "accepted";
 }

 ScanMatcherConfig config;
 ScanMatcher matcher;
 iface::MapSnapshot map;
 std::vector<ScanBeam> beams;
 std::unique_ptr<ScanMatcher::Batch> batch;
 std::vector<Hypothesis> hypotheses;
 ScanMatchResult result;
 double prior_x = 0.0;
 double prior_y = 0.0;
 double prior_heading = 0.0;
 Pose2 evidence_prior;
 double z_m = 0.0;
 double roll_rad = 0.0;
 double pitch_rad = 0.0;
 int translation_steps = 0;
 int heading_steps = 0;
 int ix = 0;
 int iy = 0;
 int ih = 0;
 bool complete = true;
 };

 struct Candidate {
 std::uint64_t id = 0;
 double distance = 0.0;
 Pose2 pose;
 };
 struct VerifiedCandidate {
 std::uint64_t old_id = 0;
 ScanMatchResult forward;
 ScanMatchResult reverse;
 double rank = -std::numeric_limits<double>::infinity();
 };
 enum class Phase { kPrepare, kCoarse, kForward, kReverse, kSelect };

 EstimatedState match_est;
 iface::ObservationFrame match_obs;
 std::uint64_t to_id = 0;
 std::vector<Candidate> candidates;
 std::size_t candidate_index = 0;
 std::uint64_t old_id = 0;
 Phase phase = Phase::kPrepare;
 Lattice lattice;
 ScanMatchResult coarse;
 ScanMatchResult forward;
 ScanMatchResult reverse;
 std::vector<VerifiedCandidate> verified;
 std::string last_rejection = "no candidate reached the matcher";
};

SlamSession::SlamSession(SlamConfig config) : config_(config) {}

SlamSession::~SlamSession() = default;

void SlamSession::reset() {
 loop_search_.reset();
  graph_.clear();
  keyframes_.clear();
  next_id_ = 1;
  loop_closures_ = 0;
 closed_historical_keyframes_.clear();
  last_attempt_s_ = -1.0e9;
  keyframes_at_last_attempt_ = 0;
  last_result_ = SlamLoopResult{};
  track_corrections_ = 0;
 fused_track_anchor_id_ = 0;
  last_track_s_ = -1.0e9;
  have_tracked_ = false;
  last_track_ = SlamTrackResult{};
  track_submap_ = iface::MapSnapshot{};
  track_submap_id_ = 0;
 pending_track_correction_valid_ = false;
 pending_track_correction_anchor_id_ = 0;
 pending_track_correction_ = Pose2{};
 pending_track_correction_observer_pose_ = Pose2{};
 pending_loop_valid_ = false;
 pending_loop_anchor_id_ = 0;
 pending_loop_transform_ = Pose2{};
 pending_loop_observer_pose_ = Pose2{};
}

bool SlamSession::keyframe_due(const EstimatedState& est) const {
  if (keyframes_.empty()) return true;
  const Pose2& last = keyframes_.back().pose;
  const double dist =
      std::hypot(est.x_m - last.position.x, est.y_m - last.position.y);
  const double dh =
      std::abs(core::wrap_angle(est.yaw_rad - last.heading_rad));
  return dist >= config_.keyframe_translation_m ||
         dh >= config_.keyframe_heading_rad;
}

void SlamSession::ingest(const EstimatedState& est,
                         const iface::ObservationFrame& obs) {
  if (!est.valid || !obs.scan_valid || obs.ranges_m.empty()) return;
  if (!keyframe_due(est)) return;
  SlamKeyframe kf;
  kf.id = next_id_++;
  kf.stamp_s = obs.stamp_ns * 1e-9;
  kf.pose.position.x = est.x_m;
  kf.pose.position.y = est.y_m;
  kf.pose.heading_rad = est.yaw_rad;
  kf.roll_rad = est.roll_rad;
  kf.pitch_rad = est.pitch_rad;
  kf.z_m = est.z_m;
  kf.ranges_m = obs.ranges_m;
  kf.static_ranges_m = obs.static_ranges_m;
  kf.angle_min_rad = obs.scan_angle_min_rad;
  kf.angle_increment_rad = normalized_scan_increment(
 obs.scan_angle_increment_rad, obs.ranges_m.size());

  PoseGraphKeyframe node;
  node.id = kf.id;
  node.stamp_s = kf.stamp_s;
  node.initial_pose = kf.pose;
  node.fixed = keyframes_.empty();  // anchor the gauge freedom at the start
  std::string reason;
  if (!graph_.add_keyframe(node, &reason)) return;
  if (!keyframes_.empty()) {
    const SlamKeyframe& prev = keyframes_.back();
    PoseGraphConstraint odom;
    odom.id = kf.id;
    odom.from = prev.id;
    odom.to = kf.id;
    odom.measured_relative_pose = relative_pose(prev.pose, kf.pose);
    odom.information = pose_graph_information(
        std::max(0.02, est.position_sigma_m),
        std::max(0.005, est.heading_sigma_rad));
    odom.kind = PoseGraphConstraintKind::Odometry;
    graph_.add_constraint(odom, nullptr);  // orphan node on failure: rare,
                                           // optimizer skips disconnected parts
  }
  keyframes_.push_back(std::move(kf));
  // Ring: drop the oldest node with its constraints once over budget. The
  // backend has no removal, so rebuild the retained tail (bounded, rare).
  if (keyframes_.size() > config_.max_keyframes && config_.max_keyframes > 1) {
    keyframes_.erase(keyframes_.begin());
    rebuild_graph();
  }
}

void SlamSession::rebuild_graph() {
  graph_.clear();
  for (const auto& k : keyframes_) {
    PoseGraphKeyframe n;
    n.id = k.id;
    n.stamp_s = k.stamp_s;
    n.initial_pose = k.pose;
    n.fixed = (k.id == keyframes_.front().id);
    graph_.add_keyframe(n, nullptr);
  }
  for (std::size_t i = 1; i < keyframes_.size(); ++i) {
    PoseGraphConstraint c;
    c.id = keyframes_[i].id;
    c.from = keyframes_[i - 1].id;
    c.to = keyframes_[i].id;
    c.measured_relative_pose =
        relative_pose(keyframes_[i - 1].pose, keyframes_[i].pose);
    c.information = pose_graph_information(0.05, 0.02);
    c.kind = PoseGraphConstraintKind::Odometry;
    graph_.add_constraint(c, nullptr);
  }
}

iface::MapSnapshot SlamSession::submap_for(
 const SlamKeyframe& kf, bool pad_endpoints) const {
  iface::MapSnapshot sub;
 // Dynamic returns are evidence for tracking, not static-map geometry.
 // With a static reference, retain only returns consistent with it;
 // absent a reference, preserve the legacy raw-scan behavior.
 auto static_range = [&](std::size_t i) -> float {
 const float r = kf.ranges_m[i];
 if (!(r > 0.0f) || !std::isfinite(r)) return 0.0f;
 if (kf.static_ranges_m.size() != kf.ranges_m.size()) return r;
 const float sr = kf.static_ranges_m[i];
 if (!(sr > 0.0f) || !std::isfinite(sr) ||
 std::abs(r - sr) > 0.30f) {
 return 0.0f;
 }
 return r;
 };
  sub.map_id = "slam-submap";
  sub.revision = kf.id;
  sub.frame_id = "map";
  // Extent: static evidence range plus margin, centered on the keyframe.
 float max_range = 0.0f;
 for (std::size_t i = 0; i < kf.ranges_m.size(); ++i) {
 const float r = static_range(i);
 if (r > 0.0f) max_range = std::max(max_range, r);
 }
 max_range = std::min(max_range, 10.0f) + 1.0f;
  sub.resolution_m = 0.1;
  sub.width = static_cast<std::uint32_t>(std::ceil(2 * max_range / sub.resolution_m));
  sub.height = sub.width;
  sub.origin_x_m = kf.pose.position.x - max_range;
  sub.origin_y_m = kf.pose.position.y - max_range;
  sub.occupancy.assign(sub.width * sub.height, -1);
  // 3D sensor pose for this keyframe: position + slope attitude composed
  // with the TF mount. Endpoints project to x/y (the submap is 2D).
  core::RigidTransform kf_from_base;
  kf_from_base.translation_m = {kf.pose.position.x, kf.pose.position.y,
                                kf.z_m};
  kf_from_base.rotation = core::quaternion_from_rpy(
      kf.roll_rad, kf.pitch_rad, kf.pose.heading_rad);
  const core::RigidTransform kf_from_sensor =
      core::compose(kf_from_base, base_from_lidar_);
  const double ksx = kf_from_sensor.translation_m.x;
  const double ksy = kf_from_sensor.translation_m.y;
  const std::size_t n = kf.ranges_m.size();
  // Two passes: ray interiors first, endpoints second. A single pass would
  // let one beam's free interior overwrite another beam's occupied endpoint
  // where rays cross near walls.
  for (std::size_t i = 0; i < n; ++i) {
    const float range = static_range(i);
    if (!(range > 0.0f) || !std::isfinite(range)) continue;
    const double angle = kf.angle_min_rad + kf.angle_increment_rad * i;
    const core::Vec3 dir = core::rotate_vector(
        kf_from_sensor.rotation, {std::cos(angle), std::sin(angle), 0.0});
    const double az = std::atan2(dir.y, dir.x);
    const double dx = std::cos(az), dy = std::sin(az);
    const double hit = std::min<double>(range, 10.0);
    // Ray interior: free (march at half-cell steps, endpoint excluded).
    const int steps = std::max(
        1, static_cast<int>(std::ceil(hit / (0.5 * sub.resolution_m))));
    for (int s = 0; s < steps; ++s) {
      const double b = static_cast<double>(s) / steps;
      const double wx = ksx + hit * dx * b;
      const double wy = ksy + hit * dy * b;
      const int gx = static_cast<int>(
          std::floor((wx - sub.origin_x_m) / sub.resolution_m));
      const int gy = static_cast<int>(
          std::floor((wy - sub.origin_y_m) / sub.resolution_m));
      if (gx >= 0 && gy >= 0 && gx < static_cast<int>(sub.width) &&
          gy < static_cast<int>(sub.height) &&
          sub.occupancy[gy * sub.width + gx] != 100) {
        sub.occupancy[gy * sub.width + gx] = 0;
      }
    }
  }
  // Endpoints splat 3x3: a scan endpoint is a quantized sample of a
  // continuous surface, and the matcher evaluates hypotheses on a 0.15 m
  // grid against 0.1 m cells, so a 1-cell shell would alias misses from
  // sub-cell jitter. The splat models endpoint quantization uncertainty;
  // walls stay thin (one cell + ring), free space stays free.
  for (std::size_t i = 0; i < n; ++i) {
    const float range = static_range(i);
    if (!(range > 0.0f) || !std::isfinite(range) || range >= 10.0f) continue;
    const double angle = kf.angle_min_rad + kf.angle_increment_rad * i;
    const core::Vec3 dir = core::rotate_vector(
        kf_from_sensor.rotation, {std::cos(angle), std::sin(angle), 0.0});
    const double wx = ksx + range * dir.x;
    const double wy = ksy + range * dir.y;
    const int cx = static_cast<int>(
        std::floor((wx - sub.origin_x_m) / sub.resolution_m));
    const int cy = static_cast<int>(
        std::floor((wy - sub.origin_y_m) / sub.resolution_m));
 const int endpoint_padding = pad_endpoints ? 1 : 0;
 for (int ay = -endpoint_padding; ay <= endpoint_padding; ++ay) {
 for (int ax = -endpoint_padding; ax <= endpoint_padding; ++ax) {
        const int gx = cx + ax, gy = cy + ay;
        if (gx >= 0 && gy >= 0 && gx < static_cast<int>(sub.width) &&
            gy < static_cast<int>(sub.height)) {
          sub.occupancy[gy * sub.width + gx] = 100;
        }
      }
    }
  }
  return sub;
}

iface::MapSnapshot SlamSession::merge_submap_window(
 std::size_t begin, std::size_t end, const char* map_id,
 std::uint64_t revision, bool pad_endpoints) const {
 if (keyframes_.empty() || begin > end || end >= keyframes_.size()) {
 return {};
 }
 if (begin == end) return submap_for(keyframes_[begin], pad_endpoints);
 std::vector<iface::MapSnapshot> scans;
 scans.reserve(end - begin + 1);
 double min_x = std::numeric_limits<double>::infinity();
 double min_y = std::numeric_limits<double>::infinity();
 double max_x = -std::numeric_limits<double>::infinity();
 double max_y = -std::numeric_limits<double>::infinity();
 for (std::size_t i = begin; i <= end; ++i) {
 iface::MapSnapshot scan = submap_for(keyframes_[i], pad_endpoints);
 if (!scan.has_raster()) continue;
 min_x = std::min(min_x, scan.origin_x_m);
 min_y = std::min(min_y, scan.origin_y_m);
 max_x = std::max(max_x,
 scan.origin_x_m + scan.width * scan.resolution_m);
 max_y = std::max(max_y,
 scan.origin_y_m + scan.height * scan.resolution_m);
 scans.push_back(std::move(scan));
 }
 if (scans.empty()) return {};

 iface::MapSnapshot merged;
 merged.map_id = map_id;
 merged.revision = revision;
 merged.frame_id = "map";
 merged.resolution_m = 0.1;
 merged.origin_x_m = min_x;
 merged.origin_y_m = min_y;
 merged.width = static_cast<std::uint32_t>(std::ceil(
 (max_x - min_x) / merged.resolution_m));
 merged.height = static_cast<std::uint32_t>(std::ceil(
 (max_y - min_y) / merged.resolution_m));
 if (merged.width == 0 || merged.height == 0) return {};
 merged.occupancy.assign(
 static_cast<std::size_t>(merged.width) * merged.height, -1);

 // Occupied evidence wins over free evidence across the posed scans. A
 // later ray must never erase an earlier surface merely because the two
 // quantized beams cross in a neighbouring cell.
 for (const auto& scan : scans) {
 for (std::uint32_t sy = 0; sy < scan.height; ++sy) {
 for (std::uint32_t sx = 0; sx < scan.width; ++sx) {
 const int value = scan.occupancy[
 static_cast<std::size_t>(sy) * scan.width + sx];
 if (value < 0) continue;
 const double wx = scan.origin_x_m +
 (static_cast<double>(sx) + 0.5) * scan.resolution_m;
 const double wy = scan.origin_y_m +
 (static_cast<double>(sy) + 0.5) * scan.resolution_m;
 const int dx = static_cast<int>(std::floor(
 (wx - merged.origin_x_m) / merged.resolution_m));
 const int dy = static_cast<int>(std::floor(
 (wy - merged.origin_y_m) / merged.resolution_m));
 if (dx < 0 || dy < 0 || dx >= static_cast<int>(merged.width) ||
 dy >= static_cast<int>(merged.height)) {
 continue;
 }
 int8_t& target = merged.occupancy[
 static_cast<std::size_t>(dy) * merged.width + dx];
 if (value >= 50) target = 100;
 else if (target < 0) target = 0;
 }
 }
 }
 return merged;
}

iface::MapSnapshot SlamSession::submap_ending_at(
 std::size_t index, bool pad_endpoints) const {
 if (keyframes_.empty() || index >= keyframes_.size()) return {};
 const std::size_t count = std::max<std::size_t>(
 1, config_.local_submap_keyframes);
 const std::size_t begin = index + 1 > count ? index + 1 - count : 0;
 return merge_submap_window(
 begin, index, "slam-local-submap", keyframes_[index].id,
 pad_endpoints);
}

iface::MapSnapshot SlamSession::historical_submap_around(
 std::size_t index, bool pad_endpoints) const {
 if (keyframes_.empty() || index >= keyframes_.size()) return {};
 const std::size_t count = std::max<std::size_t>(
 1, config_.local_submap_keyframes);
 const std::size_t half = count / 2;
 std::size_t begin = index > half ? index - half : 0;
 const std::size_t end = std::min(
 keyframes_.size() - 1, begin + count - 1);
 begin = end + 1 > count ? end + 1 - count : 0;
 return merge_submap_window(
 begin, end, "slam-historical-submap", keyframes_[index].id,
 pad_endpoints);
}

iface::MapSnapshot SlamSession::submap(std::uint64_t keyframe_id) const {
 for (const auto& kf : keyframes_) {
 if (kf.id == keyframe_id) return submap_for(kf);
 }
 return {};
}

SlamTrackResult SlamSession::track(const EstimatedState& est,
 const iface::ObservationFrame& obs,
 bool require_temporally_confirmed_correction) {
  SlamTrackResult out;
  if (!est.valid || !obs.scan_valid || obs.ranges_m.empty() ||
      keyframes_.empty()) {
    out.diagnostic = "tracking not due (invalid or no keyframes)";
    last_track_ = out;
    return out;
  }
  const double now_s = obs.stamp_ns * 1e-9;
  const bool interval_due =
      !have_tracked_ || now_s - last_track_s_ >= config_.track_interval_s;
  const double moved_m = have_tracked_
                             ? std::hypot(est.x_m - last_track_x_m_,
                                          est.y_m - last_track_y_m_)
                             : std::numeric_limits<double>::infinity();
  const double turned_rad =
      have_tracked_ ? std::abs(core::wrap_angle(est.yaw_rad -
                                                last_track_yaw_rad_))
                    : std::numeric_limits<double>::infinity();
  if (!interval_due ||
      (moved_m < config_.track_min_translation_m &&
       turned_rad < config_.track_min_heading_rad)) {
    out.diagnostic = "track rate limited";
    last_track_ = out;
    return out;
  }
  out.attempted = true;
  last_track_s_ = now_s;
  last_track_x_m_ = est.x_m;
  last_track_y_m_ = est.y_m;
  last_track_yaw_rad_ = est.yaw_rad;
  have_tracked_ = true;
  // Submap of the newest keyframe (cached per keyframe id): endpoints
  // observed occupied, ray interiors free — the survey's unknown cells
  // never appear here, so free-space evidence discriminates.
  const SlamKeyframe& anchor = keyframes_.back();
 if (fused_track_anchor_id_ == anchor.id) {
 out.diagnostic = "track rate limited by fused anchor";
 last_track_ = out;
 return out;
 }
  if (track_submap_id_ != anchor.id) {
 track_submap_ = submap_ending_at(keyframes_.size() - 1);
    track_submap_id_ = anchor.id;
  }
  ScanMatcherConfig match_cfg;
 match_cfg.expected_range_validation = false;
 match_cfg.require_prior_improvement = true;
  match_cfg.search_translation_m = config_.track_window_m;
  match_cfg.search_heading_rad = config_.track_heading_rad;
  match_cfg.translation_step_m = config_.track_step_m;
  match_cfg.heading_step_rad = config_.track_heading_step_rad;
  match_cfg.minimum_overlap = config_.track_min_overlap;
  match_cfg.minimum_score_margin = config_.track_min_margin;
  match_cfg.minimum_returns = 4;
  match_cfg.maximum_returns = config_.track_max_beams;
 ScanMatcher matcher(match_cfg);
 matcher.set_lidar_mount(base_from_lidar_);
  ScanMatchResult match = matcher.localize(
      obs.ranges_m, obs.scan_angle_min_rad, obs.scan_angle_increment_rad,
      est.x_m, est.y_m, est.yaw_rad, track_submap_, obs.stamp_ns,
      est.roll_rad, est.pitch_rad, est.z_m);
  if (!match.accepted) {
    out.diagnostic = std::string("track rejected: ") + match.rejection_reason;
    last_track_ = out;
    return out;
  }
  // Reverse verification (same two-sided check loop closures use): the old
  // keyframe scan must also match a submap rendered from the CURRENT scan
  // at the FORWARD solution. A one-sided alignment against unrelated
  // geometry (viewpoint change, repetitive boxes) must not become a
  // correction. Rendered at the forward pose, not the drifted estimate,
  // so the check cannot be offset by the very drift being measured.
  SlamKeyframe now_kf;
  now_kf.id = 0;
  now_kf.stamp_s = obs.stamp_ns * 1e-9;
  now_kf.pose.position.x = match.x_m;
  now_kf.pose.position.y = match.y_m;
  now_kf.pose.heading_rad = match.yaw_rad;
  now_kf.roll_rad = est.roll_rad;
  now_kf.pitch_rad = est.pitch_rad;
  now_kf.z_m = est.z_m;
  now_kf.ranges_m = obs.ranges_m;
  now_kf.angle_min_rad = obs.scan_angle_min_rad;
  now_kf.angle_increment_rad = normalized_scan_increment(
 obs.scan_angle_increment_rad, obs.ranges_m.size());
  const iface::MapSnapshot rsub = submap_for(now_kf);
  const SlamKeyframe& old = keyframes_.back();
 // The reverse centre is a geometric verification hypothesis, not a fresh
 // odometry prior. Requiring an improvement over it would reject a correct
 // no-op reverse alignment after the forward pass has already removed the
 // drift. Keep the same geometry gates but disable only the prior semantic.
 ScanMatcherConfig reverse_cfg = match_cfg;
 reverse_cfg.require_prior_improvement = false;
 ScanMatcher reverse_matcher(reverse_cfg);
 reverse_matcher.set_lidar_mount(base_from_lidar_);
 ScanMatchResult rev = reverse_matcher.localize(
      old.ranges_m, old.angle_min_rad, old.angle_increment_rad,
      old.pose.position.x, old.pose.position.y, old.pose.heading_rad, rsub,
      obs.stamp_ns, old.roll_rad, old.pitch_rad, old.z_m);
 if (!rev.accepted || rev.overlap < config_.track_min_overlap ||
 rev.score < config_.track_min_score ||
 rev.score_margin < config_.track_min_margin) {
    out.diagnostic = "track rejected: reverse verification failed";
    last_track_ = out;
    return out;
  }
 const double correction_m = std::hypot(
 match.x_m - est.x_m, match.y_m - est.y_m);
 const double correction_rad = std::abs(
 core::wrap_angle(match.yaw_rad - est.yaw_rad));
 out.score = match.score;
 out.overlap = std::min(match.overlap, rev.overlap);
 out.score_margin = match.score_margin;
 out.correction_m = correction_m;
 out.correction_rad = correction_rad;
 if (!std::isfinite(correction_m) || !std::isfinite(correction_rad) ||
 correction_m > config_.track_max_correction_m ||
 correction_rad > config_.track_max_correction_rad ||
 match.score < config_.track_min_score ||
 match.score_margin < config_.track_min_margin) {
 out.diagnostic = "track rejected: weak or oversized correction";
 last_track_ = out;
 return out;
 }
 // Wheel odometry is a prior, not a veto. During longitudinal slip the
 // geometrically correct scan update can oppose the encoder direction, but a
 // single such view is unsafe when no surveyed frame exists. The same is true
 // of a lateral or angular jump: actor returns and locally aliased geometry
 // do not become trustworthy merely because they are orthogonal to travel.
 // Mapless tracking therefore confirms every substantial rigid correction
 // across independent views of the same causal submap. Matcher lattice steps
 // define "substantial" and transform agreement; the existing keyframe
 // motion contract defines view independence.
 const double speed_for_independent_view_mps = config_.track_interval_s > 0.0
 ? config_.track_min_translation_m / config_.track_interval_s
 : std::numeric_limits<double>::infinity();
 const double correction_x = match.x_m - est.x_m;
 const double correction_y = match.y_m - est.y_m;
 const double along_m = correction_x * std::cos(est.yaw_rad) +
 correction_y * std::sin(est.yaw_rad);
 const bool motion_direction_observable = obs.odometry_valid &&
 std::isfinite(obs.forward_mps) &&
 std::abs(obs.forward_mps) >= speed_for_independent_view_mps;
 const double odom_sign = obs.forward_mps >= 0.0 ? 1.0 : -1.0;
 const bool opposing = motion_direction_observable &&
 along_m * odom_sign < -std::max(0.0, config_.track_step_m);
 if (opposing && !require_temporally_confirmed_correction) {
 pending_track_correction_valid_ = false;
 out.diagnostic = "track rejected: correction opposes odometry prior";
 last_track_ = out;
 return out;
 }
 const Pose2 observer{{est.x_m, est.y_m}, est.yaw_rad};
 const Pose2 correction = compose_pose(
 Pose2{{match.x_m, match.y_m}, match.yaw_rad}, inverse_pose(observer));
 const bool substantial_correction =
 correction_m > std::max(0.0, config_.track_step_m) ||
 correction_rad > std::max(0.0, config_.track_heading_step_rad);
 if (require_temporally_confirmed_correction && substantial_correction) {
 const double observed_motion_m = pending_track_correction_valid_
 ? std::hypot(observer.position.x -
 pending_track_correction_observer_pose_.position.x,
 observer.position.y -
 pending_track_correction_observer_pose_.position.y)
 : 0.0;
 const double observed_motion_rad = pending_track_correction_valid_
 ? std::abs(core::wrap_angle(observer.heading_rad -
 pending_track_correction_observer_pose_.heading_rad))
 : 0.0;
 const bool independent_view =
 observed_motion_m >= config_.track_min_translation_m ||
 observed_motion_rad >= config_.track_min_heading_rad;
 const bool consistent_transform = pending_track_correction_valid_ &&
 pending_track_correction_anchor_id_ == anchor.id &&
 pose_translation_error(pending_track_correction_, correction) <=
 std::max(0.0, config_.track_step_m) &&
 std::abs(core::wrap_angle(
 pending_track_correction_.heading_rad - correction.heading_rad)) <=
 std::max(0.0, config_.track_heading_step_rad);
 // The cached anchor is the bounded lifetime: once a new keyframe becomes
 // the tracking reference, an old foreground hypothesis cannot confirm a
 // correction in unrelated geometry. This avoids a separately tuned timeout
 // and tolerates skipped scan-search ticks under CPU load.
 if (!consistent_transform || !independent_view) {
 pending_track_correction_valid_ = true;
 pending_track_correction_anchor_id_ = anchor.id;
 pending_track_correction_ = correction;
 pending_track_correction_observer_pose_ = observer;
 out.diagnostic = opposing
 ? "track pending: opposing odometry needs another view"
 : "track pending: mapless correction needs another view";
 last_track_ = out;
 return out;
 }
 pending_track_correction_valid_ = false;
 } else {
 pending_track_correction_valid_ = false;
 }
  out.accepted = true;
  out.x_m = match.x_m;
  out.y_m = match.y_m;
  out.yaw_rad = match.yaw_rad;
  out.position_sigma_m = match.position_sigma_m;
  out.heading_sigma_rad = match.heading_sigma_rad;
 out.pose_information = match.pose_information;
 out.pose_information_valid = match.pose_information_valid;
  out.diagnostic = "track corrected";
 fused_track_anchor_id_ = anchor.id;
  ++track_corrections_;
  last_track_ = out;
  return out;
}

SlamLoopResult SlamSession::try_close_loops(
 const EstimatedState& est, const iface::ObservationFrame& obs,
 const iface::MapSnapshot* absolute_map) {
  SlamLoopResult out;
  if (!est.valid || !obs.scan_valid || obs.ranges_m.empty() ||
      keyframes_.size() < 3) {
    out.diagnostic = "insufficient session";
    last_result_ = out;
    return out;
  }
  const double now_s = obs.stamp_ns * 1e-9;
 // A pending mapless proposal is evidence tied to one historical anchor,
 // not a wall-clock request.  Keep it until that anchor is re-observed (or
 // the bounded keyframe ring drops it); expiring it by elapsed time can
 // discard the first view while the sliced matcher is still working.
 if (pending_loop_valid_) {
 const bool anchor_retained = std::any_of(
 keyframes_.begin(), keyframes_.end(), [&](const SlamKeyframe& keyframe) {
 return keyframe.id == pending_loop_anchor_id_;
 });
 if (!anchor_retained) {
 pending_loop_valid_ = false;
 pending_loop_anchor_id_ = 0;
 }
 }
  ScanMatcherConfig match_cfg;
 // Coarse retrieval uses endpoint occupancy: expected-range marching on a
 // sparse historical submap can terminate in unknown space before it has
 // shortlisted the correct basin. The dense pass below applies range
 // consistency to the bounded survivors.
 match_cfg.expected_range_validation = false;
  match_cfg.search_translation_m = config_.match_search_translation_m;
  match_cfg.translation_step_m = config_.match_translation_step_m;
  match_cfg.search_heading_rad = config_.match_search_heading_rad;
  match_cfg.heading_step_rad = config_.match_heading_step_rad;
  match_cfg.maximum_returns = config_.match_max_beams;
  match_cfg.minimum_overlap = config_.match_minimum_overlap;
 match_cfg.minimum_score_margin = config_.gate.minimum_score_margin;
 // The coarse loop lattice deliberately has wider cells than local
 // tracking. Treat candidates inside one keyframe-spacing neighbourhood as
 // the same physical mode; otherwise the raster endpoint splat produces
 // several adjacent, nearly tied cells and the ambiguity gate mistakes a
 // single broad optimum for two different places.
 match_cfg.ambiguity_distance_m = std::max(
 config_.keyframe_translation_m,
 2.0 * config_.match_translation_step_m);
 match_cfg.ambiguity_heading_rad = std::max(
 config_.keyframe_heading_rad,
 2.0 * config_.match_heading_step_rad);
 ScanMatcherConfig refine_cfg;
 // Dense refinement remains an association step inside the coarse basin.
 // The disjoint historical/recent windows below provide the independent
 // expected-range evidence used by the final loop gate; applying it again
 // to a sparse single-keyframe refinement can reject a valid revisit before
 // the causal evidence is available.
 refine_cfg.expected_range_validation = false;
 refine_cfg.search_translation_m = config_.match_refine_translation_m;
 refine_cfg.translation_step_m = config_.match_refine_translation_step_m;
 refine_cfg.search_heading_rad = config_.match_refine_heading_rad;
 refine_cfg.heading_step_rad = config_.match_refine_heading_step_rad;
 refine_cfg.maximum_returns = config_.match_refine_max_beams;
 refine_cfg.minimum_overlap = config_.match_minimum_overlap;
 refine_cfg.minimum_score_margin = config_.gate.minimum_score_margin;
 // The dense pass resolves one mode selected by the wide pass. Adjacent
 // raster cells belong to that mode and must not manufacture ambiguity.
 refine_cfg.ambiguity_distance_m = std::max(
 config_.keyframe_translation_m,
 2.0 * config_.match_refine_translation_step_m);
 refine_cfg.ambiguity_heading_rad = std::max(
 config_.keyframe_heading_rad,
 2.0 * config_.match_refine_heading_step_rad);
 if (!loop_search_) {
 if (now_s - last_attempt_s_ < config_.loop_attempt_interval_s ||
 keyframes_.size() == keyframes_at_last_attempt_) {
 out.diagnostic = "attempt rate limited";
 last_result_ = out;
 return out;
 }
 keyframes_at_last_attempt_ = keyframes_.size();
 loop_search_ = std::make_unique<LoopSearchWork>();
 loop_search_->match_est = est;
 loop_search_->match_obs = obs;
 loop_search_->to_id = keyframes_.back().id;
 std::vector<double> path_length_to_latest(keyframes_.size(), 0.0);
 for (std::size_t i = keyframes_.size() - 1; i > 0; --i) {
 path_length_to_latest[i - 1] = path_length_to_latest[i] +
 pose_translation_error(keyframes_[i - 1].pose, keyframes_[i].pose);
 }
 // Candidates: temporally separated and spatially near the scan pose.
 // Store stable ids because keyframe ingestion continues while matching is
 // sliced across ticks.
 for (std::size_t i = 0; i + 1 < keyframes_.size(); ++i) {
 const SlamKeyframe& kf = keyframes_[i];
 if (std::find(closed_historical_keyframes_.begin(),
 closed_historical_keyframes_.end(), kf.id) !=
 closed_historical_keyframes_.end()) {
 continue;
 }
 const std::size_t separation = keyframes_.size() - 1 - i;
 // The two causal submaps must not share keyframes. Shared scans make the
 // forward/reverse checks correlated and can turn an ordinary local chain
 // constraint into a false "loop". Two complete windows give independent
 // evidence without introducing a scenario-distance threshold.
 const std::size_t independent_submap_separation = std::max(
 config_.minimum_loop_keyframe_separation,
 2 * std::max<std::size_t>(1, config_.local_submap_keyframes));
 // Heading-only keyframes at a dock do not create nonlocal topology. The
 // chain between two disjoint submap windows must also contain the
 // translation those windows nominally cover; derive it from the existing
 // keyframe spacing instead of adding a scenario distance threshold.
 const double independent_path_length_m =
 static_cast<double>(independent_submap_separation) *
 config_.keyframe_translation_m;
 if (separation < independent_submap_separation ||
 path_length_to_latest[i] < independent_path_length_m ||
 now_s - kf.stamp_s < config_.gate.minimum_temporal_separation_s) {
 continue;
 }
 const double distance = std::hypot(
 est.x_m - kf.pose.position.x, est.y_m - kf.pose.position.y);
 // Candidate retrieval and pose refinement must share one correction
 // envelope. A submap can extend well beyond its anchor; retrieving anchors
 // farther away than the matcher can correct lets adjacent, repetitive
 // corners masquerade as revisits even though no odometric loop exists.
 const double candidate_radius_m = std::min(
 config_.gate.maximum_candidate_distance_m,
 config_.match_search_translation_m);
 if (distance <= candidate_radius_m) {
 loop_search_->candidates.push_back({kf.id, distance, kf.pose});
 }
 }
 std::sort(loop_search_->candidates.begin(),
 loop_search_->candidates.end(),
 [](const LoopSearchWork::Candidate& a,
 const LoopSearchWork::Candidate& b) {
 if (a.distance != b.distance) return a.distance < b.distance;
 return a.id < b.id;
 });
 // Spend the bounded matcher budget on distinct historical hypotheses,
 // rather than letting several adjacent keyframes from one place consume
 // every slot. The matcher already defines the spatial/angular resolution
 // at which hypotheses are the same mode, so candidate suppression adds no
 // second place-recognition threshold. Preserve a pending anchor first so
 // temporal confirmation can verify the exact same causal submap.
 const std::vector<LoopSearchWork::Candidate> retrieved =
 std::move(loop_search_->candidates);
 const auto same_historical_mode = [&](
 const LoopSearchWork::Candidate& a,
 const LoopSearchWork::Candidate& b) {
 return std::hypot(a.pose.position.x - b.pose.position.x,
 a.pose.position.y - b.pose.position.y) <=
 match_cfg.ambiguity_distance_m &&
 std::abs(core::wrap_angle(a.pose.heading_rad - b.pose.heading_rad)) <=
 match_cfg.ambiguity_heading_rad;
 };
 const auto append_if_distinct = [&](
 const LoopSearchWork::Candidate& candidate) {
 if (loop_search_->candidates.size() >=
 config_.max_candidates_per_attempt) {
 return;
 }
 const bool duplicate = std::any_of(
 loop_search_->candidates.begin(), loop_search_->candidates.end(),
 [&](const LoopSearchWork::Candidate& selected) {
 return same_historical_mode(candidate, selected);
 });
 if (!duplicate) loop_search_->candidates.push_back(candidate);
 };
 if (pending_loop_valid_) {
 const auto pending = std::find_if(
 retrieved.begin(), retrieved.end(),
 [&](const LoopSearchWork::Candidate& candidate) {
 return candidate.id == pending_loop_anchor_id_;
 });
 if (pending != retrieved.end()) append_if_distinct(*pending);
 }
 for (const auto& candidate : retrieved) {
 append_if_distinct(candidate);
 }
 if (loop_search_->candidates.empty()) {
 loop_search_.reset();
 out.diagnostic = "no eligible candidates";
 last_result_ = out;
 return out;
 }
 // Candidate retrieval is cheap and runs once per new keyframe. An empty
 // retrieval must not consume the expensive-search cooldown: doing so can
 // blind the frontend for the entire short approach into a real revisit.
 // Start the cadence only when scan matching work actually begins.
 last_attempt_s_ = now_s;
 }

 auto find_keyframe = [&](std::uint64_t id) -> SlamKeyframe* {
 const auto found = std::find_if(
 keyframes_.begin(), keyframes_.end(),
 [id](const SlamKeyframe& kf) { return kf.id == id; });
 return found == keyframes_.end() ? nullptr : &*found;
 };
 auto defer = [&](const std::string& diagnostic) {
 out.diagnostic = diagnostic;
 last_result_ = out;
 return out;
 };
 auto reject_candidate = [&](const std::string& diagnostic) {
 LoopSearchWork& work = *loop_search_;
 work.last_rejection = diagnostic;
 ++work.candidate_index;
 work.phase = work.candidate_index >= work.candidates.size()
 ? LoopSearchWork::Phase::kSelect
 : LoopSearchWork::Phase::kPrepare;
 return defer("loop search pending: " + diagnostic);
 };
 auto reject_search = [&](const std::string& diagnostic) {
 loop_search_.reset();
 return defer(diagnostic);
 };

 LoopSearchWork& work = *loop_search_;
 if (work.phase == LoopSearchWork::Phase::kPrepare) {
 SlamKeyframe* old = find_keyframe(
 work.candidates[work.candidate_index].id);
 if (old == nullptr || find_keyframe(work.to_id) == nullptr) {
 loop_search_.reset();
 return defer("loop search invalidated by keyframe retention");
 }
 work.old_id = old->id;
 work.lattice.start(match_cfg, base_from_lidar_,
 work.match_obs.ranges_m, work.match_obs.scan_angle_min_rad,
 work.match_obs.scan_angle_increment_rad, work.match_est.x_m,
 work.match_est.y_m, work.match_est.yaw_rad,
 historical_submap_around(
 static_cast<std::size_t>(old - keyframes_.data())),
 work.match_est.roll_rad, work.match_est.pitch_rad,
 work.match_est.z_m);
 work.phase = LoopSearchWork::Phase::kCoarse;
 }
 if (work.phase == LoopSearchWork::Phase::kCoarse) {
 if (!work.lattice.advance(config_.match_work_budget)) {
 return defer("loop coarse search pending");
 }
 work.coarse = work.lattice.result;
 if (!work.coarse.accepted) {
 return reject_candidate(
 "coarse match rejected: " + work.coarse.rejection_reason);
 }
 SlamKeyframe* old = find_keyframe(work.old_id);
 if (old == nullptr) {
 loop_search_.reset();
 return defer("loop search invalidated by keyframe retention");
 }
 iface::MapSnapshot sub = historical_submap_around(
 static_cast<std::size_t>(old - keyframes_.data()), false);
 work.lattice.start(refine_cfg, base_from_lidar_,
 work.match_obs.ranges_m, work.match_obs.scan_angle_min_rad,
 work.match_obs.scan_angle_increment_rad, work.coarse.x_m,
 work.coarse.y_m, work.coarse.yaw_rad, std::move(sub),
 work.match_est.roll_rad, work.match_est.pitch_rad,
 work.match_est.z_m,
 Pose2{{work.match_est.x_m, work.match_est.y_m},
 work.match_est.yaw_rad});
 work.phase = LoopSearchWork::Phase::kForward;
 return defer("loop forward refinement pending");
 }
 if (work.phase == LoopSearchWork::Phase::kForward) {
 if (!work.lattice.advance(config_.match_work_budget)) {
 return defer("loop forward refinement pending");
 }
 work.forward = work.lattice.result;
 if (!work.forward.accepted) {
 return reject_candidate(
 "forward refinement rejected: " + work.forward.rejection_reason);
 }
 // The coarse pass separates physical association modes. Dense refinement
 // only estimates a pose inside that chosen mode, so retain the coarse
 // separated-mode margin for the loop gate. Cross-candidate comparison
 // below can still reduce it when another historical place competes.
 work.forward.score_margin = work.coarse.score_margin;
 SlamKeyframe* old = find_keyframe(work.old_id);
 SlamKeyframe* recent = find_keyframe(work.to_id);
 if (old == nullptr || recent == nullptr) {
 loop_search_.reset();
 return defer("loop search invalidated by keyframe retention");
 }
 const std::size_t recent_index =
 static_cast<std::size_t>(recent - keyframes_.data());
 const Pose2 current_pose{
 {work.match_est.x_m, work.match_est.y_m},
 work.match_est.yaw_rad};
 const Pose2 matched_current{
 {work.forward.x_m, work.forward.y_m},
 work.forward.yaw_rad};
 // Forward maps the recent odometry frame into the historical submap.
 // Reverse verification must test its inverse; an unrelated scan optimum
 // in the recent submap is not independent support for the same closure.
 const Pose2 forward_correction = compose_pose(
 matched_current, inverse_pose(current_pose));
 const Pose2 expected_old_in_recent = compose_pose(
 inverse_pose(forward_correction), old->pose);
 ScanMatcherConfig reverse_cfg = refine_cfg;
 reverse_cfg.expected_range_validation = false;
 reverse_cfg.minimum_overlap = 0.0;
 reverse_cfg.minimum_score_margin = 0.0;
 reverse_cfg.require_prior_improvement = false;
 work.lattice.start(reverse_cfg, base_from_lidar_, old->ranges_m,
 old->angle_min_rad, old->angle_increment_rad,
 expected_old_in_recent.position.x,
 expected_old_in_recent.position.y,
 expected_old_in_recent.heading_rad,
 submap_ending_at(recent_index, false),
 old->roll_rad, old->pitch_rad,
 old->z_m, expected_old_in_recent);
 work.phase = LoopSearchWork::Phase::kReverse;
 return defer("loop reverse verification pending");
 }
 if (work.phase == LoopSearchWork::Phase::kReverse &&
 !work.lattice.advance(config_.match_work_budget)) {
 return defer("loop reverse verification pending");
 }
 if (work.phase == LoopSearchWork::Phase::kReverse) {
 work.reverse = work.lattice.result;
 if (!work.reverse.accepted) {
 return reject_candidate(
 "reverse match rejected: " + work.reverse.rejection_reason);
 }
 // The disjoint submap window below supplies the reverse overlap gate.
 SlamKeyframe* old = find_keyframe(work.old_id);
 if (old == nullptr) {
 loop_search_.reset();
 return defer("loop search invalidated by keyframe retention");
 }
 const Pose2 current_pose{
 {work.match_est.x_m, work.match_est.y_m},
 work.match_est.yaw_rad};
 const Pose2 matched_current{
 {work.forward.x_m, work.forward.y_m},
 work.forward.yaw_rad};
 const Pose2 forward_correction = compose_pose(
 matched_current, inverse_pose(current_pose));
 const Pose2 matched_old{
 {work.reverse.x_m, work.reverse.y_m},
 work.reverse.yaw_rad};
 const Pose2 reverse_correction = compose_pose(
 matched_old, inverse_pose(old->pose));
 const Pose2 reverse_as_forward = inverse_pose(reverse_correction);

 // A single scan can slide inside a thick endpoint band in both directions.
 // Resolve that ambiguity at submap level: score the complete disjoint
 // recent and historical keyframe windows under each transform proposed by
 // forward matching, reverse matching, and the graph prediction. This is
 // still ROBOT-only evidence (stored scans and estimated keyframe poses).
 struct WindowEvidence {
 double score = -std::numeric_limits<double>::infinity();
 double overlap = 0.0;
 std::size_t returns = 0;
 bool valid = false;
 };
 const auto causal_bounds = [&](std::size_t end) {
 const std::size_t count = std::max<std::size_t>(
 1, config_.local_submap_keyframes);
 const std::size_t begin = end + 1 > count ? end + 1 - count : 0;
 return std::pair<std::size_t, std::size_t>{begin, end};
 };
 const auto centered_bounds = [&](std::size_t center) {
 const std::size_t count = std::max<std::size_t>(
 1, config_.local_submap_keyframes);
 const std::size_t half = count / 2;
 std::size_t begin = center > half ? center - half : 0;
 const std::size_t end = std::min(
 keyframes_.size() - 1, begin + count - 1);
 begin = end + 1 > count ? end + 1 - count : 0;
 return std::pair<std::size_t, std::size_t>{begin, end};
 };
 ScanMatcher window_matcher(refine_cfg);
 window_matcher.set_lidar_mount(base_from_lidar_);
 const auto score_window = [&] (
 std::pair<std::size_t, std::size_t> bounds,
 const iface::MapSnapshot& map, const Pose2& transform) {
 WindowEvidence aggregate;
 double weighted_score = 0.0;
 double weighted_overlap = 0.0;
 for (std::size_t index = bounds.first; index <= bounds.second; ++index) {
 const SlamKeyframe& keyframe = keyframes_[index];
 const auto beams = window_matcher.prepare_beams(
 keyframe.ranges_m, keyframe.angle_min_rad,
 keyframe.angle_increment_rad, config_.match_refine_max_beams);
 const Pose2 transformed = compose_pose(transform, keyframe.pose);
 const PoseScore evidence = window_matcher.score_expected_ranges(
 transformed.position.x, transformed.position.y,
 transformed.heading_rad, beams, map,
 keyframe.roll_rad, keyframe.pitch_rad);
 if (!evidence.has_evidence || evidence.usable_returns == 0) continue;
 const double weight = static_cast<double>(evidence.usable_returns);
 weighted_score += weight * evidence.score;
 weighted_overlap += weight * evidence.overlap;
 aggregate.returns += evidence.usable_returns;
 }
 if (aggregate.returns < refine_cfg.minimum_returns) return aggregate;
 const double normalization = static_cast<double>(aggregate.returns);
 aggregate.score = weighted_score / normalization;
 aggregate.overlap = weighted_overlap / normalization;
 aggregate.valid = std::isfinite(aggregate.score);
 return aggregate;
 };
 const std::size_t old_index = static_cast<std::size_t>(
 old - keyframes_.data());
 SlamKeyframe* recent = find_keyframe(work.to_id);
 if (recent == nullptr) {
 loop_search_.reset();
 return defer("loop search invalidated by keyframe retention");
 }
 const std::size_t recent_index = static_cast<std::size_t>(
 recent - keyframes_.data());
 const auto historical_bounds = centered_bounds(old_index);
 const auto recent_bounds = causal_bounds(recent_index);
 const iface::MapSnapshot historical_map =
 historical_submap_around(old_index, false);
 const iface::MapSnapshot recent_map =
 submap_ending_at(recent_index, false);
 struct AlignmentHypothesis {
 Pose2 correction;
 WindowEvidence forward;
 WindowEvidence reverse;
 double rank = -std::numeric_limits<double>::infinity();
 };
 std::vector<AlignmentHypothesis> alignments;
 const auto same_alignment = [&](const Pose2& a, const Pose2& b) {
 return pose_translation_error(a, b) <=
 match_cfg.ambiguity_distance_m &&
 std::abs(core::wrap_angle(a.heading_rad - b.heading_rad)) <=
 match_cfg.ambiguity_heading_rad;
 };
 const auto evaluate_alignment = [&](const Pose2& correction) {
 const bool duplicate = std::any_of(
 alignments.begin(), alignments.end(),
 [&](const AlignmentHypothesis& existing) {
 return same_alignment(existing.correction, correction);
 });
 if (duplicate) return;
 const WindowEvidence forward = score_window(
 recent_bounds, historical_map, correction);
 const WindowEvidence reverse = score_window(
 historical_bounds, recent_map, inverse_pose(correction));
 if (!forward.valid || !reverse.valid ||
 !(forward.returns > 0 && reverse.returns > 0)) {
 return;
 }
 const double total_returns = static_cast<double>(
 forward.returns + reverse.returns);
 const double pooled_overlap =
 (forward.overlap * static_cast<double>(forward.returns) +
 reverse.overlap * static_cast<double>(reverse.returns)) /
 total_returns;
 if (pooled_overlap < config_.gate.minimum_overlap) return;
 const double rank = 0.5 *
 (forward.score * forward.overlap + reverse.score * reverse.overlap);
 alignments.push_back({correction, forward, reverse, rank});
 };
 evaluate_alignment(forward_correction);
 evaluate_alignment(reverse_as_forward);
 evaluate_alignment(Pose2{});
 if (alignments.empty()) {
 return reject_candidate("submap alignment has insufficient overlap");
 }
 std::sort(alignments.begin(), alignments.end(),
 [](const AlignmentHypothesis& a, const AlignmentHypothesis& b) {
 return a.rank > b.rank;
 });
 const double alignment_margin = alignments.size() > 1
 ? alignments[0].rank - alignments[1].rank : 1.0;
 if (alignments.size() > 1 &&
 alignment_margin < config_.gate.minimum_score_margin) {
 return reject_candidate("submap alignment is geometrically ambiguous");
 }
 const AlignmentHypothesis& alignment = alignments.front();
 const Pose2 aligned_current = compose_pose(
 alignment.correction, current_pose);
 const Pose2 aligned_old = compose_pose(
 inverse_pose(alignment.correction), old->pose);
 work.forward.x_m = aligned_current.position.x;
 work.forward.y_m = aligned_current.position.y;
 work.forward.yaw_rad = aligned_current.heading_rad;
 work.forward.score = alignment.forward.score;
 work.forward.overlap = alignment.forward.overlap;
 work.forward.score_margin = std::min(
 work.forward.score_margin, alignment_margin);
 work.reverse.x_m = aligned_old.position.x;
 work.reverse.y_m = aligned_old.position.y;
 work.reverse.yaw_rad = aligned_old.heading_rad;
 work.reverse.score = alignment.reverse.score;
 work.reverse.overlap = alignment.reverse.overlap;
 work.reverse.score_margin = alignment_margin;
 // Compare the bidirectional fraction of explained returns. Both match
 // quality and overlap contribute evidence; a high score over a small
 // accidental fragment must not outrank a broadly supported revisit.
 const double two_way_score = 0.5 *
 (work.forward.score * work.forward.overlap +
 work.reverse.score * work.reverse.overlap);
 work.verified.push_back({work.old_id, work.forward, work.reverse,
 two_way_score});
 ++work.candidate_index;
 work.phase = work.candidate_index >= work.candidates.size()
 ? LoopSearchWork::Phase::kSelect
 : LoopSearchWork::Phase::kPrepare;
 return defer("loop candidate comparison pending");
 }
 if (work.phase != LoopSearchWork::Phase::kSelect) {
 loop_search_.reset();
 return defer("loop search entered invalid phase");
 }
 if (work.verified.empty()) {
 const std::string final_diagnostic =
 "no candidate verified: " + work.last_rejection;
 loop_search_.reset();
 return defer(final_diagnostic);
 }
 std::sort(work.verified.begin(), work.verified.end(),
 [](const LoopSearchWork::VerifiedCandidate& lhs,
 const LoopSearchWork::VerifiedCandidate& rhs) {
 if (lhs.rank != rhs.rank) return lhs.rank > rhs.rank;
 return lhs.old_id < rhs.old_id;
 });
 // Candidates from neighbouring historical anchors can still describe the
 // same global alignment. Cluster them by the correction transform produced
 // for the frozen scan, using the matcher's existing ambiguity resolution.
 const auto candidate_correction = [&](
 const LoopSearchWork::VerifiedCandidate& candidate) {
 Pose2 correction;
 correction.heading_rad = core::wrap_angle(
 candidate.forward.yaw_rad - work.match_est.yaw_rad);
 const double c = std::cos(correction.heading_rad);
 const double s = std::sin(correction.heading_rad);
 correction.position.x = candidate.forward.x_m -
 (c * work.match_est.x_m - s * work.match_est.y_m);
 correction.position.y = candidate.forward.y_m -
 (s * work.match_est.x_m + c * work.match_est.y_m);
 return correction;
 };
 const auto same_correction_mode = [&](
 const LoopSearchWork::VerifiedCandidate& a,
 const LoopSearchWork::VerifiedCandidate& b) {
 const Pose2 ca = candidate_correction(a);
 const Pose2 cb = candidate_correction(b);
 return std::hypot(ca.position.x - cb.position.x,
 ca.position.y - cb.position.y) <= match_cfg.ambiguity_distance_m &&
 std::abs(core::wrap_angle(ca.heading_rad - cb.heading_rad)) <=
 match_cfg.ambiguity_heading_rad;
 };
 const LoopSearchWork::VerifiedCandidate best_mode = work.verified.front();
 // If the pending anchor independently verifies inside the winning alignment
 // mode, use that exact anchor for the second observation. This prevents a
 // sliding neighbouring submap from resetting temporal confirmation.
 if (pending_loop_valid_) {
 const auto pending = std::find_if(
 work.verified.begin(), work.verified.end(),
 [&](const LoopSearchWork::VerifiedCandidate& candidate) {
 return candidate.old_id == pending_loop_anchor_id_ &&
 same_correction_mode(best_mode, candidate);
 });
 if (pending != work.verified.end()) {
 std::iter_swap(work.verified.begin(), pending);
 }
 }
 const LoopSearchWork::VerifiedCandidate* competing_place = nullptr;
 for (const auto& candidate : work.verified) {
 if (!same_correction_mode(best_mode, candidate)) {
 competing_place = &candidate;
 break;
 }
 }
 if (competing_place != nullptr) {
 // The winning correction mode must separate from the strongest genuinely
 // different alignment using the same score-margin contract as the matcher.
 const double cross_candidate_margin =
 best_mode.rank - competing_place->rank;
 if (cross_candidate_margin < config_.gate.minimum_score_margin) {
 const std::string diagnostic =
 "loop rejected: ambiguous historical places " +
 std::to_string(best_mode.old_id) + " vs " +
 std::to_string(competing_place->old_id);
 loop_search_.reset();
 return defer(diagnostic);
 }
 work.verified[0].forward.score_margin = std::min(
 work.verified[0].forward.score_margin, cross_candidate_margin);
 }
 work.old_id = work.verified[0].old_id;
 work.forward = work.verified[0].forward;
 work.reverse = work.verified[0].reverse;

 SlamKeyframe* old_ptr = find_keyframe(work.old_id);
 SlamKeyframe* to_ptr = find_keyframe(work.to_id);
 if (old_ptr == nullptr || to_ptr == nullptr) {
 loop_search_.reset();
 return defer("loop search invalidated by keyframe retention");
 }
 const SlamKeyframe& old = *old_ptr;
 const ScanMatchResult fwd = work.forward;
 const ScanMatchResult rev = work.reverse;
    PoseGraphKeyframe from;
    from.id = old.id;
    from.stamp_s = old.stamp_s;
    from.initial_pose = old.pose;
    PoseGraphKeyframe to;
 to.id = to_ptr->id;
 to.stamp_s = to_ptr->stamp_s;
 to.initial_pose = to_ptr->pose;
 const LoopClosureGate gate(config_.gate);
    LoopClosureEvidence evidence;
    evidence.match_score = fwd.score;
    evidence.overlap = std::min(fwd.overlap, rev.overlap);
    evidence.score_margin = fwd.score_margin;
    evidence.measured_relative_pose = relative_pose(
        old.pose, Pose2{core::Vec2{fwd.x_m, fwd.y_m}, fwd.yaw_rad});
    evidence.information = pose_graph_information(fwd.position_sigma_m,
                                                  fwd.heading_sigma_rad);
    const LoopClosureDecision decision = gate.validate(from, to, evidence);
 if (!decision.accepted) {
 return reject_search(decision.diagnostic);
 }

 // In a surveyed region, reject a keyframe association before graph
 // optimization when its directly matched pose conflicts with the absolute
 // map. The later post-optimization check remains authoritative because it
 // evaluates the correction the graph would actually publish. This early
 // independent gate avoids solving a growing graph for a candidate that can
 // already be disproved by ROBOT-side map and scan evidence.
 if (absolute_map != nullptr && absolute_map->has_raster()) {
 ScanMatcher evidence_matcher(refine_cfg);
 evidence_matcher.set_lidar_mount(base_from_lidar_);
 const auto beams = evidence_matcher.prepare_beams(
 work.match_obs.ranges_m, work.match_obs.scan_angle_min_rad,
 work.match_obs.scan_angle_increment_rad,
 config_.match_refine_max_beams);
 const PoseScore prior_score = evidence_matcher.score_pose(
 work.match_est.x_m, work.match_est.y_m, work.match_est.yaw_rad,
 beams, *absolute_map, work.match_est.roll_rad,
 work.match_est.pitch_rad, work.match_est.z_m);
 const PoseScore candidate_score = evidence_matcher.score_pose(
 fwd.x_m, fwd.y_m, fwd.yaw_rad, beams, *absolute_map,
 work.match_est.roll_rad, work.match_est.pitch_rad,
 work.match_est.z_m);
 if (prior_score.has_evidence || candidate_score.has_evidence) {
 const bool supported = candidate_score.has_evidence &&
 candidate_score.overlap >= config_.match_minimum_overlap &&
 candidate_score.score >= config_.track_min_score;
 const bool improves = !prior_score.has_evidence ||
 candidate_score.score - prior_score.score >=
 config_.gate.minimum_score_margin;
 if (!supported || !improves) {
 return reject_search(
 "loop rejected early: conflicts with surveyed-map evidence");
 }
 }
 }

 // Optimize a candidate graph transactionally. Association gates below use
 // the correction the solver would actually publish, and a rejected loop
 // leaves the live graph and keyframes byte-for-byte untouched.
 PoseGraphConstraint loop;
 loop.id = to.id * 1000000u + from.id; // unique without a counter
 loop.from = from.id;
 loop.to = to.id;
 loop.measured_relative_pose = evidence.measured_relative_pose;
 loop.information = evidence.information;
 loop.kind = PoseGraphConstraintKind::LoopClosure;
 PoseGraphBackend candidate_graph = graph_;
 std::string reason;
 if (!candidate_graph.add_constraint(loop, &reason)) {
 return reject_search("constraint rejected: " + reason);
 }
 const PoseGraphOptimizationResult opt = candidate_graph.optimize();
 if (!opt.success) {
 loop_search_.reset();
 out.diagnostic = "optimization failed: " + opt.diagnostic;
 last_result_ = out;
 return out;
 }
 const PoseGraphKeyframe* candidate_newest =
 candidate_graph.find_keyframe(keyframes_.back().id);
 if (candidate_newest == nullptr) {
 loop_search_.reset();
 out.diagnostic = "optimized newest node missing";
 last_result_ = out;
 return out;
 }
 const Pose2 prior_newest = keyframes_.back().pose;
 const Pose2 optimized_newest = candidate_newest->initial_pose;
 const Pose2 current_pose{{est.x_m, est.y_m}, est.yaw_rad};
 const Pose2 corrected_here = compose_pose(
 optimized_newest, relative_pose(prior_newest, current_pose));
 const double prospective_m = std::hypot(
 corrected_here.position.x - current_pose.position.x,
 corrected_here.position.y - current_pose.position.y);
 const double prospective_rad = std::abs(core::wrap_angle(
 corrected_here.heading_rad - current_pose.heading_rad));

 // A loop beyond the local tracking window changes the global frame. In a
 // surveyed region it must independently improve the scan's fit to that
 // ROBOT-side map. Unknown survey cells are inconclusive rather than free or
 // occupied; in that case (and in mapless SLAM) require the same rigid map
 // correction to recur after the robot has moved. This prevents one sparse,
 // repetitive view from becoming an irreversible pose-graph constraint.
 const Pose2 matched_pose = corrected_here;
 enum class AbsoluteEvidence { kInconclusive, kConfirms, kConflicts };
 AbsoluteEvidence absolute = AbsoluteEvidence::kInconclusive;
 if (absolute_map != nullptr && absolute_map->has_raster()) {
 ScanMatcher evidence_matcher(refine_cfg);
 evidence_matcher.set_lidar_mount(base_from_lidar_);
 const auto beams = evidence_matcher.prepare_beams(
 obs.ranges_m, obs.scan_angle_min_rad, obs.scan_angle_increment_rad,
 config_.match_refine_max_beams);
 const PoseScore current_score = evidence_matcher.score_pose(
 est.x_m, est.y_m, est.yaw_rad, beams, *absolute_map,
 est.roll_rad, est.pitch_rad, est.z_m);
 const PoseScore matched_score = evidence_matcher.score_pose(
 corrected_here.position.x, corrected_here.position.y,
 corrected_here.heading_rad, beams, *absolute_map,
 est.roll_rad, est.pitch_rad, est.z_m);
 if (current_score.has_evidence || matched_score.has_evidence) {
 const bool candidate_supported = matched_score.has_evidence &&
 matched_score.overlap >= config_.match_minimum_overlap &&
 matched_score.score >= config_.track_min_score;
 const bool improves_absolute_fit = !current_score.has_evidence ||
 matched_score.score - current_score.score >=
 config_.gate.minimum_score_margin;
 absolute = candidate_supported && improves_absolute_fit
 ? AbsoluteEvidence::kConfirms : AbsoluteEvidence::kConflicts;
 }
 }
 // A surveyed-map frame is already an absolute constraint. Any loop that
 // would move the live pose in a region with survey evidence must improve
 // that independent fit, even when each individual correction is smaller
 // than the local tracking window. Otherwise several plausible-looking
 // small loops can cumulatively walk the robot away from the map.
 if (absolute == AbsoluteEvidence::kConflicts) {
 return reject_search(
 "loop rejected: conflicts with surveyed-map evidence");
 }
 // Without an independent surveyed-map confirmation, every map-frame loop
 // constraint needs a second view. A sequence of individually small false
 // loops can walk the pose graph just as far as one large false loop; using
 // the local tracking window as an exemption therefore gives no global
 // safety bound. This mirrors the multi-scan chain doctrine used by mature
 // graph SLAM frontends while keeping unknown survey cells inconclusive.
 if (absolute != AbsoluteEvidence::kConfirms) {
 // Rigid transform C with matched_pose = C * current_pose. Comparing C,
 // rather than raw pose deltas, remains valid after the robot translates or
 // rotates between confirmation scans.
 Pose2 correction;
 correction.heading_rad = core::wrap_angle(
 matched_pose.heading_rad - current_pose.heading_rad);
 const double cc = std::cos(correction.heading_rad);
 const double cs = std::sin(correction.heading_rad);
 correction.position.x = matched_pose.position.x -
 (cc * current_pose.position.x - cs * current_pose.position.y);
 correction.position.y = matched_pose.position.y -
 (cs * current_pose.position.x + cc * current_pose.position.y);
 const double observed_motion_m = pending_loop_valid_
 ? std::hypot(current_pose.position.x -
 pending_loop_observer_pose_.position.x,
 current_pose.position.y - pending_loop_observer_pose_.position.y)
 : 0.0;
 const double observed_motion_rad = pending_loop_valid_
 ? std::abs(core::wrap_angle(current_pose.heading_rad -
 pending_loop_observer_pose_.heading_rad))
 : 0.0;
 const double correction_disagreement_m = pending_loop_valid_
 ? std::hypot(correction.position.x -
 pending_loop_transform_.position.x,
 correction.position.y - pending_loop_transform_.position.y)
 : std::numeric_limits<double>::infinity();
 const double correction_disagreement_rad = pending_loop_valid_
 ? std::abs(core::wrap_angle(correction.heading_rad -
 pending_loop_transform_.heading_rad))
 : std::numeric_limits<double>::infinity();
 const double required_motion_m = std::max(
 config_.match_translation_step_m,
 0.5 * config_.keyframe_translation_m);
 const double required_motion_rad = std::max(
 config_.match_heading_step_rad,
 0.5 * config_.keyframe_heading_rad);
 const bool independent_view =
 observed_motion_m >= required_motion_m ||
 observed_motion_rad >= required_motion_rad;
 const bool same_historical_place = pending_loop_valid_ &&
 pending_loop_anchor_id_ == old.id;
 const bool consistent = same_historical_place &&
 correction_disagreement_m <= 2.0 * config_.match_translation_step_m &&
 correction_disagreement_rad <= 2.0 * config_.match_heading_step_rad;
 if (!pending_loop_valid_ || !independent_view || !consistent) {
 if (!pending_loop_valid_ || !consistent) {
 pending_loop_valid_ = true;
 pending_loop_anchor_id_ = old.id;
 pending_loop_transform_ = correction;
 pending_loop_observer_pose_ = current_pose;
 }
 out.diagnostic = "loop candidate pending temporal confirmation for submap " +
 std::to_string(old.id);
 loop_search_.reset();
 last_result_ = out;
 return out;
 }
 }

 // A confirmed constraint consumes any outstanding temporal proposal. This
 // also clears a proposal left by an earlier inconclusive search when an
 // independent surveyed-map observation confirms the current constraint.
 pending_loop_valid_ = false;
 pending_loop_anchor_id_ = 0;

 out.forward_score = fwd.score;
 out.reverse_score = rev.score;
 out.overlap = std::min(fwd.overlap, rev.overlap);
 out.forward_margin = fwd.score_margin;
 out.reverse_margin = rev.score_margin;
 out.position_sigma_m = std::max(
 fwd.position_sigma_m, rev.position_sigma_m);
 out.heading_sigma_rad = std::max(
 fwd.heading_sigma_rad, rev.heading_sigma_rad);

 graph_ = std::move(candidate_graph);
    // Sync session keyframe poses with the optimized graph. The loop's
    // correction is the newest node's delta; the live estimate may have
    // moved past the last keyframe, so callers compose the delta onto it
    // rather than snapping to the keyframe pose.
    for (const auto& gkf : graph_.keyframes()) {
      for (auto& kf : keyframes_) {
        if (kf.id == gkf.id) kf.pose = gkf.initial_pose;
      }
    }
    out.closed = true;
    out.from_id = from.id;
    out.to_id = to.id;
 // Mark every keyframe that formed the accepted historical submap. A later
 // visit can still close against another submap, while this exact evidence
 // window cannot be reused to manufacture additional confidence.
 const std::size_t closed_center = static_cast<std::size_t>(
 old_ptr - keyframes_.data());
 const std::size_t closed_count = std::max<std::size_t>(
 1, config_.local_submap_keyframes);
 const std::size_t closed_half = closed_count / 2;
 std::size_t closed_begin =
 closed_center > closed_half ? closed_center - closed_half : 0;
 const std::size_t closed_end = std::min(
 keyframes_.size() - 1, closed_begin + closed_count - 1);
 closed_begin = closed_end + 1 > closed_count
 ? closed_end + 1 - closed_count : 0;
 for (std::size_t i = closed_begin; i <= closed_end; ++i) {
 closed_historical_keyframes_.push_back(keyframes_[i].id);
 }
    out.correction_delta =
 relative_pose(prior_newest, optimized_newest);
      // Rigid correction: same motion that moved the newest keyframe,
      // applied to the live estimate (opt ⊕ prior⁻¹ ⊕ est).
      out.correction_m =
          std::hypot(corrected_here.position.x - est.x_m,
                     corrected_here.position.y - est.y_m);
      out.correction_rad = std::abs(core::wrap_angle(corrected_here.heading_rad -
                                                     est.yaw_rad));
      out.corrected_pose = corrected_here;
    out.diagnostic = "loop closed";
    ++loop_closures_;
 loop_search_.reset();
    last_result_ = out;
    return out;
}

}  // namespace lost_signal::robot::autonomy
