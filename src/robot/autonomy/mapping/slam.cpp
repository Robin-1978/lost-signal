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
 double body_roll_rad, double body_pitch_rad, double body_z_m) {
 config = matcher_config;
 matcher = ScanMatcher(config);
 matcher.set_lidar_mount(mount);
 map = std::move(search_map);
 prior_x = prior_x_m;
 prior_y = prior_y_m;
 prior_heading = prior_heading_rad;
 z_m = body_z_m;
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
 return std::hypot(value.x - x, value.y - y) <
 config.ambiguity_distance_m &&
 std::abs(core::wrap_angle(value.heading - heading)) <
 config.ambiguity_heading_rad;
 };
 const auto existing = std::find_if(
 hypotheses.begin(), hypotheses.end(), same);
 if (existing != hypotheses.end()) {
 if (candidate.score > existing->score) *existing = candidate;
 return;
 }
 constexpr std::size_t kMaxHypotheses = 24;
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
 double z_m = 0.0;
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
 };
 enum class Phase { kPrepare, kCoarse, kForward, kReverse };

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
  last_attempt_s_ = -1.0e9;
  keyframes_at_last_attempt_ = 0;
  last_result_ = SlamLoopResult{};
  track_corrections_ = 0;
  last_track_s_ = -1.0e9;
  have_tracked_ = false;
  last_track_ = SlamTrackResult{};
  track_submap_ = iface::MapSnapshot{};
  track_submap_id_ = 0;
 pending_loop_valid_ = false;
 pending_loop_s_ = 0.0;
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

iface::MapSnapshot SlamSession::submap_for(const SlamKeyframe& kf) const {
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
    for (int ay = -1; ay <= 1; ++ay) {
      for (int ax = -1; ax <= 1; ++ax) {
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

iface::MapSnapshot SlamSession::submap(std::uint64_t keyframe_id) const {
 for (const auto& kf : keyframes_) {
 if (kf.id == keyframe_id) return submap_for(kf);
 }
 return {};
}

SlamTrackResult SlamSession::track(const EstimatedState& est,
                                    const iface::ObservationFrame& obs) {
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
  if (track_submap_id_ != anchor.id) {
    track_submap_ = submap_for(anchor);
    track_submap_id_ = anchor.id;
  }
  ScanMatcherConfig match_cfg;
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
  ScanMatchResult rev = matcher.localize(
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
 if (!std::isfinite(correction_m) || !std::isfinite(correction_rad) ||
 correction_m > config_.track_max_correction_m ||
 correction_rad > config_.track_max_correction_rad ||
 match.score < config_.track_min_score ||
 match.score_margin < config_.track_min_margin) {
 out.diagnostic = "track rejected: weak or oversized correction";
 last_track_ = out;
 return out;
 }
  out.accepted = true;
  out.x_m = match.x_m;
  out.y_m = match.y_m;
  out.yaw_rad = match.yaw_rad;
  out.position_sigma_m = match.position_sigma_m;
  out.heading_sigma_rad = match.heading_sigma_rad;
  out.score = match.score;
  out.overlap = std::min(match.overlap, rev.overlap);
  out.score_margin = match.score_margin;
 out.correction_m = correction_m;
 out.correction_rad = correction_rad;
  out.diagnostic = "track corrected";
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
 const double pending_timeout_s = std::max(
 config_.gate.minimum_temporal_separation_s,
 2.0 * config_.loop_attempt_interval_s);
 if (pending_loop_valid_ &&
 now_s - pending_loop_s_ > pending_timeout_s) {
 pending_loop_valid_ = false;
 }
  ScanMatcherConfig match_cfg;
  match_cfg.search_translation_m = config_.match_search_translation_m;
  match_cfg.translation_step_m = config_.match_translation_step_m;
  match_cfg.search_heading_rad = config_.match_search_heading_rad;
  match_cfg.heading_step_rad = config_.match_heading_step_rad;
  match_cfg.maximum_returns = config_.match_max_beams;
  match_cfg.minimum_overlap = config_.match_minimum_overlap;
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
 refine_cfg.search_translation_m = config_.match_refine_translation_m;
 refine_cfg.translation_step_m = config_.match_refine_translation_step_m;
 refine_cfg.search_heading_rad = config_.match_refine_heading_rad;
 refine_cfg.heading_step_rad = config_.match_refine_heading_step_rad;
 refine_cfg.maximum_returns = config_.match_refine_max_beams;
 refine_cfg.minimum_overlap = config_.match_minimum_overlap;
 // The dense pass resolves one mode selected by the wide pass. Adjacent
 // raster cells belong to that mode and must not manufacture ambiguity.
 refine_cfg.ambiguity_distance_m = std::max(
 0.25, 2.0 * config_.match_refine_translation_step_m);
 refine_cfg.ambiguity_heading_rad = std::max(
 0.15, 2.0 * config_.match_refine_heading_step_rad);
 if (!loop_search_) {
 if (now_s - last_attempt_s_ < config_.loop_attempt_interval_s ||
 keyframes_.size() == keyframes_at_last_attempt_) {
 out.diagnostic = "attempt rate limited";
 last_result_ = out;
 return out;
 }
 last_attempt_s_ = now_s;
 keyframes_at_last_attempt_ = keyframes_.size();
 loop_search_ = std::make_unique<LoopSearchWork>();
 loop_search_->match_est = est;
 loop_search_->match_obs = obs;
 loop_search_->to_id = keyframes_.back().id;
 // Candidates: temporally separated and spatially near the scan pose.
 // Store stable ids because keyframe ingestion continues while matching is
 // sliced across ticks.
 for (std::size_t i = 0; i + 1 < keyframes_.size(); ++i) {
 const SlamKeyframe& kf = keyframes_[i];
 const std::size_t separation = keyframes_.size() - 1 - i;
 if (separation < config_.minimum_loop_keyframe_separation ||
 now_s - kf.stamp_s < config_.gate.minimum_temporal_separation_s) {
 continue;
 }
 const double distance = std::hypot(
 est.x_m - kf.pose.position.x, est.y_m - kf.pose.position.y);
 if (distance <= config_.gate.maximum_candidate_distance_m) {
 loop_search_->candidates.push_back({kf.id, distance});
 }
 }
 std::sort(loop_search_->candidates.begin(),
 loop_search_->candidates.end(),
 [](const LoopSearchWork::Candidate& a,
 const LoopSearchWork::Candidate& b) {
 return a.distance < b.distance;
 });
 if (loop_search_->candidates.size() >
 config_.max_candidates_per_attempt) {
 loop_search_->candidates.resize(config_.max_candidates_per_attempt);
 }
 if (loop_search_->candidates.empty()) {
 loop_search_.reset();
 out.diagnostic = "no eligible candidates";
 last_result_ = out;
 return out;
 }
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
 work.phase = LoopSearchWork::Phase::kPrepare;
 if (work.candidate_index >= work.candidates.size()) {
 const std::string final_diagnostic =
 "no candidate verified: " + work.last_rejection;
 loop_search_.reset();
 return defer(final_diagnostic);
 }
 return defer("loop search pending: " + diagnostic);
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
 work.match_est.y_m, work.match_est.yaw_rad, submap_for(*old),
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
 iface::MapSnapshot sub = std::move(work.lattice.map);
 work.lattice.start(refine_cfg, base_from_lidar_,
 work.match_obs.ranges_m, work.match_obs.scan_angle_min_rad,
 work.match_obs.scan_angle_increment_rad, work.coarse.x_m,
 work.coarse.y_m, work.coarse.yaw_rad, std::move(sub),
 work.match_est.roll_rad, work.match_est.pitch_rad,
 work.match_est.z_m);
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
 SlamKeyframe now_kf;
 now_kf.id = 0;
 now_kf.stamp_s = work.match_obs.stamp_ns * 1e-9;
 now_kf.pose.position.x = work.forward.x_m;
 now_kf.pose.position.y = work.forward.y_m;
 now_kf.pose.heading_rad = work.forward.yaw_rad;
 now_kf.roll_rad = work.match_est.roll_rad;
 now_kf.pitch_rad = work.match_est.pitch_rad;
 now_kf.z_m = work.match_est.z_m;
 now_kf.ranges_m = work.match_obs.ranges_m;
 now_kf.angle_min_rad = work.match_obs.scan_angle_min_rad;
 now_kf.angle_increment_rad = normalized_scan_increment(
 work.match_obs.scan_angle_increment_rad,
 work.match_obs.ranges_m.size());
 SlamKeyframe* old = find_keyframe(work.old_id);
 if (old == nullptr) {
 loop_search_.reset();
 return defer("loop search invalidated by keyframe retention");
 }
 work.lattice.start(refine_cfg, base_from_lidar_, old->ranges_m,
 old->angle_min_rad, old->angle_increment_rad,
 old->pose.position.x, old->pose.position.y, old->pose.heading_rad,
 submap_for(now_kf), old->roll_rad, old->pitch_rad, old->z_m);
 work.phase = LoopSearchWork::Phase::kReverse;
 return defer("loop reverse verification pending");
 }
 if (!work.lattice.advance(config_.match_work_budget)) {
 return defer("loop reverse verification pending");
 }
 work.reverse = work.lattice.result;
 if (!work.reverse.accepted) {
 return reject_candidate(
 "reverse match rejected: " + work.reverse.rejection_reason);
 }
 if (work.reverse.overlap < config_.gate.minimum_overlap) {
 return reject_candidate("reverse overlap below loop gate");
 }

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
 return reject_candidate(decision.diagnostic);
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
 return reject_candidate(
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
 return reject_candidate("constraint rejected: " + reason);
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
 const bool large_correction =
 prospective_m > config_.track_window_m ||
 prospective_rad > config_.track_heading_rad;
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
 return reject_candidate(
 "loop rejected: conflicts with surveyed-map evidence");
 }
 if (large_correction) {
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
 const bool consistent =
 correction_disagreement_m <= 2.0 * config_.match_translation_step_m &&
 correction_disagreement_rad <= 2.0 * config_.match_heading_step_rad;
 if (!pending_loop_valid_ || !independent_view || !consistent) {
 if (!pending_loop_valid_ || !consistent) {
 pending_loop_valid_ = true;
 pending_loop_s_ = now_s;
 pending_loop_transform_ = correction;
 pending_loop_observer_pose_ = current_pose;
 }
 out.diagnostic = "loop candidate pending temporal confirmation";
 loop_search_.reset();
 last_result_ = out;
 return out;
 }
 pending_loop_valid_ = false;
 }

 out.forward_score = fwd.score;
 out.reverse_score = rev.score;
 out.overlap = std::min(fwd.overlap, rev.overlap);
 out.forward_margin = fwd.score_margin;
 out.reverse_margin = rev.score_margin;

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
