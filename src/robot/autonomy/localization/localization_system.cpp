#include "robot/autonomy/localization/localization_system.hpp"

#include <algorithm>
#include <cmath>

namespace lost_signal::robot::autonomy {

void LocalizationSystem::reset() {
  supervisor_.reset(false);
 global_search_.cancel();
  startup_hyps_.reset();
 // A surveyed-map recovery must retain uncertainty through corridor-like
 // observations. Partial wall evidence can track a candidate, but it cannot
 // establish the missing along-wall coordinate as an absolute map fix.
 recovery_hyps_.set_require_full_observation_for_promotion(true);
 recovery_hyps_.reset();
  slam_.reset();
  slam_origin_seeded_ = false;
 global_origin_valid_ = false;
 global_search_radius_m_ = 0.0;
 global_search_heading_radius_rad_ = 0.0;
 map_evidence_cached_ = false;
 cached_map_id_.clear();
 cached_map_revision_ = 0;
 cached_map_has_known_occupancy_ = false;
 last_recovery_local_scan_ns_ = core::kInvalidTime;
}

void LocalizationSystem::set_lidar_mount(
    const core::RigidTransform& base_from_lidar) {
 base_from_lidar_ = base_from_lidar;
  global_search_.set_lidar_mount(base_from_lidar);
  startup_hyps_.set_lidar_mount(base_from_lidar);
 recovery_hyps_.set_lidar_mount(base_from_lidar);
  slam_.set_lidar_mount(base_from_lidar);
}

bool LocalizationSystem::localized() const {
  return supervisor_.localized();
}

std::size_t LocalizationSystem::slam_loops_closed() const {
  return slam_.loop_closure_count();
}

const SlamLoopResult& LocalizationSystem::last_slam_result() const {
  return slam_.last_result();
}

std::size_t LocalizationSystem::slam_keyframes() const {
  return slam_.keyframes().size();
}

LocalizationSystem::TickResult LocalizationSystem::update(
    const iface::MapSnapshot& map, const iface::ObservationFrame& obs,
    core::MonotonicNs now_ns, bool has_observation, IEstimator& estimator,
    IMapper& mapper) {
  TickResult out;
 if (global_search_.active() && !global_search_.active_map_matches(map)) {
 // WorkState intentionally holds a zero-copy view of an immutable map.
 // Never continue it across a provider revision or a replaced anonymous
 // snapshot; the supervisor may start a fresh bounded search below.
 global_search_.cancel();
 recovery_hyps_.reset();
 global_origin_valid_ = false;
 global_search_radius_m_ = 0.0;
 global_search_heading_radius_rad_ = 0.0;
 }
 if (!map_evidence_cached_ || cached_map_id_ != map.map_id ||
 cached_map_revision_ != map.revision) {
 cached_map_id_ = map.map_id;
 cached_map_revision_ = map.revision;
 cached_map_has_known_occupancy_ = map.has_known_occupancy();
 map_evidence_cached_ = true;
 }
 bool local_match_attempted = false;
 const bool frame_reanchored =
 run_localization(map, cached_map_has_known_occupancy_, obs, now_ns,
 estimator, out.action, local_match_attempted);
 if (frame_reanchored) {
 // Keyframes and the rolling raster were authored in the old map frame.
 // Keeping either after an absolute global fix lets local SLAM pull the
 // estimator back toward the obsolete frame on the next tick.
 slam_.reset();
 mapper.reset();
out.local_plan_reset_requested = true;
 }
 if (!frame_reanchored) {
   run_motion_evidence(obs, has_observation, cached_map_has_known_occupancy_,
                       estimator);
 }
 // Absolute survey matching and two-sided scan-to-submap tracking are both
 // heavy scan searches. Never stack them in one 20 ms control tick: the
 // absolute correction owns its due tick, and SLAM tracking uses intervening
 // scans. This preserves both constraints while bounding latency spikes.
 if (!frame_reanchored && !local_match_attempted) {
 run_tracking(obs, has_observation, estimator);
 }
 if (supervisor_.localized() &&
 run_slam(map, cached_map_has_known_occupancy_, obs, has_observation,
 estimator, mapper)) {
    out.replan_requested = true;
out.local_plan_reset_requested = true;
  }
 out.motion_authorized = supervisor_.localized();
 out.observation_motion_requested = supervisor_.localized() &&
 (global_search_.active() || !recovery_hyps_.empty());
  return out;
}

// Incremental scan-to-submap tracking, ahead of the supervisor-sequenced
// corrections. Bounded window (±track_window_m) so it can never kidnap;
// applied according to the frame owner below. In mapless mode the evolving
// SLAM submap is the navigation frame, so a verified track is a local pose
// observation and may reduce navigation covariance. With a surveyed map the
// same keyframe chain is only a correlated relative constraint and cannot
// claim new absolute-map information.
// Runs after the supervisor establishes a map frame. A known-map cold start
// must not feed arbitrary odometry-frame SLAM corrections into a concurrent
// global search; mapless startup establishes its odometry origin immediately.
void LocalizationSystem::run_tracking(const iface::ObservationFrame& obs,
                                     bool has_observation,
                                     IEstimator& estimator) {
 // Before the global/mapless startup branch establishes a map frame, the
 // SLAM submap has only an arbitrary odometry-frame anchor. Feeding those
 // corrections into a known-map startup can bias the motion delta used to
 // advance an incremental global-search result. Start tracking once the
 // supervisor has established the frame; mapless startup does so immediately.
 if (!supervisor_.localized() || !has_observation || !obs.scan_valid) return;
 // In localization mode the surveyed map owns global correction authority;
 // an opposing scan-to-submap update stays rejected and the independent
 // scan-to-map chain resolves it. In mapless SLAM there is no absolute map,
 // so the tracker may accept that sign only after its own multi-view gate.
 const bool mapless_slam =
 supervisor_.startup_mode() == StartupMode::kSlamOnly;
 SlamTrackResult tr = slam_.track(
 estimator.estimate(), obs, mapless_slam);
 if (mapless_slam && tr.scan_motion_valid) {
   estimator.observe_scan_motion(
       tr.scan_estimated_dx_m, tr.scan_estimated_dy_m,
       tr.scan_measured_dx_m, tr.scan_measured_dy_m,
       tr.position_sigma_m, obs.stamp_ns);
 }
  if (!tr.accepted) return;
 if (mapless_slam) {
 // SlamSession releases at most one accepted observation per causal submap
 // anchor. That makes this an incremental graph-front-end update instead of
 // repeatedly counting the same walls as independent evidence.
 if (tr.pose_information_valid) {
 estimator.correct_pose_information(
 tr.x_m, tr.y_m, tr.yaw_rad, tr.pose_information,
 tr.position_sigma_m, tr.heading_sigma_rad, obs.stamp_ns);
 } else {
 estimator.correct_pose(tr.x_m, tr.y_m, tr.yaw_rad,
 tr.position_sigma_m, tr.heading_sigma_rad, obs.stamp_ns);
 }
 } else {
 // A surveyed map owns absolute authority. Scan-to-submap shares the
 // estimator's keyframe history, so it may improve local consistency but
 // must preserve absolute map covariance. Global scan matching supplies
 // independent map information.
 estimator.correct_relative_pose(tr.x_m, tr.y_m, tr.yaw_rad,
 tr.position_sigma_m, tr.heading_sigma_rad, obs.stamp_ns);
 }
}

void LocalizationSystem::run_motion_evidence(
    const iface::ObservationFrame& obs, bool has_observation,
    bool map_has_known_occupancy, IEstimator& estimator) {
  // A surveyed map already supplies an independent absolute scan constraint.
  // Relative motion evidence is reserved for mapless SLAM, where wheel/IMU
  // prediction is the only continuous prior and the live LiDAR pair is the
  // robot's own observable traction witness. Running both chains on one known
  // map correlated the same endpoints and made normal motion look like slip.
  if (map_has_known_occupancy) return;
  if (!supervisor_.localized() || !has_observation || !obs.scan_valid) return;
  const SlamMotionResult motion =
      slam_.observe_motion(estimator.estimate(), obs);
  if (!motion.valid) return;
  estimator.observe_scan_motion(
      motion.estimated_dx_m, motion.estimated_dy_m,
      motion.measured_dx_m, motion.measured_dy_m,
      motion.position_sigma_m, obs.stamp_ns);
}

bool LocalizationSystem::run_slam(const iface::MapSnapshot& map,
 bool map_has_known_occupancy,
 const iface::ObservationFrame& obs,
 bool has_observation, IEstimator& estimator,
                                  IMapper& mapper) {
  if (!has_observation || !obs.scan_valid) return false;
  slam_.ingest(estimator.estimate(), obs);
 // A surveyed map and mapless pose graph are alternative owners of the
 // global frame. In mapped localization, supervised scan-to-map corrections
 // already provide the absolute constraint; committing a second pose-graph
 // frame correction duplicates work and can make the two estimators fight.
 // Keep keyframing and local scan-to-submap tracking in both modes, but run
 // loop closure only when SLAM actually owns the global frame.
 const bool slam_owns_global_frame =
 supervisor_.startup_mode() == StartupMode::kSlamOnly ||
 !map_has_known_occupancy;
 if (!slam_owns_global_frame) return false;
 const iface::MapSnapshot* absolute_map =
 supervisor_.localized() && map_has_known_occupancy &&
 supervisor_.startup_mode() != StartupMode::kSlamOnly
 ? &map : nullptr;
 SlamLoopResult loop = slam_.try_close_loops(
 estimator.estimate(), obs, absolute_map);
  if (!loop.closed) return false;
  // Accepted closure: snap the estimate to the optimized pose and rebuild
  // the rolling map from keyframes at their optimized poses. Past cells
  // fed at drifted poses are stale; replaying keyframe scans re-registers
  // them. The decision branch refreshes on the corrected estimate.
  estimator.seed_pose(loop.corrected_pose.position.x,
                     loop.corrected_pose.position.y,
 loop.corrected_pose.heading_rad, loop.position_sigma_m,
 loop.heading_sigma_rad,
                     obs.stamp_ns);
  mapper.reset();
  for (const auto& kf : slam_.keyframes()) {
    iface::ObservationFrame kobs;
    kobs.sequence = 1;  // rebuild order is keyframe order, not live sequence
    kobs.stamp_ns = static_cast<core::MonotonicNs>(kf.stamp_s * 1e9);
    kobs.source = obs.source;
    kobs.frame = core::Frame::kLidarLink;
    kobs.scan_valid = true;
    kobs.ranges_m = kf.ranges_m;
    kobs.static_ranges_m = kf.static_ranges_m;
    kobs.scan_static_valid = !kf.static_ranges_m.empty();
    kobs.scan_angle_min_rad = kf.angle_min_rad;
    kobs.scan_angle_increment_rad = kf.angle_increment_rad;
    EstimatedState est;
    est.valid = true;
    est.stamp_ns = kobs.stamp_ns;
    est.x_m = kf.pose.position.x;
    est.y_m = kf.pose.position.y;
    est.yaw_rad = kf.pose.heading_rad;
    mapper.update(kobs, est);
  }
 mapper.mark_slam_replay(loop.to_id);
  return true;
}

bool LocalizationSystem::run_localization(
 const iface::MapSnapshot& map, bool map_has_known_occupancy,
 const iface::ObservationFrame& obs,
 core::MonotonicNs now_ns, IEstimator& estimator, std::string& action,
 bool& local_match_attempted) {
 local_match_attempted = false;
  const double now_s = now_ns * 1e-9;
  const EstimatedState& est = estimator.estimate();
  LocalizationHealth health;
  health.valid = est.valid;
  health.position_sigma_m = est.position_sigma_m;
  health.heading_sigma_rad = est.heading_sigma_rad;
  health.confidence = est.confidence;
  health.age_s = est.valid ? now_s - est.stamp_ns * 1e-9 : 1e9;
  std::string reason;
 LocalizationAction act = LocalizationAction::kDefer;
 action = localization_action_name(act);
 // An explicit SLAM-only deployment trusts the current odometry origin even
 // when a sparse navigation prior exists. Waiting for a global-search action
 // here deadlocks because the supervisor deliberately suppresses that action
 // in this mode, so no keyframe session would ever start.
 if (!supervisor_.localized() &&
 supervisor_.startup_mode() == StartupMode::kSlamOnly &&
 !slam_origin_seeded_) {
 estimator.seed_pose(est.x_m, est.y_m, est.yaw_rad, 0.1, 0.05,
 obs.stamp_ns);
 supervisor_.reset(true);
 slam_origin_seeded_ = true;
 action = startup_mode_name(StartupMode::kSlamOnly);
 supervisor_.record_global(true, ScanMatchResult{});
 return false;
 }
 // SLAM-only means the ROBOT's keyframe graph owns the navigation frame.
 // A sparse or incidental survey may still be present for place resolution
 // and planning, but it is not an absolute localization source in this mode.
 // Bypass both local scan-to-survey correction and global recovery after the
 // odometry/SLAM origin has been established.
 if (supervisor_.startup_mode() == StartupMode::kSlamOnly) {
 action = "slam-only";
 return false;
 }
 const bool fresh_scan = obs.scan_valid && !obs.ranges_m.empty();
 const core::MonotonicNs scan_stamp_ns = iface::scan_stamp_ns(obs);
 const core::MonotonicNs local_interval_ns = static_cast<core::MonotonicNs>(
 std::max(0.0, supervisor_.config().correction_interval_s) * 1e9);
 const bool new_recovery_scan = scan_stamp_ns != core::kInvalidTime &&
 (last_recovery_local_scan_ns_ == core::kInvalidTime ||
 scan_stamp_ns > last_recovery_local_scan_ns_);
 const bool recovery_local_due = new_recovery_scan &&
 (last_recovery_local_scan_ns_ == core::kInvalidTime ||
 scan_stamp_ns - last_recovery_local_scan_ns_ >= local_interval_ns);
 if (global_search_.active() && supervisor_.localized() && fresh_scan &&
 recovery_local_due) {
 last_recovery_local_scan_ns_ = scan_stamp_ns;
 local_match_attempted = true;
 bool full_observation_correction = false;
 const bool corrected = try_local_correction(
 map, obs, now_ns, estimator, action, &full_observation_correction);
 const EstimatedState& corrected_estimate = estimator.estimate();
 const auto& config = supervisor_.config();
 const bool covariance_resolved = corrected &&
 full_observation_correction &&
 std::isfinite(corrected_estimate.position_sigma_m) &&
 std::isfinite(corrected_estimate.heading_sigma_rad) &&
 corrected_estimate.position_sigma_m < config.lost_position_sigma_m &&
 corrected_estimate.heading_sigma_rad < config.lost_heading_sigma_rad;
 if (covariance_resolved) {
 global_search_.cancel();
 recovery_hyps_.reset();
 action = "local_correction:global recovery resolved";
 }
 return false;
 }
 // A new supervisor action consumes a measurement opportunity. Never spend
 // local/global cadence or retry backoff on an empty asynchronous tick. An
 // already active cooperative search is different: it owns a frozen scan
 // and must keep advancing even when this tick carries no new lidar frame.
 if (!global_search_.active()) {
 if (!fresh_scan) return false;
 if (!recovery_hyps_.empty()) {
 action = "recovery-tracking";
 } else {
 act = supervisor_.next(
 now_s, est.x_m, est.y_m, est.yaw_rad, health, &reason);
 action = localization_action_name(act);
 }
 }
  // Deferred startup disambiguation: an ambiguous whole-map search seeds
  // hypotheses instead of guessing; later scans promote or drop them.
 // Deferred hypotheses own their unlocalized scans; ordinary local SLAM
 // remains gated until one hypothesis establishes the map frame.
  if (supervisor_.localized()) startup_hyps_.reset();
 if (supervisor_.localized() && !recovery_hyps_.empty() && fresh_scan) {
 StartupTrackResult tracked = recovery_hyps_.update(
 obs.ranges_m, obs.scan_angle_min_rad, obs.scan_angle_increment_rad,
 map, now_ns, obs.forward_mps, obs.yaw_rate_radps,
 obs.odometry_valid);
 if (tracked.status == StartupTrackStatus::kPromoted) {
 const double position_innovation_m = std::hypot(
 tracked.x_m - est.x_m, tracked.y_m - est.y_m);
 const double heading_innovation_rad = std::abs(
 core::wrap_angle(tracked.yaw_rad - est.yaw_rad));
 if (supervisor_.accept_global_innovation(
 position_innovation_m, heading_innovation_rad,
 est.position_sigma_m, est.heading_sigma_rad)) {
 estimator.correct_pose(
 tracked.x_m, tracked.y_m, tracked.yaw_rad,
 tracked.position_sigma_m, tracked.heading_sigma_rad, obs.stamp_ns);
 supervisor_.record_global(true, ScanMatchResult{});
 action = std::string("recovery-promoted score=") +
 std::to_string(tracked.score) +
 " hypotheses=" + std::to_string(tracked.hypotheses);
 return true;
 }
 recovery_hyps_.reset();
 estimator.note_absolute_rejection(
 tracked.x_m - est.x_m, tracked.y_m - est.y_m,
 core::wrap_angle(tracked.yaw_rad - est.yaw_rad), obs.stamp_ns);
 action = "recovery-promotion rejected by pose covariance";
 return false;
 }
 if (tracked.status == StartupTrackStatus::kTracking) {
 // A single survivor is still not an absolute fix when every supporting
 // scan is rank deficient. Once the fused estimate has crossed the same
 // covariance loss envelope used by LocalizationSupervisor, keeping this
 // candidate alive would block the only recovery path that can search for a
 // different map place. Release the deferred tracker and let the supervisor
 // start a fresh, bounded global search on the next measurement. The
 // condition uses observation rank plus the existing covariance contract;
 // it does not add a scenario-specific timeout or distance threshold.
 const EstimatedState& current = estimator.estimate();
 const auto& config = supervisor_.config();
 const bool uncertainty_outside =
 !std::isfinite(current.position_sigma_m) ||
 !std::isfinite(current.heading_sigma_rad) ||
 current.position_sigma_m >= config.lost_position_sigma_m ||
 current.heading_sigma_rad >= config.lost_heading_sigma_rad;
 if (tracked.hypotheses == 1 &&
 !tracked.full_observation_confirmed && uncertainty_outside) {
 recovery_hyps_.reset();
 action = "recovery-tracking:rank-deficient belief; global retry";
 return false;
 }
 action = std::string("recovery-tracking hypotheses=") +
 std::to_string(tracked.hypotheses);
 return false;
 }
 action = "recovery-tracking lost";
 return false;
 }
if (!supervisor_.localized() && !startup_hyps_.empty() && fresh_scan) {
    StartupTrackResult tracked = startup_hyps_.update(
        obs.ranges_m, obs.scan_angle_min_rad, obs.scan_angle_increment_rad,
 map, now_ns, obs.forward_mps, obs.yaw_rate_radps,
 obs.odometry_valid);
    if (tracked.status == StartupTrackStatus::kPromoted) {
 estimator.seed_pose(tracked.x_m, tracked.y_m, tracked.yaw_rad,
 tracked.position_sigma_m, tracked.heading_sigma_rad, obs.stamp_ns);
      supervisor_.record_global(true, ScanMatchResult{});
      action = "startup-promoted";
 return true;
    }
    if (tracked.status == StartupTrackStatus::kTracking) {
      action = "startup-tracking";
 return false;
    }
    // kLost: fall through — the supervisor cadence (or this tick's
    // global branch) re-runs the whole-map search with fresh evidence.
  }
 // An in-flight whole-map search advances on every localization tick even
 // while the supervisor's retry cadence says "defer". The cadence starts a
 // search; it must not throttle the bounded slices inside that search.
 if (act == LocalizationAction::kDefer && !global_search_.active()) return false;
 const bool requested_global = act == LocalizationAction::kGlobalRelocalization;
  // Mapless startup: no static raster exists, so neither local matching nor
  // the whole-map search can constrain the pose. Seed the odometry origin
  // once (no teleport: the current estimate IS the origin) and let the SLAM
  // session own the drift it can observe. Latched for the session.
 if (requested_global && !map_has_known_occupancy &&
 !slam_origin_seeded_) {
    const StartupDecision startup =
        select_startup_mode(false, /*has_prior_pose=*/false);
    estimator.seed_pose(est.x_m, est.y_m, est.yaw_rad, 0.1, 0.05,
                        obs.stamp_ns);
    supervisor_.reset(true);
    slam_origin_seeded_ = true;
    action = startup_mode_name(startup.mode);
    supervisor_.record_global(true, ScanMatchResult{});
 return false; // same odometry frame: no map/keyframe transform to retire
  }
 if (requested_global || global_search_.active()) {
    // Whole-map search: kidnap recovery and cold start. An accepted global
    // fix establishes the estimate and marks the supervisor localized; a
    // rejection is evidence only and never moves the estimate. An
    // AMBIGUOUS rejection with runner-ups seeds deferred tracking (the
    // next scans disambiguate) instead of guessing now.
 if (!global_search_.active()) {
 const EstimatedState& odom_origin = estimator.odometry_estimate();
 global_origin_valid_ = odom_origin.valid;
 global_origin_odom_x_m_ = odom_origin.x_m;
 global_origin_odom_y_m_ = odom_origin.y_m;
 global_origin_odom_yaw_rad_ = odom_origin.yaw_rad;
 if (supervisor_.localized()) {
 const double prior_sigma_m = std::isfinite(est.position_sigma_m)
 ? std::max(0.0, est.position_sigma_m)
 : supervisor_.config().lost_position_sigma_m;
 const double recovery_radius_m =
 supervisor_.global_recovery_radius_m(prior_sigma_m);
 const double prior_heading_sigma_rad =
 std::isfinite(est.heading_sigma_rad)
 ? std::max(0.0, est.heading_sigma_rad)
 : supervisor_.config().lost_heading_sigma_rad;
 const double recovery_heading_radius_rad =
 supervisor_.global_recovery_heading_radius_rad(
 prior_heading_sigma_rad);
 global_search_radius_m_ = recovery_radius_m;
 global_search_heading_radius_rad_ = recovery_heading_radius_rad;
 global_search_.begin_near(
 obs.ranges_m, obs.scan_angle_min_rad, obs.scan_angle_increment_rad,
 map, now_ns, est.x_m, est.y_m, recovery_radius_m,
 est.yaw_rad, recovery_heading_radius_rad,
 est.roll_rad, est.pitch_rad, est.z_m);
 } else {
 global_search_radius_m_ = 0.0;
 global_search_heading_radius_rad_ = 0.0;
 global_search_.begin(
 obs.ranges_m, obs.scan_angle_min_rad, obs.scan_angle_increment_rad,
 map, now_ns, est.roll_rad, est.pitch_rad, est.z_m);
 }
 }
 const std::optional<GlobalSearchResult> update = global_search_.advance();
 if (!update.has_value()) {
 action = "global_relocalization:search pending";
 return false;
 }
 GlobalSearchResult global = *update;
 core::MonotonicNs correction_stamp_ns = global.stamp_ns;
 // The cooperative search scores a frozen scan over many control ticks.
 // Its winning pose belongs to that scan's acquisition pose, not to the
 // robot's current pose. Carry every hypothesis forward by the ROBOT-only
 // relative motion accumulated while the search was running. Applying the
 // stale pose directly made a moving robot jump backwards after an otherwise
 // correct search and poisoned the following SLAM graph.
 const EstimatedState& current_odom = estimator.odometry_estimate();
 if (global_origin_valid_ && current_odom.valid) {
 const double c0 = std::cos(global_origin_odom_yaw_rad_);
 const double s0 = std::sin(global_origin_odom_yaw_rad_);
 const double wx = current_odom.x_m - global_origin_odom_x_m_;
 const double wy = current_odom.y_m - global_origin_odom_y_m_;
 const double dx = c0 * wx + s0 * wy;
 const double dy = -s0 * wx + c0 * wy;
 const double dyaw = core::wrap_angle(
 current_odom.yaw_rad - global_origin_odom_yaw_rad_);
 const auto advance_pose = [&](double& x, double& y, double& yaw) {
 const double c = std::cos(yaw);
 const double s = std::sin(yaw);
 x += c * dx - s * dy;
 y += s * dx + c * dy;
 yaw = core::wrap_angle(yaw + dyaw);
 };
 advance_pose(global.x_m, global.y_m, global.yaw_rad);
 for (auto& runner_up : global.runner_ups) {
 advance_pose(runner_up.x_m, runner_up.y_m, runner_up.yaw_rad);
 }
 correction_stamp_ns = current_odom.stamp_ns;
 }
 global_origin_valid_ = false;
 bool accept_global = global.accepted;
 bool require_temporal_confirmation = false;
 if (global.accepted && !supervisor_.localized()) {
 const double jump_m = std::hypot(global.x_m - est.x_m,
 global.y_m - est.y_m);
 const double direct_startup_limit_m =
 supervisor_.global_recovery_radius_m(
 supervisor_.config().lost_position_sigma_m);
 if (jump_m > direct_startup_limit_m) {
 accept_global = false;
 require_temporal_confirmation = true;
 global.rejection_reason =
 "distant startup candidate requires motion confirmation";
 }
 }
 if (global.accepted && supervisor_.localized()) {
 // A global candidate may recover accumulated drift, but covariance must
 // widen the admissible innovation continuously. Crossing the supervisor's
 // recovery threshold is not permission for an unbounded one-scan jump in
 // repetitive geometry. A genuine large displacement becomes admissible
 // only when ROBOT uncertainty has grown to support it.
 const double jump_m = std::hypot(global.x_m - est.x_m,
 global.y_m - est.y_m);
 const double heading_jump_rad = std::abs(
 core::wrap_angle(global.yaw_rad - est.yaw_rad));
 const double prior_sigma_m = std::isfinite(est.position_sigma_m)
 ? std::max(0.0, est.position_sigma_m)
 : supervisor_.config().lost_position_sigma_m;
 const double prior_heading_sigma_rad =
 std::isfinite(est.heading_sigma_rad)
 ? std::max(0.0, est.heading_sigma_rad)
 : supervisor_.config().lost_heading_sigma_rad;
 if (!supervisor_.accept_global_innovation(
 jump_m, heading_jump_rad, prior_sigma_m,
 prior_heading_sigma_rad)) {
 accept_global = false;
 global.rejection_reason =
 "global pose correction exceeds trusted covariance";
 }
 }
 if (accept_global && supervisor_.localized()) {
 // AMCL-style recovery keeps a belief across measurements. A one-scan
 // global winner is only a candidate: propagate it with measured odometry
 // and require later scans plus useful motion to confirm it before changing
 // map->odom. This prevents repetitive aisles from teleporting a still
 // coherent local estimate to a geometrically similar pose.
 recovery_hyps_.seed(global, correction_stamp_ns,
 /*require_motion=*/true);
 action = std::string("global_relocalization:confirming score=") +
 std::to_string(global.score) +
 " overlap=" + std::to_string(global.overlap) +
 " margin=" + std::to_string(global.score_margin) +
 " returns=" + std::to_string(global.usable_returns);
 return false;
 }
 const bool trackable_ambiguity =
 !global.accepted && supervisor_.localized() &&
 !global.runner_ups.empty() &&
 (global.rejection_reason == "global match is geometrically ambiguous" ||
 global.rejection_reason == "global match has equivalent support");
 if (trackable_ambiguity) {
 // A repeated aisle or wall is not a reason to discard every globally
 // consistent pose and restart the same one-scan search two seconds later.
 // Keep the separated candidates as an AMCL-style temporal belief, advance
 // all of them with measured odometry, and let later scans plus useful
 // motion remove the aliases. Promotion still passes the joint covariance
 // innovation gate below, so ambiguity can never become a one-scan jump.
 supervisor_.record_global(false, ScanMatchResult{});
 recovery_hyps_.seed(global, correction_stamp_ns,
 /*require_motion=*/true);
 action = std::string("global_relocalization:tracking ambiguity score=") +
 std::to_string(global.score) +
 " overlap=" + std::to_string(global.overlap) +
 " margin=" + std::to_string(global.score_margin) +
 " hypotheses=" + std::to_string(recovery_hyps_.size());
 return false;
 }
 if (accept_global) {
 if (!supervisor_.localized()) {
        // No trustworthy prior exists (unknown start / kidnapped): snap
        // like a deliberate initial pose instead of blending toward a
        // confident-but-wrong dead-reckoned origin.
        estimator.seed_pose(global.x_m, global.y_m, global.yaw_rad,
                            global.position_sigma_m,
global.heading_sigma_rad, correction_stamp_ns);
      } else {
        estimator.correct_pose(global.x_m, global.y_m, global.yaw_rad,
                               global.position_sigma_m,
global.heading_sigma_rad, correction_stamp_ns);
      }
    }
 // A geometrically supported candidate that fails the covariance innovation
 // gate is evidence that the current belief is over-confident. Widen only
 // from the measured residual; do not move the estimate or expose any SIM
 // state. The next bounded search then grows continuously with the actual
 // rejected evidence and can recover from an underestimated prior.
 if (!accept_global && global.accepted && supervisor_.localized()) {
 estimator.note_absolute_rejection(
 global.x_m - est.x_m, global.y_m - est.y_m,
 core::wrap_angle(global.yaw_rad - est.yaw_rad), correction_stamp_ns);
 }
 // A localized search can also fail before producing a raycast-valid pose.
 // When it evaluated real geometric returns and no runner-up belief exists,
 // the frozen covariance envelope is still a measured lower bound on the
 // missing pose. Feed that envelope back once so the next retry expands
 // adaptively; sparse/empty scans have zero coarse evaluations and remain
 // diagnostic only.
 if (supervisor_.localized() && !global.accepted &&
     global.runner_ups.empty() && global.coarse_evaluations > 0 &&
     global.usable_returns > 0 &&
     global_search_radius_m_ > 0.0 &&
     global_search_heading_radius_rad_ > 0.0) {
   // `global_search_*_radius` is an uncertainty envelope, not a one-sigma
   // measurement. Convert it back through the same configured probability
   // quantile before handing it to the estimator. Feeding the envelope in
   // directly creates a self-amplifying retry loop (sigma -> radius -> sigma)
   // and eventually allows far-away repeated-map aliases to pass the gate.
   const double recovery_quantile = supervisor_.global_recovery_quantile();
   if (std::isfinite(recovery_quantile) && recovery_quantile > 0.0) {
     estimator.note_absolute_search_failure(
         global_search_radius_m_ / recovery_quantile,
         global_search_heading_radius_rad_ / recovery_quantile,
         correction_stamp_ns);
   }
 }
 global_search_radius_m_ = 0.0;
 global_search_heading_radius_rad_ = 0.0;
 supervisor_.record_global(accept_global, ScanMatchResult{});
 if (accept_global) {
 action = std::string("global_relocalization:accepted score=") +
 std::to_string(global.score) +
 " overlap=" + std::to_string(global.overlap) +
 " margin=" + std::to_string(global.score_margin) +
 " returns=" + std::to_string(global.usable_returns);
 }
 if (!accept_global && !global.rejection_reason.empty()) {
 action = std::string("global_relocalization:") + global.rejection_reason +
 " score=" + std::to_string(global.score) +
 " overlap=" + std::to_string(global.overlap) +
 " margin=" + std::to_string(global.score_margin) +
 " returns=" + std::to_string(global.usable_returns) +
 " coarse=" + std::to_string(global.coarse_evaluations) +
 " refined=" + std::to_string(global.refined_poses);
 }
 if (!accept_global && !supervisor_.localized() &&
 (!global.runner_ups.empty() || require_temporal_confirmation)) {
startup_hyps_.seed(global, correction_stamp_ns,
 require_temporal_confirmation);
 action = require_temporal_confirmation ? "startup-confirming"
 : "startup-seeded";
    }
 return accept_global;
  }
 // Search only the correction the supervisor could actually accept. At low
 // covariance this removes hypotheses outside the innovation gate; near the
 // loss threshold the window expands up to the matcher's normal cap. The
 // gate and search therefore share one uncertainty doctrine.
 local_match_attempted = true;
 try_local_correction(map, obs, now_ns, estimator, action);
 return false;
}

bool LocalizationSystem::try_local_correction(
 const iface::MapSnapshot& map, const iface::ObservationFrame& obs,
 core::MonotonicNs now_ns, IEstimator& estimator, std::string& action,
 bool* full_observation_correction) {
 if (full_observation_correction != nullptr) {
 *full_observation_correction = false;
 }
 const EstimatedState& est = estimator.estimate();
 ScanMatcherConfig local_config;
 const auto& supervisor_config = supervisor_.config();
 const double translation_cap = local_config.search_translation_m;
 const double heading_cap = local_config.search_heading_rad;
 local_config.search_translation_m = std::clamp(
 supervisor_config.local_innovation_sigma_factor *
 est.position_sigma_m,
 std::min(supervisor_config.local_innovation_floor_m, translation_cap),
 translation_cap);
 local_config.search_heading_rad = std::clamp(
 supervisor_config.local_innovation_sigma_factor *
 est.heading_sigma_rad,
 std::min(supervisor_config.local_innovation_floor_rad, heading_cap),
 heading_cap);
 // Searching finer than half a survey cell repeats identical raster evidence
 // under slightly different floating-point coordinates. Tie the lattice to
 // map resolution and keep a bounded beam subset; covariance still controls
 // the window and the independent innovation gate still owns acceptance.
 local_config.translation_step_m = std::max(
 local_config.translation_step_m, 0.5 * map.resolution_m);
 local_config.maximum_returns = 64;
 ScanMatcher local_matcher(local_config);
 local_matcher.set_lidar_mount(base_from_lidar_);
 ScanMatchResult match = local_matcher.localize(
      obs.ranges_m, obs.scan_angle_min_rad, obs.scan_angle_increment_rad,
      est.x_m, est.y_m, est.yaw_rad, map, now_ns, est.roll_rad,
      est.pitch_rad, est.z_m);
 const bool partial_observation =
 !match.accepted && match.partial_observation_valid &&
 match.rejection_reason == "map match is geometrically ambiguous";
 if (match.accepted || partial_observation) {
 const double innovation_m = std::hypot(
 match.x_m - est.x_m, match.y_m - est.y_m);
 const double innovation_rad = std::abs(
 core::wrap_angle(match.yaw_rad - est.yaw_rad));
 const double position_sigma = std::max(
 est.position_sigma_m, match.position_sigma_m);
 const double heading_sigma = std::max(
 est.heading_sigma_rad, match.heading_sigma_rad);
 const auto& cfg = supervisor_.config();
 const double position_limit = std::max(
 cfg.local_innovation_floor_m,
 cfg.local_innovation_sigma_factor * position_sigma);
 const double heading_limit = std::max(
 cfg.local_innovation_floor_rad,
 cfg.local_innovation_sigma_factor * heading_sigma);
 bool innovation_rejected =
 !std::isfinite(innovation_m) || !std::isfinite(innovation_rad);
 if (partial_observation && !innovation_rejected) {
 // A ridge supplies information only in its observable subspace. Applying
 // the full-pose Euclidean gate would reject harmless motion along the null
 // space, while skipping the gate lets a sequence of locally plausible
 // ridges walk the map pose across repeated walls. Gate the same innovation
 // by the matcher's measured information instead: delta' I delta is the
 // squared normalized residual in exactly the directions the scan claims to
 // observe. The supervisor's existing sigma factor is therefore also the
 // statistical acceptance radius; no distance threshold is invented here.
 const double delta[3] = {
 match.x_m - est.x_m,
 match.y_m - est.y_m,
 core::wrap_angle(match.yaw_rad - est.yaw_rad)};
 double normalized_residual2 = 0.0;
 for (int row = 0; row < 3; ++row) {
 for (int column = 0; column < 3; ++column) {
 normalized_residual2 += delta[row] *
 match.pose_information[static_cast<std::size_t>(row * 3 + column)] *
 delta[column];
 }
 }
 const double acceptance_radius2 =
 cfg.local_innovation_sigma_factor * cfg.local_innovation_sigma_factor;
 innovation_rejected = !std::isfinite(normalized_residual2) ||
 normalized_residual2 < 0.0 ||
 normalized_residual2 > acceptance_radius2;
 } else if (!partial_observation) {
 innovation_rejected = innovation_rejected ||
 innovation_m > position_limit || innovation_rad > heading_limit;
 }
 if (innovation_rejected) {
   // A rejected local match is weak negative evidence (see
   // LocalizationSupervisor::next): it neither moves the pose nor widens the
   // covariance. Wheel/IMU odometry stays well constrained in feature-poor
   // space, and global recovery may begin only when the fused covariance
   // itself crosses the loss envelope. Folding the residual back in would
   // turn that weak evidence into a recovery trigger. A rank-deficient ridge
   // leaves the residual along its unobservable axis arbitrary, and the local
   // search window grows with covariance, so repeated corridor scans ratchet
   // sigma past the envelope and let whole-map recovery promote an aliased
   // pose. Rejecting an already accepted global candidate or a promoted
   // recovery hypothesis is different: that evidence has passed the matcher's
   // and tracker's own gates, and it still widens the belief through
   // IEstimator::note_absolute_rejection.
 supervisor_.record_local(false, match);
 action = partial_observation
 ? "local_correction:partial innovation rejected"
 : "local_correction:innovation rejected";
 return false;
 }
 if (match.pose_information_valid) {
 estimator.correct_pose_information(
 match.x_m, match.y_m, match.yaw_rad, match.pose_information,
 match.position_sigma_m, match.heading_sigma_rad, obs.stamp_ns);
 } else {
 estimator.correct_pose(match.x_m, match.y_m, match.yaw_rad,
 match.position_sigma_m, match.heading_sigma_rad, obs.stamp_ns);
 }
 if (full_observation_correction != nullptr && !partial_observation) {
 *full_observation_correction = true;
 }
 if (partial_observation) {
 supervisor_.record_local(true, match);
 action = "local_correction:partial observable ridge";
 return true;
 }
  }
  supervisor_.record_local(match.accepted, match);
 if (!match.accepted) {
 action = std::string("local_correction:") + match.rejection_reason;
 }
 return match.accepted;
}

}  // namespace lost_signal::robot::autonomy
