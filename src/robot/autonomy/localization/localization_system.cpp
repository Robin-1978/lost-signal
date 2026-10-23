#include "robot/autonomy/localization/localization_system.hpp"

namespace lost_signal::robot::autonomy {

void LocalizationSystem::reset() {
  supervisor_.reset(false);
  startup_hyps_.reset();
  slam_.reset();
  slam_origin_seeded_ = false;
}

void LocalizationSystem::set_lidar_mount(
    const core::RigidTransform& base_from_lidar) {
  scan_matcher_.set_lidar_mount(base_from_lidar);
  global_search_.set_lidar_mount(base_from_lidar);
  startup_hyps_.set_lidar_mount(base_from_lidar);
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
  run_localization(map, obs, now_ns, estimator, out.action);
  if (run_slam(obs, has_observation, estimator, mapper)) {
    out.replan_requested = true;
  }
  return out;
}

bool LocalizationSystem::run_slam(const iface::ObservationFrame& obs,
                                  bool has_observation, IEstimator& estimator,
                                  IMapper& mapper) {
  if (!has_observation || !obs.scan_valid) return false;
  slam_.ingest(estimator.estimate(), obs);
  SlamLoopResult loop = slam_.try_close_loops(estimator.estimate(), obs);
  if (!loop.closed) return false;
  // Accepted closure: snap the estimate to the optimized pose and rebuild
  // the rolling map from keyframes at their optimized poses. Past cells
  // fed at drifted poses are stale; replaying keyframe scans re-registers
  // them. The decision branch refreshes on the corrected estimate.
  estimator.seed_pose(loop.corrected_pose.position.x,
                     loop.corrected_pose.position.y,
                     loop.corrected_pose.heading_rad, 0.05, 0.02,
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
  return true;
}

void LocalizationSystem::run_localization(
    const iface::MapSnapshot& map, const iface::ObservationFrame& obs,
    core::MonotonicNs now_ns, IEstimator& estimator, std::string& action) {
  const double now_s = now_ns * 1e-9;
  const EstimatedState& est = estimator.estimate();
  LocalizationHealth health;
  health.valid = est.valid;
  health.position_sigma_m = est.position_sigma_m;
  health.heading_sigma_rad = est.heading_sigma_rad;
  health.confidence = est.confidence;
  health.age_s = est.valid ? now_s - est.stamp_ns * 1e-9 : 1e9;
  std::string reason;
  LocalizationAction act = supervisor_.next(
      now_s, est.x_m, est.y_m, est.yaw_rad, health, &reason);
  action = localization_action_name(act);
  // Deferred startup disambiguation: an ambiguous whole-map search seeds
  // hypotheses instead of guessing; later scans promote or drop them.
  // Runs while unlocalized regardless of the supervisor action (tracking
  // owns the unlocalized scans); the supervisor retry still backstops.
  if (supervisor_.localized()) startup_hyps_.reset();
  if (!supervisor_.localized() && !startup_hyps_.empty() &&
      obs.scan_valid && !obs.ranges_m.empty()) {
    StartupTrackResult tracked = startup_hyps_.update(
        obs.ranges_m, obs.scan_angle_min_rad, obs.scan_angle_increment_rad,
        map, now_ns);
    if (tracked.status == StartupTrackStatus::kPromoted) {
      estimator.seed_pose(tracked.x_m, tracked.y_m, tracked.yaw_rad, 0.05,
                          0.02, obs.stamp_ns);
      supervisor_.record_global(true, ScanMatchResult{});
      action = "startup-promoted";
      return;
    }
    if (tracked.status == StartupTrackStatus::kTracking) {
      action = "startup-tracking";
      return;
    }
    // kLost: fall through — the supervisor cadence (or this tick's
    // global branch) re-runs the whole-map search with fresh evidence.
  }
  if (act == LocalizationAction::kDefer) return;
  const bool attempted_global = act == LocalizationAction::kGlobalRelocalization;
  // Mapless startup: no static raster exists, so neither local matching nor
  // the whole-map search can constrain the pose. Seed the odometry origin
  // once (no teleport: the current estimate IS the origin) and let the SLAM
  // session own the drift it can observe. Latched for the session.
  if (attempted_global && !map.has_raster() && !slam_origin_seeded_) {
    const StartupDecision startup =
        select_startup_mode(false, /*has_prior_pose=*/false);
    estimator.seed_pose(est.x_m, est.y_m, est.yaw_rad, 0.1, 0.05,
                        obs.stamp_ns);
    supervisor_.reset(true);
    slam_origin_seeded_ = true;
    action = startup_mode_name(startup.mode);
    supervisor_.record_global(true, ScanMatchResult{});
    return;
  }
  if (attempted_global) {
    // Whole-map search: kidnap recovery and cold start. An accepted global
    // fix establishes the estimate and marks the supervisor localized; a
    // rejection is evidence only and never moves the estimate. An
    // AMBIGUOUS rejection with runner-ups seeds deferred tracking (the
    // next scans disambiguate) instead of guessing now.
    GlobalSearchResult global = global_search_.search(
        obs.ranges_m, obs.scan_angle_min_rad, obs.scan_angle_increment_rad,
        map, now_ns, est.roll_rad, est.pitch_rad, est.z_m);
    if (global.accepted) {
      if (!supervisor_.localized()) {
        // No trustworthy prior exists (unknown start / kidnapped): snap
        // like a deliberate initial pose instead of blending toward a
        // confident-but-wrong dead-reckoned origin.
        estimator.seed_pose(global.x_m, global.y_m, global.yaw_rad,
                            global.position_sigma_m,
                            global.heading_sigma_rad, obs.stamp_ns);
      } else {
        estimator.correct_pose(global.x_m, global.y_m, global.yaw_rad,
                               global.position_sigma_m,
                               global.heading_sigma_rad, obs.stamp_ns);
      }
    }
    supervisor_.record_global(global.accepted, ScanMatchResult{});
    if (!global.accepted && !global.runner_ups.empty() &&
        !supervisor_.localized()) {
      startup_hyps_.seed(global);
      action = "startup-seeded";
    }
    return;
  }
  ScanMatchResult match = scan_matcher_.localize(
      obs.ranges_m, obs.scan_angle_min_rad, obs.scan_angle_increment_rad,
      est.x_m, est.y_m, est.yaw_rad, map, now_ns, est.roll_rad,
      est.pitch_rad, est.z_m);
  if (match.accepted) {
    estimator.correct_pose(match.x_m, match.y_m, match.yaw_rad,
                           match.position_sigma_m, match.heading_sigma_rad,
                           obs.stamp_ns);
  }
  supervisor_.record_local(match.accepted, match);
}

}  // namespace lost_signal::robot::autonomy
