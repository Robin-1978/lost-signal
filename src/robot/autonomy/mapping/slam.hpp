#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "robot/autonomy/estimation/estimator.hpp"
#include "robot/autonomy/localization/scan_matcher.hpp"
#include "robot/autonomy/mapping/loop_closure.hpp"
#include "robot/autonomy/mapping/pose_graph.hpp"
#include "robot/core/time.hpp"
#include "robot/iface/map.hpp"
#include "robot/iface/observation.hpp"

namespace lost_signal::robot::autonomy {

// Online SLAM session: keyframed pose graph with scan-match loop closures.
// Odometry and accepted loop constraints feed the Gauss-Newton backend;
// accepted closures correct the running estimate and trigger a local-map
// rebuild from keyframes at their optimized poses.
//
// The session never touches simulator truth and never publishes navigation
// maps: the graph is session data, and corrections flow through the same
// correct_pose path as scan-match localization (gated, variance-weighted).
struct SlamConfig {
  double keyframe_translation_m = 0.5;
  double keyframe_heading_rad = 0.30;
  std::size_t max_keyframes = 200;  // ring: oldest non-fixed node drops
  // Loop attempts run at most this often and only after a new keyframe.
  double loop_attempt_interval_s = 2.0;
  std::size_t max_candidates_per_attempt = 3;
  // Widened match window: loops close after drift the local tracker could
  // never span. Cost is bounded by beam downsampling + attempt rate.
  double match_search_translation_m = 2.0;
  double match_translation_step_m = 0.15;
  double match_search_heading_rad = 0.5;
  double match_heading_step_rad = 0.10;
  std::size_t match_max_beams = 48;
  double match_minimum_overlap = 0.30;
  LoopClosureGateConfig gate;
  // Incremental scan-to-submap tracking (the workhorse between loop
  // events): match the live scan against the most recent keyframe's
  // submap in a small window. Bounded (a few cells) so it can never
  // kidnap; gated by the same overlap/margin contract as the local
  // matcher, plus a reverse verification (old scan vs submap rendered at
  // the forward solution — the same two-sided check loop closures use).
  // Independent of supervisor trust: it bounds drift, it does not
  // establish the frame (loop closures still do that).
  double track_window_m = 0.30;
  double track_step_m = 0.05;
  double track_heading_rad = 0.20;
  double track_heading_step_rad = 0.05;
  std::size_t track_max_beams = 64;
  double track_min_overlap = 0.30;
  double track_min_margin = 0.02;
  double track_interval_s = 0.2;
  double track_min_translation_m = 0.05;
  double track_min_heading_rad = 0.02;
};

struct SlamKeyframe {
  std::uint64_t id = 0;
  double stamp_s = 0.0;
  Pose2 pose;  // SE(2) graph pose (x, y, heading); the graph stays planar
  // Slope attitude at creation (rendering only): submap rasterization and
  // matcher scoring place endpoints through the full 3D sensor pose.
  double roll_rad = 0.0;
  double pitch_rad = 0.0;
  double z_m = 0.0;
  std::vector<float> ranges_m;
  std::vector<float> static_ranges_m;
  double angle_min_rad = 0.0;
  double angle_increment_rad = 0.0;
};

struct SlamLoopResult {
  bool closed = false;
  std::uint64_t from_id = 0;
  std::uint64_t to_id = 0;
  // Optimized pose of the newest keyframe (diagnostics / map rebuild).
  Pose2 corrected_pose;
  // Correction delta (optimized newest ⊖ prior newest): compose onto the
  // live estimate, which may have moved past the last keyframe.
  Pose2 correction_delta;
  double correction_m = 0.0;
  double correction_rad = 0.0;
  std::string diagnostic = "no attempt";
};

// Incremental tracking outcome: scan-to-most-recent-submap match. Same
// evidence contract as loop closure (overlap + margin gates), but the
// window is small and the chain frame is the drifted keyframe chain, so
// a correction bounds incremental drift without establishing the frame.
struct SlamTrackResult {
  bool attempted = false;
  bool accepted = false;
  double x_m = 0.0;
  double y_m = 0.0;
  double yaw_rad = 0.0;
  double position_sigma_m = 0.0;
  double heading_sigma_rad = 0.0;
  double score = 0.0;
  double overlap = 0.0;
  double score_margin = 0.0;
  std::string diagnostic = "no attempt";
};

class SlamSession {
 public:
  explicit SlamSession(SlamConfig config = SlamConfig{});

  // Full lidar mount from the TF tree (coordinator fans out).
  void set_lidar_mount(const core::RigidTransform& base_from_lidar) {
    base_from_lidar_ = base_from_lidar;
  }
  void reset();
  // Feed the post-localization estimate + latest scan. Adds keyframes on
  // motion thresholds and chains odometry constraints. Never matches.
  void ingest(const EstimatedState& est, const iface::ObservationFrame& obs);
  // Rate-limited loop-closure attempt + optimize. Returns the correction to
  // apply (caller feeds it through correct_pose + map rebuild when closed).
  // No static map required: candidates match against keyframe submaps.
  SlamLoopResult try_close_loops(const EstimatedState& est,
                                 const iface::ObservationFrame& obs);
  // Incremental tracking: match the live scan against the most recent
  // keyframe's submap in a small window. Rate-limited by interval +
  // movement. The caller applies accepted corrections through correct_pose;
  // the supervisor trust state is untouched (tracking bounds drift, loop
  // closures establish the frame).
  SlamTrackResult track(const EstimatedState& est,
                        const iface::ObservationFrame& obs);

  bool empty() const { return keyframes_.empty(); }
  std::size_t keyframe_count() const { return keyframes_.size(); }
  std::size_t loop_closure_count() const { return loop_closures_; }
  std::size_t track_corrections() const { return track_corrections_; }
  const std::vector<SlamKeyframe>& keyframes() const { return keyframes_; }
 // Render one stored keyframe as a static submap (diagnostics/tests).
 // Dynamic returns are excluded when the keyframe carried a static
 // reference; an empty snapshot means no such keyframe.
 iface::MapSnapshot submap(std::uint64_t keyframe_id) const;
  const PoseGraphBackend& graph() const { return graph_; }
  const SlamLoopResult& last_result() const { return last_result_; }
  const SlamTrackResult& last_track_result() const { return last_track_; }

 private:
  // Render one keyframe's scan into a small occupancy raster in the map
  // frame (endpoints occupied, ray interiors free) for scan-to-submap match.
  iface::MapSnapshot submap_for(const SlamKeyframe& kf) const;
  bool keyframe_due(const EstimatedState& est) const;
  // Rebuild the backend from retained keyframes (ring drop). Chain links
  // are re-derived from keyframe poses; loop constraints do not survive a
  // drop of either endpoint (documented: the ring bounds memory, old loops
  // age out with their keyframes).
  void rebuild_graph();

  SlamConfig config_;
  core::RigidTransform base_from_lidar_{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0}};
  PoseGraphBackend graph_;
  std::vector<SlamKeyframe> keyframes_;
  std::uint64_t next_id_ = 1;
  std::size_t loop_closures_ = 0;
  double last_attempt_s_ = -1.0e9;
  std::size_t keyframes_at_last_attempt_ = 0;
  SlamLoopResult last_result_;
  // Incremental tracking state: submap cache keys on the newest keyframe
  // id (render once per keyframe, not once per tick); attempt pose/time
  // gate the cadence. All ROBOT-side: scans, submaps, odometry prior.
  std::size_t track_corrections_ = 0;
  double last_track_s_ = -1.0e9;
  double last_track_x_m_ = 0.0;
  double last_track_y_m_ = 0.0;
  double last_track_yaw_rad_ = 0.0;
  bool have_tracked_ = false;
  SlamTrackResult last_track_;
  mutable iface::MapSnapshot track_submap_;
  mutable std::uint64_t track_submap_id_ = 0;
};

}  // namespace lost_signal::robot::autonomy
