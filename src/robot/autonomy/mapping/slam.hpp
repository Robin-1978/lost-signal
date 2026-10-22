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
};

struct SlamKeyframe {
  std::uint64_t id = 0;
  double stamp_s = 0.0;
  Pose2 pose;  // corrected estimate at creation (map frame)
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

class SlamSession {
 public:
  explicit SlamSession(SlamConfig config = SlamConfig{});

  void reset();
  // Feed the post-localization estimate + latest scan. Adds keyframes on
  // motion thresholds and chains odometry constraints. Never matches.
  void ingest(const EstimatedState& est, const iface::ObservationFrame& obs);
  // Rate-limited loop-closure attempt + optimize. Returns the correction to
  // apply (caller feeds it through correct_pose + map rebuild when closed).
  // No static map required: candidates match against keyframe submaps.
  SlamLoopResult try_close_loops(const EstimatedState& est,
                                 const iface::ObservationFrame& obs);

  bool empty() const { return keyframes_.empty(); }
  std::size_t keyframe_count() const { return keyframes_.size(); }
  std::size_t loop_closure_count() const { return loop_closures_; }
  const std::vector<SlamKeyframe>& keyframes() const { return keyframes_; }
  const PoseGraphBackend& graph() const { return graph_; }
  const SlamLoopResult& last_result() const { return last_result_; }

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
  PoseGraphBackend graph_;
  std::vector<SlamKeyframe> keyframes_;
  std::uint64_t next_id_ = 1;
  std::size_t loop_closures_ = 0;
  double last_attempt_s_ = -1.0e9;
  std::size_t keyframes_at_last_attempt_ = 0;
  SlamLoopResult last_result_;
};

}  // namespace lost_signal::robot::autonomy
