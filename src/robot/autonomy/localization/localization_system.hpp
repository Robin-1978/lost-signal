#pragma once

// Localization subsystem: supervisor-sequenced scan corrections + the SLAM
// session over one estimate. This unit owns the orchestration the
// coordinator must not: which matcher runs when (defer/local/global),
// seed-vs-blend correction decisions, startup modes (seeded, global,
// mapless, deferred multi-hypothesis tracking), and loop-closure
// correction + keyframe replay. Thresholds stay in the owning modules
// (supervisor cadence, matcher margins, tracker rules, slam thresholds);
// this unit only sequences them. The coordinator wires estimator, mapper
// and observations in, and consumes the action name + replan flag out.

#include <string>

#include "robot/autonomy/estimation/estimator.hpp"
#include "robot/autonomy/localization/global_search.hpp"
#include "robot/autonomy/localization/scan_matcher.hpp"
#include "robot/autonomy/localization/startup_selector.hpp"
#include "robot/autonomy/localization/startup_tracker.hpp"
#include "robot/autonomy/localization/supervisor.hpp"
#include "robot/autonomy/mapping/mapper.hpp"
#include "robot/autonomy/mapping/slam.hpp"
#include "robot/core/time.hpp"
#include "robot/iface/map.hpp"
#include "robot/iface/observation.hpp"

namespace lost_signal::robot::autonomy {

class LocalizationSystem {
 public:
  void reset();
  void set_lidar_mount(const core::RigidTransform& base_from_lidar);
  // Startup mode passthrough (see supervisor): Mapless disables the global
  // branch for missions with no surveyed map. Reset never changes the mode
  // (configuration, not bringup state).
  void set_startup_mode(StartupMode mode) {
    supervisor_.set_startup_mode(mode);
  }
  // Operator seeding (dock initialpose): the estimate is trustworthy by
  // declaration — mark localized and drop stale startup hypotheses.
  void mark_localized() {
    supervisor_.reset(true);
    startup_hyps_.reset();
  }

  struct TickResult {
    std::string action = "init";
    bool replan_requested = false;
  };
  // One decision tick over the estimate. Corrections feed the estimator
  // only when accepted, so a hard scan never drags the pose.
  TickResult update(const iface::MapSnapshot& map,
                    const iface::ObservationFrame& obs, core::MonotonicNs now_ns,
                    bool has_observation, IEstimator& estimator,
                    IMapper& mapper);

  bool localized() const;
  // SLAM observability (diagnostics/tests fan out, no policy here).
  std::size_t slam_loops_closed() const;
  const SlamLoopResult& last_slam_result() const;
  std::size_t slam_keyframes() const;
  // Incremental tracking observability (same discipline, no policy).
  std::size_t slam_tracks() const { return slam_.track_corrections(); }
  const SlamTrackResult& last_track_result() const {
    return slam_.last_track_result();
  }

 private:
  void run_localization(const iface::MapSnapshot& map,
                        const iface::ObservationFrame& obs,
                        core::MonotonicNs now_ns, IEstimator& estimator,
                        std::string& action);
  // Bounded scan-to-submap tracking ahead of supervised corrections.
  void run_tracking(const iface::ObservationFrame& obs, bool has_observation,
                    IEstimator& estimator);
  // Returns true when a loop closure rebuilt the map (caller replans).
  bool run_slam(const iface::ObservationFrame& obs, bool has_observation,
                IEstimator& estimator, IMapper& mapper);

  LocalizationSupervisor supervisor_;
  ScanMatcher scan_matcher_;
  GlobalSearcher global_search_;
  StartupHypotheses startup_hyps_;
  SlamSession slam_;
  bool slam_origin_seeded_ = false;
};

}  // namespace lost_signal::robot::autonomy
