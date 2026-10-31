#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <unordered_map>
#include <vector>

#include "robot/autonomy/estimation/estimator.hpp"
#include "robot/autonomy/mapping/terrain_math.hpp"
#include "robot/core/geometry.hpp"
#include "robot/core/time.hpp"
#include "robot/core/transform.hpp"
#include "robot/iface/observation.hpp"
#include "robot/iface/map.hpp"

namespace lost_signal::robot::autonomy {

// Anonymous dynamic track. No actor IDs, no simulator labels.
// Carries the fields prediction and planning consume: geometry for the
// current position, uncertainty envelope grown with forecast time, and a
// readiness flag separating estimator maturity from motion capability.
struct DynamicTrack {
  std::uint32_t id = 0;
  double x_m = 0.0;
  double y_m = 0.0;
  double vx_mps = 0.0;
  double vy_mps = 0.0;
  double age_s = 0.0;
  double radius_m = 0.45;
  // Radial position-prediction uncertainty, kept separate from physical
  // geometry so planners report physical and conservative clearance with one
  // unambiguous contract. Predictors grow this value with forecast time.
  double position_uncertainty_m = 0.0;
 // Acquisition time of the last measurement incorporated into this state.
 // Prediction advances from this time, rather than pretending that a
 // coasted track was observed at the current decision tick.
 core::MonotonicNs last_observed_ns = core::kInvalidTime;
 // Deterministic engineering envelopes learned by the tracker. They are
 // deliberately not probabilities: all emitted hypotheses remain safety
 // relevant until fresh measurements resolve them.
 double position_uncertainty_growth_mps = 0.0;
 double position_uncertainty_cap_m = 2.0;
 double velocity_uncertainty_mps = 0.0;
 core::Vec2 alternate_velocity{0.0, 0.0};
 bool alternate_velocity_valid = false;
 core::Vec2 min_observed{0.0, 0.0};
 core::Vec2 max_observed{0.0, 0.0};
 bool bounded_x = false;
 bool bounded_y = false;
 double endpoint_dwell_s = 0.0;
 double initial_dwell_remaining_s = 0.0;
  // True when perception has enough timestamp-distinct history for this
  // track to participate in a behavior decision. Local collision avoidance
  // still consumes unready tracks conservatively.
  bool prediction_ready = false;

  double occupied_radius_m() const {
    double r = radius_m < 0.0 ? 0.0 : radius_m;
    double u = position_uncertainty_m < 0.0 ? 0.0 : position_uncertainty_m;
    return r + u;
  }
};

struct PerceptionResult {
  bool scan_ok = false;  // false = unknown scene, never free space
  core::MonotonicNs stamp_ns = core::kInvalidTime;
  std::vector<DynamicTrack> tracks;
};

// IPerception: scan -> free-space evidence + anonymous tracks.
class IPerception {
 public:
  virtual ~IPerception() = default;
  virtual void reset() = 0;
  virtual PerceptionResult update(const iface::ObservationFrame& obs,
                                  const EstimatedState& est) = 0;
 // Dynamic scan history belongs to the continuous odom frame (REP-105),
 // while surveyed-map foreground classification belongs to map. The
 // default preserves compatibility for estimators that expose one pose;
 // implementations that keep temporal state should consume both.
 virtual PerceptionResult update(const iface::ObservationFrame& obs,
 const EstimatedState& map_estimate,
 const EstimatedState& odom_estimate) {
 (void)odom_estimate;
 return update(obs, map_estimate);
 }
  // Full lidar mount from the TF tree (base<-lidar). Default identity.
  // The coordinator is the only caller.
  virtual void set_lidar_mount(const core::RigidTransform& base_from_lidar) {
    (void)base_from_lidar;
  }
 // Optional ROBOT map prior used to raycast a static scan reference. Unknown
 // cells stay unknown; implementations must never treat them as free space.
 virtual void set_static_map(const iface::MapSnapshot& map) { (void)map; }
};

struct LidarPerceptionConfig {
  // Algorithm thresholds only. The sensor mount lives in the TF tree
  // (RobotDescription.links[]); the coordinator fans the planar x/y/yaw
  // out via set_lidar_mount(). No second copy here.
  // Foreground test: |measured - expected| beyond this is not static world.
  double foreground_threshold_m = 0.30;
  // Surveyed terrain returns at the sensor plane are ground evidence, not
  // dynamic objects.  This is the same contract used by the validator and
  // mapper; it prevents a continuous ramp face from becoming a moving track.
  double survey_ground_tolerance_m = kTerrainGroundToleranceM;
 // Maximum probability that propagated pose uncertainty alone is mistaken
 // for map-relative foreground. The corresponding 2-D Gaussian radial
 // quantile expands the map comparison gate; when localization is uncertain,
 // raw scan safety remains active but the tracker does not invent moving
 // actors from displaced static walls.
 double foreground_false_positive_probability = 0.01;
  // Clustering: adjacent foreground beams split past this range gap.
  double cluster_range_gap_m = 0.5;
  int min_beams_per_cluster = 2;
  // Association gate + track lifetime.
  double association_gate_m = 0.6;
  // Coast tracks across short lidar occlusions. The
  // predictor grows the explicit position uncertainty while coasting, so a
  // temporary dropout cannot make the local planner forget a crossing object
  // and accelerate into its last predicted corridor.
  int max_misses = 15;
  int min_updates_for_ready = 3;
 // Object extent estimate from the cluster's world-space bounding box.
 // One beam (or a tiny cluster) clamps up to the minimum; long walls clamp
 // down so they stay conservative without swallowing the whole corridor.
 double min_object_radius_m = 0.15;
 double max_object_radius_m = 0.9;
 // Mapless motion segmentation. Without a static reference every compact
 // lidar cluster is tracked in the estimated map frame, but it is published
 // as dynamic only after repeatable motion exceeds this gate. This keeps
 // walls out of the dynamic layer while allowing SLAM/unknown environments
 // to detect pedestrians and vehicles from sensor history alone.
 double min_dynamic_speed_mps = 0.18;
 int min_motion_updates = 3;
 // Accumulated below-speed observation time required to return a stopped
 // track to the rolling occupancy layer. A positive value is an explicit
 // sensor-profile override. Zero derives the hold from the velocity-fit
 // window and the time needed for a minimum-speed target to leave the
 // association gate. That keeps motion identity through ordinary stops
 // without baking a scan cadence or scenario dwell into the tracker.
 double static_reclassification_time_s = 0.0;
 double max_tracked_speed_mps = 4.0;
 // Alpha-beta correction and bounded uncertainty proxy. These describe the
 // sensor/tracker, never simulator actor properties.
 double position_filter_gain = 0.45;
 double velocity_filter_gain = 0.10;
 double position_uncertainty_floor_m = 0.04;
 double position_uncertainty_cap_m = 0.30;
 double uncertainty_growth_floor_mps = 0.05;
 double velocity_uncertainty_floor_mps = 0.08;
 // Regression window and minimum time span before motion classification.
 // Expressing maturity in sensor time keeps classification independent of
 // LiDAR cadence; min_motion_updates alone only rejects isolated samples.
 double velocity_fit_window_s = 1.0;
};

// Planar lidar foreground extraction + nearest-neighbor tracking.
// Static reference present (sensor field or ROBOT map raycast): beams
// disagreeing with it cluster into candidates; clusters associate to tracks.
// Static reference absent: compact clusters are associated across estimated
// map-frame scans and publish only after repeatable motion is confirmed.
// Raw safety coverage remains active during that warm-up and for unmatched
// returns (validator + safety monitor consume the scan directly).
class LidarPerception : public IPerception {
 public:
  explicit LidarPerception(LidarPerceptionConfig config = {});

  void reset() override;
  PerceptionResult update(const iface::ObservationFrame& obs,
                          const EstimatedState& est) override;
 PerceptionResult update(const iface::ObservationFrame& obs,
 const EstimatedState& map_estimate,
 const EstimatedState& odom_estimate) override;
  void set_lidar_mount(const core::RigidTransform& base_from_lidar) override {
    base_from_lidar_ = base_from_lidar;
  }
 void set_static_map(const iface::MapSnapshot& map) override {
 if (static_map_ != nullptr && static_map_->map_id == map.map_id &&
 static_map_->revision == map.revision) {
 return;
 }
 static_map_has_known_occupancy_ = map.has_known_occupancy();
 static_map_ = &map;
 }

 private:
  struct TrackState {
 struct MotionSample {
 double time_s = 0.0;
 core::Vec2 position;
 };
    DynamicTrack track;
    int updates = 0;
    int misses = 0;
 int moving_updates = 0;
 double stationary_evidence_s = 0.0;
 bool mapless_dynamic_confirmed = false;
 bool created_with_static_reference = false;
 core::Vec2 last_reliable_velocity{0.0, 0.0};
 bool reliable_velocity_valid = false;
 bool seen_positive_x = false;
 bool seen_negative_x = false;
 bool seen_positive_y = false;
 bool seen_negative_y = false;
    double first_seen_s = 0.0;
    double first_x_m = 0.0;
    double first_y_m = 0.0;
    double last_seen_s = 0.0;
 double position_error_ema_m = 0.04;
 double velocity_error_ema_mps = 0.08;
 std::deque<MotionSample> motion_samples;
  };
  LidarPerceptionConfig config_;
  // Full mount from the TF tree (set by the coordinator, default identity).
  core::RigidTransform base_from_lidar_{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0}};
 // IMapProvider owns immutable revisions; perception retains a view.
 const iface::MapSnapshot* static_map_ = nullptr;
 bool static_map_has_known_occupancy_ = false;
  std::unordered_map<std::uint32_t, TrackState> tracks_;
  std::uint32_t next_id_ = 1;
 // Last map<-odom correction used to express the internally retained
 // temporal history. When localization changes this edge, all retained
 // samples move rigidly with it; a map correction must never look like
 // object velocity.
 std::optional<core::RigidTransform> map_from_odom_;
};

}  // namespace lost_signal::robot::autonomy
