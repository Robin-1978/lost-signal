#pragma once

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include "robot/autonomy/estimation/estimator.hpp"
#include "robot/core/time.hpp"
#include "robot/iface/observation.hpp"

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
  // Planar lidar mount from the TF tree (base<-lidar x/y/yaw). Default
  // identity = aligned mount. The coordinator is the only caller.
  virtual void set_lidar_mount(double x_m, double y_m, double yaw_rad) {
    (void)x_m;
    (void)y_m;
    (void)yaw_rad;
  }
};

struct LidarPerceptionConfig {
  // Algorithm thresholds only. The sensor mount lives in the TF tree
  // (RobotDescription.links[]); the coordinator fans the planar x/y/yaw
  // out via set_lidar_mount(). No second copy here.
  // Foreground test: |measured - expected| beyond this is not static world.
  double foreground_threshold_m = 0.30;
  // Clustering: adjacent foreground beams split past this range gap.
  double cluster_range_gap_m = 0.5;
  int min_beams_per_cluster = 2;
  // Association gate + track lifetime.
  double association_gate_m = 0.6;
  int max_misses = 3;
  int min_updates_for_ready = 3;
};

// Planar lidar foreground extraction + nearest-neighbor tracking.
// Static reference present: beams disagreeing with it cluster into
// candidates; clusters associate to tracks by distance gate.
// Static reference absent: classification is unavailable -> no tracks, but
// scan_ok stays true so raw safety coverage is retained (validator + safety
// monitor consume the scan directly and never need tracks).
class LidarPerception : public IPerception {
 public:
  explicit LidarPerception(LidarPerceptionConfig config = {});

  void reset() override;
  PerceptionResult update(const iface::ObservationFrame& obs,
                          const EstimatedState& est) override;
  void set_lidar_mount(double x_m, double y_m, double yaw_rad) override {
    mount_x_m_ = x_m;
    mount_y_m_ = y_m;
    mount_yaw_rad_ = yaw_rad;
  }

 private:
  struct TrackState {
    DynamicTrack track;
    int updates = 0;
    int misses = 0;
    double first_seen_s = 0.0;
    double first_x_m = 0.0;
    double first_y_m = 0.0;
    double last_seen_s = 0.0;
  };
  LidarPerceptionConfig config_;
  // Planar mount from the TF tree (set by the coordinator, default 0).
  double mount_x_m_ = 0.0;
  double mount_y_m_ = 0.0;
  double mount_yaw_rad_ = 0.0;
  std::unordered_map<std::uint32_t, TrackState> tracks_;
  std::uint32_t next_id_ = 1;
};

}  // namespace lost_signal::robot::autonomy
