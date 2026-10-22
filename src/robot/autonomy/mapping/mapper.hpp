#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "robot/autonomy/estimation/estimator.hpp"
#include "robot/core/geometry.hpp"
#include "robot/core/geometry.hpp"
#include "robot/core/time.hpp"
#include "robot/iface/observation.hpp"

namespace lost_signal::robot::autonomy {

// Rolling local world model derived from the sensor stream. Occupancy is
// -1 unknown, 0 free, 100 occupied (row-major, x fastest). Unknown is
// optimistic for sampling (the planner proposes) and conservative for safety
// (the validator + safety monitor veto on raw scan evidence) — the split is
// deliberate: one layer proposes, independent layers dispose.
struct LocalMap {
  bool valid = false;
  double resolution_m = 0.1;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  double origin_x_m = 0.0;  // map-frame pose of cell (0,0)
  double origin_y_m = 0.0;
  core::MonotonicNs stamp_ns = core::kInvalidTime;
  std::vector<int8_t> occupancy;  // -1 unknown; size width*height when valid

  bool inside(int gx, int gy) const {
    return gx >= 0 && gy >= 0 && gx < static_cast<int>(width) &&
           gy < static_cast<int>(height);
  }
  // Free means observed-free. Unknown is NOT free here; callers choose the
  // policy (planner sampling treats unknown as traversable, the safety
  // layers never do).
  bool is_free(double wx, double wy) const;
  // Blocked means observed-occupied only. The planner's optimistic
  // counterpart to is_free: unknown and out-of-window read as traversable
  // so sampling works on partially observed maps. Safety layers must never
  // use this; they read raw scan evidence instead.
  bool is_blocked(double wx, double wy) const;
  // Continuous line-of-travel query, sampled at <= half a cell so a segment
  // cannot jump a lethal cell. Admission and validation share this
  // primitive so they cannot disagree on static traversability.
  // Conservative: unknown counts as blocked.
  bool segment_is_free(double x0, double y0, double x1, double y1) const;
  // Optimistic twin: only observed-occupied cells block. For planner
  // sampling on partially observed maps; safety layers never use this.
  bool segment_is_traversable(double x0, double y0, double x1,
                              double y1) const;
  // Exact world-space distance to the occupied set. Occupied cells own
  // their full square; outside the window counts as occupied. Zero inside
  // occupied space; continuous in free space, suitable as a planner critic.
  double clearance(double wx, double wy) const;
  // Inflate occupied cells by radius into a binary costmap. Transient
  // observation layers ignore the boundary (the base map owns that policy;
  // an empty sensor layer must contribute no obstacles).
  LocalMap inflated(double radius_m) const;
};

// IMapper: observations -> rolling local map around the estimate.
class IMapper {
 public:
  virtual ~IMapper() = default;
  virtual void reset() = 0;
  virtual void update(const iface::ObservationFrame& obs,
                      const EstimatedState& est) = 0;
  virtual LocalMap local_map() const = 0;
};

// v1: fixed-size window centered on the estimate, all cells unknown.
// A ray-clearing mapper replaces this body; the query semantics above stay.
class RayClearingMapper : public IMapper {
 public:
  void reset() override { map_ = LocalMap{}; }
  void update(const iface::ObservationFrame& obs,
              const EstimatedState& est) override {
    if (!est.valid) {
      map_.valid = false;
      return;
    }
    // Rolling window: recenter (dropping old content) when the estimate
    // leaves the inner half. A mapping session with loop closure replaces
    // this reset with submap re-rasterization; see docs.
    const double half_w = map_.width * map_.resolution_m * 0.5;
    const double half_h = map_.height * map_.resolution_m * 0.5;
    bool need_window = !map_.valid || map_.occupancy.size() !=
                                            map_.width * map_.height;
    if (!need_window) {
      const double cx = map_.origin_x_m + half_w;
      const double cyy = map_.origin_y_m + half_h;
      need_window = std::abs(est.x_m - cx) > half_w * 0.5 ||
                    std::abs(est.y_m - cyy) > half_h * 0.5;
    }
    if (need_window) {
      map_.valid = true;
      map_.resolution_m = 0.1;
      map_.width = 100;
      map_.height = 100;
      map_.origin_x_m = est.x_m - 5.0;
      map_.origin_y_m = est.y_m - 5.0;
      map_.occupancy.assign(map_.width * map_.height, -1);
    }
    map_.stamp_ns = obs.stamp_ns;
    if (!obs.scan_valid || obs.ranges_m.empty()) return;
    // Ray clearing from the estimate pose (v1: lidar ~ base center; the
    // mount offset enters with calibrated extrinsics, see perception).
    // Background beams paint endpoint cells occupied; foreground beams
    // clear the ray but never paint (actors must not become static map).
    const std::size_t n = obs.ranges_m.size();
    const bool has_ref =
        obs.scan_static_valid && obs.static_ranges_m.size() == n;
    for (std::size_t i = 0; i < n; ++i) {
      const float range = obs.ranges_m[i];
      if (!(range > 0.0) || !std::isfinite(range)) continue;
      const double angle = -core::kPi + 2.0 * core::kPi * i / n + est.yaw_rad;
      const double ex = est.x_m + range * std::cos(angle);
      const double ey = est.y_m + range * std::sin(angle);
      clear_segment(est.x_m, est.y_m, ex, ey);
      bool background = false;
      if (has_ref) {
        background =
            std::abs(range - obs.static_ranges_m[i]) <= 0.30;
      }
      if (background) occupy(ex, ey);
    }
  }
  LocalMap local_map() const override { return map_; }

 private:
  void clear_segment(double x0, double y0, double x1, double y1) {
    const double length = std::hypot(x1 - x0, y1 - y0);
    const int steps = std::max(
        1, static_cast<int>(
               std::ceil(length / (0.5 * map_.resolution_m))));
    for (int s = 0; s < steps; ++s) {  // exclude endpoint: actor cells stay
      const double b = static_cast<double>(s) / steps;
      set_cell(x0 + (x1 - x0) * b, y0 + (y1 - y0) * b, 0);
    }
  }
  void occupy(double wx, double wy) {
    int gx, gy;
    world_to_grid(wx, wy, gx, gy);
    if (map_.inside(gx, gy)) {
      map_.occupancy[gy * map_.width + gx] = 100;
    }
  }
  void set_cell(double wx, double wy, int8_t v) {
    int gx, gy;
    world_to_grid(wx, wy, gx, gy);
    if (map_.inside(gx, gy)) {
      map_.occupancy[gy * map_.width + gx] = v;
    }
  }
  void world_to_grid(double wx, double wy, int& gx, int& gy) const {
    gx = static_cast<int>(std::floor((wx - map_.origin_x_m) / map_.resolution_m));
    gy = static_cast<int>(std::floor((wy - map_.origin_y_m) / map_.resolution_m));
  }

  LocalMap map_{};
};

}  // namespace lost_signal::robot::autonomy
