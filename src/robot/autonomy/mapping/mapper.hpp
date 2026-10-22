#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "robot/autonomy/estimation/estimator.hpp"
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
class NullMapper : public IMapper {
 public:
  void reset() override { map_ = LocalMap{}; }
  void update(const iface::ObservationFrame& obs,
              const EstimatedState& est) override {
    map_.valid = est.valid;
    map_.width = 100;
    map_.height = 100;
    map_.origin_x_m = est.x_m - 5.0;
    map_.origin_y_m = est.y_m - 5.0;
    map_.stamp_ns = obs.stamp_ns;
    map_.occupancy.assign(map_.width * map_.height, -1);
  }
  LocalMap local_map() const override { return map_; }

 private:
  LocalMap map_{};
};

}  // namespace lost_signal::robot::autonomy
