#pragma once

#include <cstdint>

#include "robot/autonomy/estimation/estimator.hpp"
#include "robot/core/time.hpp"
#include "robot/iface/observation.hpp"

namespace lost_signal::robot::autonomy {

// Rolling local world model derived from the static map view + ray-cleared
// observations. Never persisted into the static map revision.
struct LocalMap {
  bool valid = false;
  double resolution_m = 0.1;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  double origin_x_m = 0.0;  // map-frame pose of cell (0,0)
  double origin_y_m = 0.0;
  core::MonotonicNs stamp_ns = core::kInvalidTime;
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

// v1: fixed-size window centered on the estimate, no obstacle content.
// Planners treat absent content as unknown -> conservative.
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
  }
  LocalMap local_map() const override { return map_; }

 private:
  LocalMap map_{};
};

}  // namespace lost_signal::robot::autonomy
