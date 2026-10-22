#pragma once

#include <cstdint>
#include <vector>

#include "robot/autonomy/estimation/estimator.hpp"
#include "robot/core/time.hpp"
#include "robot/iface/observation.hpp"

namespace lost_signal::robot::autonomy {

// Anonymous dynamic track. No actor IDs, no simulator labels.
struct DynamicTrack {
  std::uint32_t id = 0;
  double x_m = 0.0;
  double y_m = 0.0;
  double vx_mps = 0.0;
  double vy_mps = 0.0;
  double age_s = 0.0;
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
};

// v1: reports scan presence only, no tracks. Safe: unknown, never free.
class NullPerception : public IPerception {
 public:
  void reset() override { last_ = PerceptionResult{}; }
  PerceptionResult update(const iface::ObservationFrame& obs,
                          const EstimatedState& /*est*/) override {
    last_.scan_ok = obs.scan_valid;
    last_.stamp_ns = obs.stamp_ns;
    last_.tracks.clear();
    return last_;
  }

 private:
  PerceptionResult last_{};
};

}  // namespace lost_signal::robot::autonomy
