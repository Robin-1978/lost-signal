#pragma once

#include <optional>
#include <vector>

#include "robot/core/transform.hpp"
#include "robot/iface/observation.hpp"

namespace lost_signal::robot::autonomy {

// Result of accepting one transport envelope. Estimation events contain
// only newly acquired motion/IMU sections and are ordered by acquisition
// time. A scan event is emitted once for perception/mapping.
struct MeasurementBatch {
  std::vector<iface::ObservationFrame> estimator_events;
  std::optional<iface::ObservationFrame> scan_event;
  bool accepted = false;
};

// ROBOT-side acquisition cache for asynchronous sensor endpoints.
//
// ObservationFrame is only a transport envelope: each section retains its
// own source, sequence and acquisition time. This class owns normalization,
// per-section ordering, IMU frame conversion, latest-value caching and the
// one-shot views consumed by localization and online adaptation. It never
// consults simulator truth.
class MeasurementSynchronizer {
 public:
  void reset();

  MeasurementBatch ingest(const iface::ObservationFrame& observation,
                          const core::FrameTransformTree& transforms);

  bool has_observation() const { return has_observation_; }
  bool has_scan() const { return have_scan_; }
  bool has_motion() const { return have_motion_; }
  bool has_imu() const { return have_imu_; }

  const iface::ObservationFrame& latest() const { return latest_; }

  // Returns the cached multi-rate view. A newly accepted scan is exposed
  // once; later calls remove all scan evidence so localization/SLAM cannot
  // process the same acquisition again.
  iface::ObservationFrame take_localization_observation();

  // Returns only motion/IMU sections accepted since the previous call.
  std::optional<iface::ObservationFrame> take_adaptation_observation();

 private:
  bool has_observation_ = false;
  bool have_scan_ = false;
  bool have_motion_ = false;
  bool have_imu_ = false;
  bool localization_scan_pending_ = false;
  bool adaptation_pending_ = false;
  iface::ObservationFrame latest_;
  iface::ObservationFrame adaptation_;
};

}  // namespace lost_signal::robot::autonomy
