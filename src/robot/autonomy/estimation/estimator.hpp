#pragma once

#include "robot/core/time.hpp"
#include "robot/iface/observation.hpp"

namespace lost_signal::robot::autonomy {

// ROBOT-owned state estimate. Produced only from LSRI observations.
// Never seeded or corrected by simulator truth.
struct EstimatedState {
  bool valid = false;
  core::MonotonicNs stamp_ns = core::kInvalidTime;
  double x_m = 0.0;
  double y_m = 0.0;
  double yaw_rad = 0.0;
  double forward_mps = 0.0;
  double yaw_rate_radps = 0.0;
};

// IEstimator: odometry/IMU prediction + scan/map correction.
// v1: DeadReckoningEstimator (prediction only, no map correction).
class IEstimator {
 public:
  virtual ~IEstimator() = default;
  virtual void reset() = 0;
  virtual void predict(const iface::ObservationFrame& obs) = 0;
  virtual void correct(const iface::ObservationFrame& obs) = 0;
  virtual const EstimatedState& estimate() const = 0;
};

// Dead-reckoning from measured twist only. No truth, no map correction.
class DeadReckoningEstimator : public IEstimator {
 public:
  void reset() override;
  void predict(const iface::ObservationFrame& obs) override;
  void correct(const iface::ObservationFrame& obs) override;
  const EstimatedState& estimate() const override { return estimate_; }

 private:
  EstimatedState estimate_{};
  bool has_prior_ = false;
  iface::ObservationFrame prior_{};
};

}  // namespace lost_signal::robot::autonomy
