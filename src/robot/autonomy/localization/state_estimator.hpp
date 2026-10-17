#pragma once

#include <array>
#include <cstddef>
#include <limits>
#include <vector>

namespace sim {

// State estimator contract (design slice, no producers/consumers wired yet).
// The failure it prevents already happened once: SlipDetector was written
// before its inputs existed, and the upper-bound probe then showed the metric
// itself was wrong for steady slide. Contract first, consumers second —
// DWA/Behavior/Safety/Slip will read StateEstimate, never fuse themselves.
//
// State: planar full [x, y, theta, vx, vy, w]. vy is carried even though our
// chassis are non-holonomic: the estimator must not assume what slip breaks.
// Covariance is 6x6 row-major in the same order. Producers set it; this layer
// carries it and flags staleness, but applies NO growth model (inventing
// process noise here would be tuning disguised as math — growth belongs to
// the future filter slice, with identification data).
struct StateEstimate {
  static constexpr std::size_t kDim = 6;
  // Measurement timestamp (when the world looked like this), NOT publish time.
  double stamp_s{0.0};
  // Known sensor delay at stamp time (producer-set, seconds >= 0).
  double latency_s{0.0};
  std::array<double, kDim> state{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, kDim * kDim> covariance{0.0};
  // Per-source validity at stamp time. A consumer must degrade (never
  // silently substitute) when its required sources are false.
  bool wheel_valid{false};
  bool imu_valid{false};
  bool lidar_valid{false};
};

enum class EstimateStatus {
  kFresh,          // within max_staleness_s of now.
  kPredictedStale, // propagated past staleness by policy (covariance NOT
                   // grown: status, not numbers, carries the warning).
  kInvalid,        // unusable: future-dated stamp, empty holder, or policy refusal.
};

enum class DropoutPolicy {
  kHoldLast,              // freeze mean at last estimate.
  kPredictConstantVelocity, // advance x/y/theta along (vx, vy, w).
  kFlagInvalid,           // refuse to guess: report Invalid past staleness.
};

struct PropagatedEstimate {
  StateEstimate estimate;
  EstimateStatus status{EstimateStatus::kInvalid};
};

// Single choke point for estimate time management. Pure: holder state is
// explicit (last estimate), policy is an argument, nothing hidden.
// Coverage is the caller's duty like timed_gap: asking past the data is
// answered by policy, never by silent extrapolation.
class EstimateHolder {
 public:
  void reset();
  void push(const StateEstimate& estimate);
  bool has_estimate() const;
  PropagatedEstimate at(double now_s, double max_staleness_s, DropoutPolicy policy) const;

 private:
  bool has_estimate_{false};
  StateEstimate last_{};
};

} // namespace sim
