#include "autonomy/localization/state_estimator.hpp"

#include <algorithm>
#include <cmath>

namespace sim {

void EstimateHolder::reset() {
  has_estimate_ = false;
  last_ = StateEstimate{};
}

void EstimateHolder::push(const StateEstimate& estimate) {
  has_estimate_ = true;
  last_ = estimate;
}

bool EstimateHolder::has_estimate() const { return has_estimate_; }

PropagatedEstimate EstimateHolder::at(double now_s, double max_staleness_s,
                                      DropoutPolicy policy) const {
  PropagatedEstimate out;
  if (!has_estimate_) {
    return out;
  }
  if (!(max_staleness_s >= 0.0)) {
    return out;
  }
  const double age = now_s - last_.stamp_s;
  if (!(age >= 0.0)) {
    // Future-dated measurement against this clock: refuse, don't extrapolate
    // backwards. Clock discipline is the caller's; silence would be a lie.
    return out;
  }
  out.estimate = last_;
  if (age <= max_staleness_s) {
    out.status = EstimateStatus::kFresh;
    return out;
  }
  if (policy == DropoutPolicy::kFlagInvalid) {
    out.status = EstimateStatus::kInvalid;
    return out;
  }
  if (policy == DropoutPolicy::kPredictConstantVelocity) {
    const double dt = age;
    out.estimate.state[0] += out.estimate.state[3] * dt;
    out.estimate.state[1] += out.estimate.state[4] * dt;
    out.estimate.state[2] += out.estimate.state[5] * dt;
    // stamp_s stays at measurement time (honest age); covariance untouched
    // (no growth model — see header).
  }
  out.status = EstimateStatus::kPredictedStale;
  return out;
}

} // namespace sim
