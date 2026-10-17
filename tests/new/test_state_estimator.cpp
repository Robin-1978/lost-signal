// StateEstimator contract tests: time management, dropout policies, and the
// honesty boundaries (future stamps refused, staleness flagged not hidden,
// covariance never grown here). No producers/consumers exist yet by design.

#include "robot/autonomy/localization/state_estimator.hpp"

#include <cmath>
#include <iostream>
#include <limits>

#define CHECK(condition)                                                                                     \
 do {                                                                                                        \
  if (!(condition)) {                                                                                        \
   std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n";                           \
   return 1;                                                                                                 \
  }                                                                                                          \
 } while (false)

namespace {

sim::StateEstimate make_estimate(double stamp) {
 sim::StateEstimate estimate;
 estimate.stamp_s = stamp;
 estimate.latency_s = 0.05;
 estimate.state = {1.0, 2.0, 0.5, 0.8, 0.1, -0.2};
 for (std::size_t i = 0; i < 36; ++i) {
  estimate.covariance[i] = i == 0 || i == 7 || i == 14 ? 0.01 : 0.0;
 }
 estimate.wheel_valid = true;
 estimate.imu_valid = true;
 return estimate;
}

} // namespace

int main() {
 // Empty holder is Invalid under every policy.
 {
  sim::EstimateHolder holder;
  CHECK(!holder.has_estimate());
  for (auto policy : {sim::DropoutPolicy::kHoldLast, sim::DropoutPolicy::kPredictConstantVelocity,
                      sim::DropoutPolicy::kFlagInvalid}) {
   CHECK(holder.at(10.0, 1.0, policy).status == sim::EstimateStatus::kInvalid);
  }
 }
 // Fresh boundary: age == max is fresh, past it is policy.
 {
  sim::EstimateHolder holder;
  holder.push(make_estimate(10.0));
  CHECK(holder.at(11.0, 1.0, sim::DropoutPolicy::kHoldLast).status == sim::EstimateStatus::kFresh);
  CHECK(holder.at(11.0 + 1e-9, 1.0, sim::DropoutPolicy::kHoldLast).status ==
        sim::EstimateStatus::kPredictedStale);
  CHECK(holder.at(12.0, 1.0, sim::DropoutPolicy::kFlagInvalid).status ==
        sim::EstimateStatus::kInvalid);
  CHECK(holder.at(11.0, 1.0, sim::DropoutPolicy::kFlagInvalid).status == sim::EstimateStatus::kFresh);
  CHECK(holder.at(11.0, -1.0, sim::DropoutPolicy::kHoldLast).status == sim::EstimateStatus::kInvalid);
 }
 // Future-dated stamp against this clock: refused, never back-extrapolated.
 {
  sim::EstimateHolder holder;
  holder.push(make_estimate(10.0));
  CHECK(holder.at(9.9, 5.0, sim::DropoutPolicy::kHoldLast).status == sim::EstimateStatus::kInvalid);
 }
 // HoldLast freezes the mean; stamp and covariance pass through untouched.
 {
  sim::EstimateHolder holder;
  holder.push(make_estimate(10.0));
  const auto got = holder.at(13.0, 1.0, sim::DropoutPolicy::kHoldLast);
  CHECK(got.status == sim::EstimateStatus::kPredictedStale);
  CHECK(got.estimate.state[0] == 1.0 && got.estimate.state[1] == 2.0);
  CHECK(got.estimate.stamp_s == 10.0);
  CHECK(got.estimate.covariance[0] == 0.01 && got.estimate.covariance[1] == 0.0);
  CHECK(got.estimate.wheel_valid && got.estimate.imu_valid && !got.estimate.lidar_valid);
 }
 // Constant-velocity advances x/y/theta along (vx, vy, w) — vy carried even
 // though our chassis are non-holonomic (the estimator must not assume what
 // slip breaks). Covariance still untouched (no growth model here).
 {
  sim::EstimateHolder holder;
  holder.push(make_estimate(10.0));
  const auto got = holder.at(12.0, 1.0, sim::DropoutPolicy::kPredictConstantVelocity);
  CHECK(got.status == sim::EstimateStatus::kPredictedStale);
  CHECK(std::abs(got.estimate.state[0] - (1.0 + 0.8 * 2.0)) < 1e-9);
  CHECK(std::abs(got.estimate.state[1] - (2.0 + 0.1 * 2.0)) < 1e-9);
  CHECK(std::abs(got.estimate.state[2] - (0.5 - 0.2 * 2.0)) < 1e-9);
  CHECK(got.estimate.state[3] == 0.8 && got.estimate.state[5] == -0.2);
  CHECK(got.estimate.covariance[0] == 0.01);
  CHECK(got.estimate.stamp_s == 10.0);
 }
 // reset() returns to empty.
 {
  sim::EstimateHolder holder;
  holder.push(make_estimate(10.0));
  holder.reset();
  CHECK(!holder.has_estimate());
  CHECK(holder.at(10.5, 1.0, sim::DropoutPolicy::kHoldLast).status == sim::EstimateStatus::kInvalid);
 }
 std::cerr << "state estimator contract green\n";
 return 0;
}
