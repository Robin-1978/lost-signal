#include "autonomy/planning/local/pocket_shadow.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace sim {

namespace {

// Content-derived reporting bands (level score bars: 0.2 m clearance,
// 1.0 m arrive tolerance). Observation bins, never release gates.
constexpr double kReportMaxDevM = 0.20;
constexpr double kReportEndpointM = 1.0;

} // namespace

TrajectoryError compare_trajectories(const std::vector<TimedPose>& predicted,
                                     const std::vector<TimedPose>& actual) {
  TrajectoryError out;
  if (predicted.empty() || actual.empty()) {
    out.max_dev_m = std::numeric_limits<double>::infinity();
    out.rmse_m = std::numeric_limits<double>::infinity();
    out.endpoint_err_m = std::numeric_limits<double>::infinity();
    out.time_bias_s = std::numeric_limits<double>::infinity();
    return out;
  }
  double sum_sq = 0.0;
  for (const auto& p : predicted) {
    double best_dt = std::numeric_limits<double>::infinity();
    double best_d = std::numeric_limits<double>::infinity();
    for (const auto& a : actual) {
      const double dt = std::abs(a.t_s - p.t_s);
      if (dt < best_dt) {
        best_dt = dt;
        best_d = std::hypot(a.p.x - p.p.x, a.p.y - p.p.y);
      }
    }
    out.max_dev_m = std::max(out.max_dev_m, best_d);
    sum_sq += best_d * best_d;
  }
  out.rmse_m = std::sqrt(sum_sq / static_cast<double>(predicted.size()));
  const Vec2& pe = predicted.back().p;
  const Vec2& ae = actual.back().p;
  out.endpoint_err_m = std::hypot(ae.x - pe.x, ae.y - pe.y);
  out.time_bias_s = actual.back().t_s - predicted.back().t_s;
  return out;
}

PocketShadowEpisode make_pocket_shadow_episode(const LegAssessment& predicted,
                                               const std::vector<TimedPose>& actual,
                                               const PropagatedEstimate& estimator_view,
                                               double estimator_age_s) {
  PocketShadowEpisode episode;
  episode.predicted_end_s = predicted.end_s;
  episode.min_gap_pred_m = predicted.min_gap_m;
  episode.latency_s = estimator_view.estimate.latency_s;
  episode.estimator_status = estimator_view.status;
  episode.estimator_age_s = estimator_age_s;
  double trace = 0.0;
  for (std::size_t i = 0; i < StateEstimate::kDim; ++i) {
    trace += estimator_view.estimate.covariance[i * StateEstimate::kDim + i];
  }
  episode.covariance_trace = trace;
  double base = 1.0 / (1.0 + trace);
  if (estimator_view.status == EstimateStatus::kFresh) {
    episode.state_confidence = base;
  } else if (estimator_view.status == EstimateStatus::kPredictedStale) {
    episode.state_confidence = 0.5 * base;
  } else {
    episode.state_confidence = 0.0;
  }
  if (predicted.exec_traj.empty() || actual.empty()) {
    episode.verdict = ShadowVerdict::kInvalid;
    episode.error = compare_trajectories(predicted.exec_traj, actual);
    return episode;
  }
  episode.actual_end_s = actual.back().t_s;
  episode.error = compare_trajectories(predicted.exec_traj, actual);
  if (episode.error.max_dev_m <= kReportMaxDevM && episode.error.endpoint_err_m <= kReportEndpointM) {
    episode.verdict = ShadowVerdict::kObservedTracked;
  } else {
    episode.verdict = ShadowVerdict::kObservedDeviated;
  }
  return episode;
}

const char* shadow_verdict_name(ShadowVerdict verdict) {
  switch (verdict) {
    case ShadowVerdict::kObservedTracked:
      return "ObservedTracked";
    case ShadowVerdict::kObservedDeviated:
      return "ObservedDeviated";
    case ShadowVerdict::kInvalid:
      return "Invalid";
  }
  return "Invalid";
}

} // namespace sim
