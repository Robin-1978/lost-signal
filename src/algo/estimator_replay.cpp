#include "algo/estimator_replay.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace sim {

LatencyStats latency_stats(std::vector<double> latencies) {
  LatencyStats out;
  out.n = latencies.size();
  if (latencies.empty()) {
    return out;
  }
  std::sort(latencies.begin(), latencies.end());
  double sum = 0.0;
  for (double v : latencies) {
    sum += v;
  }
  out.mean_s = sum / static_cast<double>(latencies.size());
  auto rank = [&](double q) -> double {
    // Nearest-rank: idx = ceil(q*n) - 1, clamped. Deterministic, stated.
    std::size_t idx =
      static_cast<std::size_t>(std::ceil(q * static_cast<double>(latencies.size()))) - 1;
    idx = std::min(idx, latencies.size() - 1);
    return latencies[idx];
  };
  out.p50_s = rank(0.50);
  out.p95_s = rank(0.95);
  out.p99_s = rank(0.99);
  out.max_s = latencies.back();
  return out;
}

std::vector<TimedPose> delay_trajectory(const std::vector<TimedPose>& truth, double latency_s) {
  std::vector<TimedPose> out;
  if (truth.empty()) {
    return out;
  }
  const double lag = std::max(0.0, latency_s);
  for (const auto& s : truth) {
    const double want = s.t_s - lag;
    // Step back to the latest truth sample at or before `want`; before the
    // first stamp, hold the first pose.
    Vec2 pose = truth.front().p;
    for (const auto& cand : truth) {
      if (cand.t_s <= want + 1e-9) {
        pose = cand.p;
      } else {
        break;
      }
    }
    out.push_back({s.t_s, pose});
  }
  return out;
}

std::vector<GapLatencyRow> gap_vs_latency(const std::vector<TimedPose>& ego_truth,
                                          const std::vector<std::vector<DynState>>& frames,
                                          double frame_dt_s, double ego_radius_m,
                                          const std::vector<double>& latencies) {
  std::vector<GapLatencyRow> rows;
  if (ego_truth.empty() || latencies.empty()) {
    return rows;
  }
  const double base = timed_gap(ego_truth, frames, frame_dt_s, ego_radius_m);
  for (double lag : latencies) {
    const auto stale = delay_trajectory(ego_truth, lag);
    const double gap = timed_gap(stale, frames, frame_dt_s, ego_radius_m);
    rows.push_back({lag, gap, gap - base});
  }
  return rows;
}

DropoutRow dropout_observe(const std::vector<TimedPose>& truth, double stream_latency_s,
                           double drop_start_s, double drop_duration_s, double query_s,
                           double max_staleness_s, DropoutPolicy policy) {
  DropoutRow row;
  row.dropout_s = std::max(0.0, drop_duration_s);
  EstimateHolder holder;
  const double drop = std::max(0.0, drop_duration_s);
  for (std::size_t i = 0; i < truth.size(); ++i) {
    const auto& s = truth[i];
    // A dropout covering the query removes every sample from drop_start
    // through the query itself (closed at the query end: the endpoint sample
    // must not leak through, or staleness would never be observed).
    if (drop > 0.0 && s.t_s >= drop_start_s && s.t_s <= query_s + 1e-9) {
      continue;
    }
    if (s.t_s > query_s) {
      break;
    }
    StateEstimate estimate;
    estimate.stamp_s = s.t_s;
    estimate.latency_s = std::max(0.0, stream_latency_s);
    // Stream velocity from finite difference (what the body did): the CV
    // policy propagates on this, HoldLast freezes — the error difference
    // between them is the honest policy comparison.
    double vx = 0.0, vy = 0.0;
    if (i + 1 < truth.size() && truth[i + 1].t_s <= query_s + 1e-9) {
      const double dt = std::max(1e-6, truth[i + 1].t_s - s.t_s);
      vx = (truth[i + 1].p.x - s.p.x) / dt;
      vy = (truth[i + 1].p.y - s.p.y) / dt;
    }
    estimate.state = {s.p.x, s.p.y, 0.0, vx, vy, 0.0};
    for (std::size_t i = 0; i < 36; ++i) {
      estimate.covariance[i] = (i % 7 == 0) ? 0.01 : 0.0;
    }
    estimate.wheel_valid = true;
    estimate.imu_valid = true;
    holder.push(estimate);
  }
  const auto view = holder.at(query_s, max_staleness_s, policy);
  row.status = view.status;
  const double trace = 6.0 * 0.01;
  if (view.status == EstimateStatus::kFresh) {
    row.confidence = 1.0 / (1.0 + trace);
  } else if (view.status == EstimateStatus::kPredictedStale) {
    row.confidence = 0.5 / (1.0 + trace);
  } else {
    row.confidence = 0.0;
  }
  if (view.status == EstimateStatus::kInvalid) {
    row.position_error_m = std::numeric_limits<double>::infinity();
    return row;
  }
  Vec2 truth_at = truth.front().p;
  for (const auto& s : truth) {
    if (s.t_s <= query_s + 1e-9) {
      truth_at = s.p;
    } else {
      break;
    }
  }
  row.position_error_m =
    std::hypot(view.estimate.state[0] - truth_at.x, view.estimate.state[1] - truth_at.y);
  return row;
}

} // namespace sim
