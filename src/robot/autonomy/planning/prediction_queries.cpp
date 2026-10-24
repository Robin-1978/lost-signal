#include "robot/autonomy/planning/prediction.hpp"

#include <algorithm>
#include <cmath>

namespace lost_signal::robot::autonomy {

PredictionBatch predict_perception(
 const PerceptionResult& perception, double decision_time_s,
 DynamicPredictionConfig config) {
 std::vector<PredictInput> inputs;
 inputs.reserve(perception.tracks.size());
 const double fallback_observed_s =
 perception.stamp_ns != core::kInvalidTime
 ? perception.stamp_ns * 1e-9
 : decision_time_s;
 for (const auto& track : perception.tracks) {
 const double observed_s = track.last_observed_ns != core::kInvalidTime
 ? track.last_observed_ns * 1e-9
 : fallback_observed_s;
 inputs.push_back(predict_input_from_track(track, observed_s));
 }
 return predict_hypotheses(
 inputs, decision_time_s, std::max(0.0, config.horizon_s),
 config.frame_dt_s, std::max(0.0, config.max_track_age_s));
}

std::optional<DynamicTrack> predicted_state_at(
 const PredictedHypothesis& hypothesis, const PredictionBatch& prediction,
 double future_time_s) {
 if (hypothesis.states.empty() || prediction.future_times_s.empty() ||
 hypothesis.states.size() != prediction.future_times_s.size() ||
 !std::isfinite(future_time_s)) {
 return std::nullopt;
 }
 const double time = std::clamp(
 future_time_s, prediction.future_times_s.front(),
 prediction.future_times_s.back());
 const auto upper = std::lower_bound(
 prediction.future_times_s.begin(), prediction.future_times_s.end(), time);
 if (upper == prediction.future_times_s.begin()) {
 return hypothesis.states.front();
 }
 if (upper == prediction.future_times_s.end()) {
 return hypothesis.states.back();
 }
 const std::size_t hi = static_cast<std::size_t>(
 upper - prediction.future_times_s.begin());
 const std::size_t lo = hi - 1;
 const double t0 = prediction.future_times_s[lo];
 const double t1 = prediction.future_times_s[hi];
 const double u = t1 > t0 ? (time - t0) / (t1 - t0) : 0.0;
 DynamicTrack state = hypothesis.states[lo];
 const DynamicTrack& next = hypothesis.states[hi];
 state.x_m += (next.x_m - state.x_m) * u;
 state.y_m += (next.y_m - state.y_m) * u;
 state.vx_mps += (next.vx_mps - state.vx_mps) * u;
 state.vy_mps += (next.vy_mps - state.vy_mps) * u;
 state.radius_m += (next.radius_m - state.radius_m) * u;
 state.position_uncertainty_m +=
 (next.position_uncertainty_m - state.position_uncertainty_m) * u;
 return state;
}

PredictedClearance minimum_predicted_clearance(
 const PredictionBatch& prediction, core::Vec2 ego_start,
 core::Vec2 ego_end, double interval_start_s, double interval_end_s,
 double ego_radius_m) {
 PredictedClearance best;
 if (prediction.future_times_s.empty() ||
 interval_end_s < interval_start_s ||
 !std::isfinite(interval_start_s) || !std::isfinite(interval_end_s)) {
 return best;
 }
 const double begin = std::max(
 interval_start_s, prediction.future_times_s.front());
 const double end = std::min(
 interval_end_s, prediction.future_times_s.back());
 if (end + 1e-12 < begin) return best;
 const double interval = interval_end_s - interval_start_s;
 const auto ego_at = [&](double time_s) {
 const double u = interval > 1e-12
 ? std::clamp((time_s - interval_start_s) / interval, 0.0, 1.0)
 : 0.0;
 return core::Vec2{
 ego_start.x + (ego_end.x - ego_start.x) * u,
 ego_start.y + (ego_end.y - ego_start.y) * u};
 };
 for (const auto& hypothesis : prediction.hypotheses) {
 std::vector<double> cuts{begin};
 for (const double t : prediction.future_times_s) {
 if (t > begin + 1e-12 && t < end - 1e-12) cuts.push_back(t);
 }
 if (end > begin + 1e-12) cuts.push_back(end);
 for (std::size_t i = 0; i + 1 < cuts.size(); ++i) {
 const double ta = cuts[i], tb = cuts[i + 1];
 const auto a = predicted_state_at(hypothesis, prediction, ta);
 const auto b = predicted_state_at(hypothesis, prediction, tb);
 if (!a || !b) continue;
 const core::Vec2 ea = ego_at(ta), eb = ego_at(tb);
 const core::Vec2 r0{ea.x - a->x_m, ea.y - a->y_m};
 const core::Vec2 relative_delta{
 (eb.x - ea.x) - (b->x_m - a->x_m),
 (eb.y - ea.y) - (b->y_m - a->y_m)};
 const double relative_norm2 =
 relative_delta.x * relative_delta.x +
 relative_delta.y * relative_delta.y;
 const double u = relative_norm2 > 1e-12
 ? std::clamp(-(r0.x * relative_delta.x +
 r0.y * relative_delta.y) / relative_norm2, 0.0, 1.0)
 : 0.0;
 const double center_distance = std::hypot(
 r0.x + relative_delta.x * u, r0.y + relative_delta.y * u);
 const double occupied = std::max(
 a->occupied_radius_m(), b->occupied_radius_m());
 const double clearance = center_distance -
 std::max(0.0, ego_radius_m) - occupied;
 if (clearance < best.clearance_m) {
 best.clearance_m = clearance;
 best.time_s = ta + (tb - ta) * u;
 best.track_id = hypothesis.track_id;
 best.hypothesis_id = hypothesis.hypothesis_id;
 }
 }
 if (cuts.size() == 1) {
 const auto state = predicted_state_at(hypothesis, prediction, begin);
 if (!state) continue;
 const core::Vec2 ego = ego_at(begin);
 const double clearance = std::hypot(
 ego.x - state->x_m, ego.y - state->y_m) -
 std::max(0.0, ego_radius_m) - state->occupied_radius_m();
 if (clearance < best.clearance_m) {
 best.clearance_m = clearance;
 best.time_s = begin;
 best.track_id = hypothesis.track_id;
 best.hypothesis_id = hypothesis.hypothesis_id;
 }
 }
 }
 return best;
}

} // namespace lost_signal::robot::autonomy
