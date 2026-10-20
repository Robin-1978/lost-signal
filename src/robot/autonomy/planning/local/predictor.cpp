#include "robot/autonomy/planning/local/predictor.hpp"

#include "robot/autonomy/planning/global/astar.hpp"
#include "robot/autonomy/planning/path_smoothing.hpp"
#include "robot/core/math.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <set>
#include <utility>

namespace sim {
namespace {

// Static recheck of an executed trajectory on the caller-inflated grid.
// The A* contract covers the grid polyline; smoothing and servo lag move the
// body off it, so the CHECKED samples are rechecked here, not inherited.
bool path_hits_obstacle(const std::vector<Vec2>& points, const GridMap& inflated_map) {
 for (const auto& p : points) {
  if (!inflated_map.isFree(p.x, p.y)) {
   return true;
  }
 }
 return false;
}

bool traj_hits_obstacle(const std::vector<TimedPose>& traj, const GridMap& inflated_map) {
 for (const auto& s : traj) {
  if (!inflated_map.isFree(s.p.x, s.p.y)) {
   return true;
  }
 }
 return false;
}

} // namespace

namespace {

// Reflection inside the observed corridor. Unbounded axes (no direction flip
// seen yet) extrapolate linearly; bounded axes bounce off the observed
// min/max. The 0.05 m corridor floor and 8-reflection cap are method
// constants: narrow corridors carry no direction information, and the cap
// bounds work for pathological velocity/corridor ratios.
struct ReflectedAxisState {
 double position{0.0};
 double velocity{0.0};
};

ReflectedAxisState reflected_axis(double position, double velocity,
 double time_s, double minimum, double maximum, bool bounded,
 double endpoint_dwell_s, double initial_dwell_remaining_s) {
 if (!bounded || maximum - minimum < 0.05 || std::abs(velocity) < 1e-9)
 return {position + velocity * time_s, velocity};
 double remaining = std::max(0.0, time_s);
 position = std::clamp(position, minimum, maximum);
 if (initial_dwell_remaining_s > 0.0) {
 if (remaining <= initial_dwell_remaining_s)
 return {position, 0.0};
 remaining -= initial_dwell_remaining_s;
 if ((position >= maximum - 1e-6 && velocity > 0.0) ||
 (position <= minimum + 1e-6 && velocity < 0.0))
 velocity = -velocity;
 }
 for (int transition = 0; transition < 16 && remaining > 1e-12; ++transition) {
 const double boundary = velocity > 0.0 ? maximum : minimum;
 const double travel_s = std::abs(boundary - position) / std::abs(velocity);
 if (remaining < travel_s)
 return {position + velocity * remaining, velocity};
 position = boundary;
 remaining -= travel_s;
 if (remaining <= endpoint_dwell_s)
 return {position, 0.0};
 remaining -= endpoint_dwell_s;
 velocity = -velocity;
 }
 return {std::clamp(position + velocity * remaining, minimum, maximum), velocity};
}

// Time-parameterized closest approach over one frame interval: both bodies
// move with the SAME time parameter (relative motion r + w*t), so this is a
// point-to-segment distance, not a segment-segment one. An independent-(s,t)
// segment distance would flag two bodies using the same space at different
// times as a collision — exactly what waiting separates. Reviewer gate:
// discrete frame sampling alone can miss a fast crosser slipping between
// frames; this sweeps every interval exactly (linear motion per interval is
// exact for our predictors) and is conservative nowhere.
struct IntervalClosestApproach {
 double distance_m{0.0};
 double fraction{0.0};
};

IntervalClosestApproach interval_closest_approach(Vec2 a0, Vec2 a1, Vec2 b0, Vec2 b1) {
 const Vec2 r{a0.x - b0.x, a0.y - b0.y};
 const Vec2 w{(a1.x - a0.x) - (b1.x - b0.x), (a1.y - a0.y) - (b1.y - b0.y)};
 const double speed_squared = w.x * w.x + w.y * w.y;
 const double fraction = speed_squared <= 1e-12
 ? 0.0
 : std::clamp(-(r.x * w.x + r.y * w.y) / speed_squared, 0.0, 1.0);
 return {
 std::hypot(r.x + fraction * w.x, r.y + fraction * w.y),
 fraction,
 };
}

double interval_min_dist(Vec2 a0, Vec2 a1, Vec2 b0, Vec2 b1) {
 return interval_closest_approach(a0, a1, b0, b1).distance_m;
}

} // namespace

PredictionBatch predict_hypotheses(
 const std::vector<PredictInput>& inputs, double time_s, double horizon_s,
 double frame_dt_s, double max_age_s) {
 PredictionBatch batch;
 batch.stamp_s = time_s;
 batch.frame_dt_s = frame_dt_s;
 if (!(frame_dt_s > 0.0) || horizon_s < 0.0 || max_age_s < 0.0)
 return batch;
 for (double future_s = 0.0; future_s <= horizon_s + 1e-9;
 future_s += frame_dt_s) {
 batch.future_times_s.push_back(future_s);
 }
 const auto predict_mode = [](
 const PredictInput& input, Vec2 velocity, double prediction_time) {
 auto predicted = input.state;
 const auto x = reflected_axis(
 predicted.p.x, velocity.x, prediction_time, input.min_observed.x,
 input.max_observed.x, input.bounded_x, input.endpoint_dwell_s,
 input.initial_dwell_remaining_s);
 const auto y = reflected_axis(
 predicted.p.y, velocity.y, prediction_time, input.min_observed.y,
 input.max_observed.y, input.bounded_y, input.endpoint_dwell_s,
 input.initial_dwell_remaining_s);
 predicted.p = {x.position, y.position};
 predicted.v = {x.velocity, y.velocity};
 predicted.position_uncertainty_m =
 std::min(std::max(0.0, input.position_uncertainty_cap_m),
 std::max(0.0, input.state.position_uncertainty_m) +
 std::max(0.0, input.position_uncertainty_growth_mps) *
 std::max(0.0, prediction_time));
 return predicted;
 };
 for (const auto& input : inputs) {
 const double age = time_s - input.last_observed_s;
 if (age < -1e-9 || age > max_age_s)
 continue;
 struct VelocityMode {
 Vec2 velocity;
 bool alternate{false};
 };
 std::vector<VelocityMode> modes;
 const auto append_mode = [&](Vec2 velocity, bool alternate) {
 const auto duplicate = std::any_of(
 modes.begin(), modes.end(), [&](const VelocityMode& existing) {
 return std::hypot(
 existing.velocity.x - velocity.x,
 existing.velocity.y - velocity.y) <= 1e-9;
 });
 if (!duplicate)
 modes.push_back({velocity, alternate});
 };
 const auto append_speed_envelope = [&](Vec2 velocity, bool alternate) {
 const double speed = std::hypot(velocity.x, velocity.y);
 const double uncertainty = std::max(0.0, input.velocity_uncertainty_mps);
 if (speed > 1e-9 && uncertainty > 0.0) {
 const Vec2 direction{velocity.x / speed, velocity.y / speed};
 const double slow = std::max(0.0, speed - uncertainty);
 append_mode({direction.x * slow, direction.y * slow}, alternate);
 }
 append_mode(velocity, alternate);
 if (speed > 1e-9 && uncertainty > 0.0) {
 const Vec2 direction{velocity.x / speed, velocity.y / speed};
 const double fast = speed + uncertainty;
 append_mode({direction.x * fast, direction.y * fast}, alternate);
 }
 };
 append_speed_envelope(input.state.v, false);
 if (input.alternate_velocity_valid)
 append_speed_envelope(input.alternate_velocity, true);
 for (std::size_t mode_index = 0; mode_index < modes.size(); ++mode_index) {
 const auto& mode = modes[mode_index];
 PredictedObjectHypothesis hypothesis;
 hypothesis.track_id = input.track_id;
 hypothesis.hypothesis_id = mode_index;
 const double speed = std::hypot(mode.velocity.x, mode.velocity.y);
 hypothesis.model = mode.alternate
 ? PredictionModel::AlternateIntent
 : speed <= 1e-9
 ? PredictionModel::Stationary
 : (input.bounded_x || input.bounded_y)
 ? PredictionModel::ReflectedCorridor
 : PredictionModel::ConstantVelocity;
 hypothesis.states.reserve(batch.future_times_s.size());
 for (const double future_s : batch.future_times_s) {
 hypothesis.states.push_back(predict_mode(
 input, mode.velocity, age + future_s));
 }
 batch.hypotheses.push_back(std::move(hypothesis));
 }
 }
 return batch;
}

std::vector<std::vector<DynamicObjectState>> flatten_prediction_frames(
 const PredictionBatch& prediction) {
 std::vector<std::vector<DynamicObjectState>> frames(
 prediction.future_times_s.size());
 for (const auto& hypothesis : prediction.hypotheses) {
 const std::size_t count = std::min(frames.size(), hypothesis.states.size());
 for (std::size_t frame = 0; frame < count; ++frame)
 frames[frame].push_back(hypothesis.states[frame]);
 }
 return frames;
}

std::vector<std::vector<DynamicObjectState>> predict_frames(
 const std::vector<PredictInput>& inputs, double time_s, double horizon_s,
 double frame_dt_s, double max_age_s) {
 return flatten_prediction_frames(predict_hypotheses(
 inputs, time_s, horizon_s, frame_dt_s, max_age_s));
}

std::vector<TimedPose> remaining_timed_trajectory(
 const std::vector<TimedPose>& trajectory, double elapsed_s) {
 if (trajectory.empty())
 return {};
 const double elapsed = std::clamp(
 std::isfinite(elapsed_s) ? elapsed_s : 0.0,
 trajectory.front().t_s, trajectory.back().t_s);
 const auto after = std::lower_bound(
 trajectory.begin(), trajectory.end(), elapsed,
 [](const TimedPose& pose, double time_s) { return pose.t_s < time_s; });
 Vec2 current = trajectory.back().p;
 if (after == trajectory.begin()) {
 current = after->p;
 } else if (after != trajectory.end()) {
 const auto& b = *after;
 const auto& a = *(after - 1);
 const double blend = (elapsed - a.t_s) /
 std::max(1e-9, b.t_s - a.t_s);
 current = {
 a.p.x + (b.p.x - a.p.x) * blend,
 a.p.y + (b.p.y - a.p.y) * blend,
 };
 }
 std::vector<TimedPose> remaining{{0.0, current}};
 for (const auto& pose : trajectory) {
 if (pose.t_s <= elapsed + 1e-12)
 continue;
 remaining.push_back({pose.t_s - elapsed, pose.p});
 }
 return remaining;
}

double match_timed_trajectory_phase(
 const std::vector<TimedPose>& trajectory, Vec2 measured_position,
 double nominal_elapsed_s, double admitted_time_error_s) {
 if (trajectory.empty())
 return 0.0;
 const double first_time_s = trajectory.front().t_s;
 const double last_time_s = trajectory.back().t_s;
 const double nominal_s = std::clamp(
 std::isfinite(nominal_elapsed_s) ? nominal_elapsed_s : first_time_s,
 first_time_s, last_time_s);
 const double tolerance_s = std::max(
 0.0, std::isfinite(admitted_time_error_s) ? admitted_time_error_s : 0.0);
 const double minimum_s = std::max(first_time_s, nominal_s - tolerance_s);
 const double maximum_s = std::min(last_time_s, nominal_s + tolerance_s);
 double best_time_s = nominal_s;
 double best_distance_sq = std::numeric_limits<double>::infinity();
 const auto consider = [&](double time_s, Vec2 position) {
 const double dx = measured_position.x - position.x;
 const double dy = measured_position.y - position.y;
 const double distance_sq = dx * dx + dy * dy;
 constexpr double kDistanceEquivalenceSq = 1e-18;
 if (distance_sq + kDistanceEquivalenceSq < best_distance_sq ||
 (std::abs(distance_sq - best_distance_sq) <= kDistanceEquivalenceSq &&
 std::abs(time_s - nominal_s) < std::abs(best_time_s - nominal_s))) {
 best_distance_sq = distance_sq;
 best_time_s = time_s;
 }
 };

 for (std::size_t index = 0; index + 1 < trajectory.size(); ++index) {
 const auto& from = trajectory[index];
 const auto& to = trajectory[index + 1];
 const double segment_start_s = std::max(minimum_s, from.t_s);
 const double segment_end_s = std::min(maximum_s, to.t_s);
 if (segment_end_s + 1e-12 < segment_start_s)
 continue;
 const double duration_s = std::max(1e-12, to.t_s - from.t_s);
 const double start_blend = std::clamp(
 (segment_start_s - from.t_s) / duration_s, 0.0, 1.0);
 const double end_blend = std::clamp(
 (segment_end_s - from.t_s) / duration_s, 0.0, 1.0);
 const Vec2 segment_start{
 from.p.x + (to.p.x - from.p.x) * start_blend,
 from.p.y + (to.p.y - from.p.y) * start_blend,
 };
 const Vec2 segment_end{
 from.p.x + (to.p.x - from.p.x) * end_blend,
 from.p.y + (to.p.y - from.p.y) * end_blend,
 };
 const Vec2 delta{
 segment_end.x - segment_start.x,
 segment_end.y - segment_start.y,
 };
 const double length_sq = delta.x * delta.x + delta.y * delta.y;
 double projection = 0.0;
 if (length_sq > 1e-18) {
 projection = std::clamp(
 ((measured_position.x - segment_start.x) * delta.x +
 (measured_position.y - segment_start.y) * delta.y) / length_sq,
 0.0, 1.0);
 } else if (segment_end_s > segment_start_s + 1e-12) {
 // A dwell has identical geometry at every phase. Keep the phase nearest
 // the nominal clock so a stationary reference cannot manufacture drift.
 projection = std::clamp(
 (nominal_s - segment_start_s) / (segment_end_s - segment_start_s),
 0.0, 1.0);
 }
 consider(
 segment_start_s + (segment_end_s - segment_start_s) * projection,
 {segment_start.x + delta.x * projection,
 segment_start.y + delta.y * projection});
 }
 if (!std::isfinite(best_distance_sq)) {
 const auto remaining = remaining_timed_trajectory(trajectory, nominal_s);
 if (!remaining.empty())
 consider(nominal_s, remaining.front().p);
 }
 return std::clamp(best_time_s, minimum_s, maximum_s);
}

double timed_gap(const std::vector<TimedPose>& ego,
                 const std::vector<std::vector<DynamicObjectState>>& frames, double frame_dt_s,
                 double ego_radius_m) {
 if (ego.empty() || frames.empty()) {
  return std::numeric_limits<double>::infinity();
 }
 const double dt = std::max(1e-6, frame_dt_s);
 auto ego_at = [&](double t) {
  if (t <= ego.front().t_s) {
   return ego.front().p;
  }
  for (std::size_t i = 0; i + 1 < ego.size(); ++i) {
   if (t <= ego[i + 1].t_s) {
    const double span = ego[i + 1].t_s - ego[i].t_s;
    const double a = span < 1e-9 ? 0.0 : std::clamp((t - ego[i].t_s) / span, 0.0, 1.0);
    return Vec2{ego[i].p.x + (ego[i + 1].p.x - ego[i].p.x) * a,
                ego[i].p.y + (ego[i + 1].p.y - ego[i].p.y) * a};
   }
  }
  return ego.back().p;
 };
 double gap = std::numeric_limits<double>::infinity();
 if (frames.size() == 1) {
  for (const auto& threat : frames.front()) {
   const Vec2 e = ego_at(0.0);
 gap = std::min(gap, std::hypot(threat.p.x - e.x, threat.p.y - e.y) -
 threat.occupied_radius_m() - ego_radius_m);
  }
  return gap;
 }
 for (std::size_t k = 0; k + 1 < frames.size(); ++k) {
  const Vec2 ego0 = ego_at(static_cast<double>(k) * dt);
  const Vec2 ego1 = ego_at(static_cast<double>(k + 1) * dt);
  const auto& fa = frames[k];
  const auto& fb = frames[k + 1];
  for (std::size_t i = 0; i < fa.size() && i < fb.size(); ++i) {
 gap = std::min(gap, interval_min_dist(ego0, ego1, fa[i].p, fb[i].p) -
 std::max(fa[i].occupied_radius_m(), fb[i].occupied_radius_m()) - ego_radius_m);
  }
 }
 return gap;
}

double predicted_gap(const std::vector<Vec2>& path, std::size_t start_idx, double speed_mps,
                     const std::vector<std::vector<DynamicObjectState>>& frames, double frame_dt_s,
                     double ego_radius_m, double depart_delay_s) {
 if (path.empty() || frames.empty()) {
  return std::numeric_limits<double>::infinity();
 }
 // Ego distance at frame k (absolute time k*dt), departing after the delay.
 // Frame 0 is now: ego sits at start. Fixed from an off-by-one ((k+1)*dt)
 // that compared the threat-now against the ego-in-one-step.
 const double dt = std::max(1e-6, frame_dt_s);
 const double speed = std::max(0.0, speed_mps);
 const double delay = std::max(0.0, depart_delay_s);
 auto ego_at = [&](double distance) {
  double walked = 0.0;
  Vec2 ego = path[std::min(start_idx, path.size() - 1)];
  for (std::size_t seg = start_idx; seg + 1 < path.size(); ++seg) {
   const double leg = std::hypot(path[seg + 1].x - path[seg].x, path[seg + 1].y - path[seg].y);
   if (leg < 1e-9) {
    continue;
   }
   if (walked + leg >= distance) {
    const double t = std::clamp((distance - walked) / leg, 0.0, 1.0);
    return Vec2{path[seg].x + (path[seg + 1].x - path[seg].x) * t,
                path[seg].y + (path[seg + 1].y - path[seg].y) * t};
   }
   walked += leg;
   ego = path[seg + 1];
  }
  return ego;
 };
 const auto ego_frame = [&](std::size_t k) {
  return ego_at(speed * std::max(0.0, static_cast<double>(k) * dt - delay));
 };
 double gap = std::numeric_limits<double>::infinity();
 if (frames.size() == 1) {
  // Degenerate single frame: point check (segment helper covers it, but be
  // explicit so the intent reads).
  for (const auto& threat : frames.front()) {
 gap = std::min(gap, std::hypot(threat.p.x - ego_frame(0).x, threat.p.y - ego_frame(0).y) -
 threat.occupied_radius_m() -
                         ego_radius_m);
  }
  return gap;
 }
 // Swept intervals: both ego and threat move linearly between frames, so the
 // closed-form segment distance cannot miss an inter-frame crossing.
 for (std::size_t k = 0; k + 1 < frames.size(); ++k) {
  const Vec2 ego0 = ego_frame(k);
  const Vec2 ego1 = ego_frame(k + 1);
  const auto& fa = frames[k];
  const auto& fb = frames[k + 1];
  for (std::size_t i = 0; i < fa.size() && i < fb.size(); ++i) {
 gap = std::min(gap, interval_min_dist(ego0, ego1, fa[i].p, fb[i].p) -
 std::max(fa[i].occupied_radius_m(), fb[i].occupied_radius_m()) - ego_radius_m);
  }
 }
 return gap;
}

PredictedGapEvidence predicted_gap_evidence(
 const std::vector<Vec2>& path, std::size_t start_idx, double speed_mps,
 const std::vector<std::vector<DynamicObjectState>>& frames, double frame_dt_s,
 double ego_radius_m, double margin_m, double depart_delay_s) {
 PredictedGapEvidence evidence;
 if (path.empty() || frames.empty())
 return evidence;
 const double dt = std::max(1e-6, frame_dt_s);
 const double speed = std::max(0.0, speed_mps);
 const double delay = std::max(0.0, depart_delay_s);
 auto ego_at = [&](double distance) {
 double walked = 0.0;
 Vec2 ego = path[std::min(start_idx, path.size() - 1)];
 for (std::size_t segment = start_idx; segment + 1 < path.size(); ++segment) {
 const double length = std::hypot(
 path[segment + 1].x - path[segment].x,
 path[segment + 1].y - path[segment].y);
 if (length < 1e-9)
 continue;
 if (walked + length >= distance) {
 const double fraction = std::clamp((distance - walked) / length, 0.0, 1.0);
 return Vec2{
 path[segment].x + (path[segment + 1].x - path[segment].x) * fraction,
 path[segment].y + (path[segment + 1].y - path[segment].y) * fraction,
 };
 }
 walked += length;
 ego = path[segment + 1];
 }
 return ego;
 };
 const auto ego_frame = [&](std::size_t frame) {
 return ego_at(speed * std::max(0.0, static_cast<double>(frame) * dt - delay));
 };
 const auto consider = [&](double center_distance_m, double occupied_radius_m,
 double time_s, Vec2 ego_position) {
 const double gap_m = center_distance_m - occupied_radius_m - ego_radius_m;
 evidence.min_gap_m = std::min(evidence.min_gap_m, gap_m);
 if (gap_m < margin_m && time_s < evidence.first_conflict_s) {
 evidence.conflict = true;
 evidence.first_conflict_s = time_s;
 evidence.first_conflict_position = ego_position;
 }
 };
 if (frames.size() == 1) {
 const Vec2 ego = ego_frame(0);
 for (const auto& threat : frames.front()) {
 consider(std::hypot(threat.p.x - ego.x, threat.p.y - ego.y),
 threat.occupied_radius_m(), 0.0, ego);
 }
 return evidence;
 }
 for (std::size_t frame = 0; frame + 1 < frames.size(); ++frame) {
 const Vec2 ego0 = ego_frame(frame);
 const Vec2 ego1 = ego_frame(frame + 1);
 const auto& current = frames[frame];
 const auto& next = frames[frame + 1];
 for (std::size_t object = 0; object < current.size() && object < next.size(); ++object) {
 const auto closest = interval_closest_approach(
 ego0, ego1, current[object].p, next[object].p);
 const double occupied_radius_m =
 std::max(current[object].occupied_radius_m(), next[object].occupied_radius_m());
 const double gap_m = closest.distance_m - occupied_radius_m - ego_radius_m;
 evidence.min_gap_m = std::min(evidence.min_gap_m, gap_m);
 if (gap_m >= margin_m)
 continue;

 // Report the first entry into the safety envelope, not merely the later
 // closest-approach instant. In relative coordinates q(f)=r+w*f, solve
 // |q(f)|^2=R^2 for the smaller root. The strict gap test above proves the
 // interval actually enters the open envelope, so a tangent does not become
 // a false conflict. This timestamp is used to place an upstream stop pose.
 const Vec2 relative0{ego0.x - current[object].p.x,
 ego0.y - current[object].p.y};
 const Vec2 relative_step{
 (ego1.x - ego0.x) - (next[object].p.x - current[object].p.x),
 (ego1.y - ego0.y) - (next[object].p.y - current[object].p.y),
 };
 const double threshold_m = occupied_radius_m + ego_radius_m + margin_m;
 const double c = relative0.x * relative0.x + relative0.y * relative0.y -
 threshold_m * threshold_m;
 double entry_fraction = 0.0;
 if (c >= 0.0) {
 const double a = relative_step.x * relative_step.x +
 relative_step.y * relative_step.y;
 const double b = 2.0 * (relative0.x * relative_step.x +
 relative0.y * relative_step.y);
 const double discriminant = std::max(0.0, b * b - 4.0 * a * c);
 if (a > 1e-12) {
 entry_fraction = std::clamp(
 (-b - std::sqrt(discriminant)) / (2.0 * a), 0.0, 1.0);
 } else {
 entry_fraction = closest.fraction;
 }
 }
 const Vec2 ego_entry{
 ego0.x + entry_fraction * (ego1.x - ego0.x),
 ego0.y + entry_fraction * (ego1.y - ego0.y),
 };
 const double entry_time_s =
 (static_cast<double>(frame) + entry_fraction) * dt;
 if (entry_time_s < evidence.first_conflict_s) {
 evidence.conflict = true;
 evidence.first_conflict_s = entry_time_s;
 evidence.first_conflict_position = ego_entry;
 }
 }
 }
 return evidence;
}

double earliest_departure(const std::vector<Vec2>& path, std::size_t start_idx, double cruise_mps,
                          const std::vector<std::vector<DynamicObjectState>>& frames, double frame_dt_s,
                          double ego_radius_m, double margin_m) {
 return assess_departure(path, start_idx, cruise_mps, frames, frame_dt_s, ego_radius_m, margin_m).wait_s;
}

const char* depart_reason_name(DepartReason reason) {
 switch (reason) {
 case DepartReason::kGoNow:
  return "GoNow";
 case DepartReason::kWait:
  return "Wait";
 case DepartReason::kBlockedInWindow:
  return "BlockedInWindow";
 case DepartReason::kWaitUnsafe:
  return "WaitUnsafe";
 case DepartReason::kHorizonShort:
  return "HorizonShort";
 case DepartReason::kInvalid:
  return "Invalid";
 case DepartReason::kUnassessed:
  return "Unassessed";
 case DepartReason::kClear:
  return "Clear";
 case DepartReason::kStaticBlocked:
  return "StaticBlocked";
 case DepartReason::kCapsViolated:
  return "CapsViolated";
 }
 return "Invalid";
}

Feasibility check_candidate(LocalCandidate type, const std::vector<Vec2>& path, std::size_t start_idx,
                            double cruise_mps, const std::vector<std::vector<DynamicObjectState>>& frames,
                            double frame_dt_s, double ego_radius_m, double margin_m) {
 if (type == LocalCandidate::kWaitAtPocket || type == LocalCandidate::kRejoin ||
     type == LocalCandidate::kDetour) {
  // No spatial planner exists yet: Unknown with its own reason, never
  // borrowed. Phase-2 implements these without touching this contract.
  return {Feasibility::Verdict::kUnknown, DepartReason::kUnassessed, std::numeric_limits<double>::infinity(),
          std::numeric_limits<double>::infinity()};
 }
 const DepartureAssessment assessed =
   assess_departure(path, start_idx, cruise_mps, frames, frame_dt_s, ego_radius_m, margin_m);
 if (type == LocalCandidate::kProceed) {
  // Proceed means now: any required wait refuses it, but the refusal keeps
  // its cause (too-early vs blocked vs undecidable).
  if (assessed.reason == DepartReason::kGoNow) {
   return {Feasibility::Verdict::kFeasible, DepartReason::kGoNow, 0.0,
           predicted_gap(path, start_idx, cruise_mps, frames, frame_dt_s, ego_radius_m, 0.0)};
  }
  if (assessed.reason == DepartReason::kHorizonShort || assessed.reason == DepartReason::kInvalid) {
   return {Feasibility::Verdict::kUnknown, assessed.reason, std::numeric_limits<double>::infinity(),
           predicted_gap(path, start_idx, cruise_mps, frames, frame_dt_s, ego_radius_m, 0.0)};
  }
  return {Feasibility::Verdict::kInfeasible, assessed.reason, std::numeric_limits<double>::infinity(),
          predicted_gap(path, start_idx, cruise_mps, frames, frame_dt_s, ego_radius_m, 0.0)};
 }
 // kWaitInPlace: feasible exactly when the time assessment names a departure
 // (now or later); the waiting spot was already proven inside assess.
 if (assessed.reason == DepartReason::kGoNow || assessed.reason == DepartReason::kWait) {
  const double delay = assessed.reason == DepartReason::kGoNow ? 0.0 : assessed.wait_s;
  return {Feasibility::Verdict::kFeasible, assessed.reason, assessed.wait_s,
          predicted_gap(path, start_idx, cruise_mps, frames, frame_dt_s, ego_radius_m, delay)};
 }
 if (assessed.reason == DepartReason::kHorizonShort || assessed.reason == DepartReason::kInvalid) {
  return {Feasibility::Verdict::kUnknown, assessed.reason, std::numeric_limits<double>::infinity(),
          predicted_gap(path, start_idx, cruise_mps, frames, frame_dt_s, ego_radius_m, 0.0)};
 }
 return {Feasibility::Verdict::kInfeasible, assessed.reason, std::numeric_limits<double>::infinity(),
         predicted_gap(path, start_idx, cruise_mps, frames, frame_dt_s, ego_radius_m, 0.0)};
}

namespace {

// Remaining polyline length from start_idx (chaining schedule, not proof).
double remaining_length(const std::vector<Vec2>& path, std::size_t start_idx) {
 double length = 0.0;
 for (std::size_t seg = start_idx; seg + 1 < path.size(); ++seg) {
  length += std::hypot(path[seg + 1].x - path[seg].x, path[seg + 1].y - path[seg].y);
 }
 return length;
}

} // namespace

// Turn-drive-turn execution of a geometric polyline: rotate in place at
// yaw_rate, drive legs at cruise. The checked trajectory is exactly this,
// never the raw grid polyline by proxy (smoothing, when the Ackermann slice
// needs it, will likewise rebuild this trajectory before any check).
std::vector<TimedPose> execute_turn_drive_turn(const std::vector<Vec2>& route, double start_theta,
                                               double cruise_mps, double yaw_rate_rads) {
 auto wrap = [](double angle) { return std::atan2(std::sin(angle), std::cos(angle)); };
 std::vector<TimedPose> exec{{0.0, route.front()}};
 double time_s = 0.0;
 double heading = start_theta;
 for (std::size_t i = 0; i + 1 < route.size(); ++i) {
  const double leg = std::hypot(route[i + 1].x - route[i].x, route[i + 1].y - route[i].y);
  if (leg < 1e-9) {
   continue;
  }
  const double desired = std::atan2(route[i + 1].y - route[i].y, route[i + 1].x - route[i].x);
  time_s += std::abs(wrap(desired - heading)) / std::max(1e-6, yaw_rate_rads);
  exec.push_back({time_s, route[i]});
  heading = desired;
  time_s += leg / std::max(1e-6, cruise_mps);
  exec.push_back({time_s, route[i + 1]});
 }
 return exec;
}

// Ackermann execution: follow the smoothed (filleted) path at cruise through
// a servo-lag bicycle sim. Curvature feedforward comes from EXACT construction
// curvature (kappa parallel to the fillet points), plus identity pursuit of the
// path tangent (gain 1 = point wheels where the path goes, capped by authority;
// only the feedforward part can trip the caps verdict). Servo mirrors
// production structure (ackermann.cpp): P loop on angle error scaled by rate,
// first-order lag on the rate state, clamps on both.
// Lag/rate shape the CHECKED trajectory — a slow servo cuts corners wide and
// the swept + static rechecks see it. Instant-cruise like the spin branch
// (same approximation level, documented); the 0.5 m/s steer-authority floor
// never binds because execution never drives slow.
struct AckermannExec {
 std::vector<TimedPose> traj;
 double max_delta{0.0};
 bool demand_clipped{false};
};

AckermannExec execute_ackermann(const std::vector<Vec2>& points, const std::vector<double>& kappa,
                                double start_theta, double cruise_mps, const ChassisExecCaps& caps) {
 AckermannExec out;
 constexpr double kSimDtS = 0.02; // servo-lag resolution (matches physics step; stated).
 if (points.empty() || points.size() != kappa.size() || !(cruise_mps > 0.0)) {
  return out;
 }
 std::vector<double> arc(points.size(), 0.0);
 for (std::size_t i = 1; i < points.size(); ++i) {
  arc[i] = arc[i - 1] + std::hypot(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y);
 }
 const double total = arc.back();
 // Per-sample tangent (segment direction; last sample reuses its segment).
 // Demand = curvature feedforward + identity pursuit of the tangent, capped
 // by steer authority. Gain 1 is the identity map (point wheels where the
 // path goes), not a tuned gain; only the feedforward part can trip the
 // caps verdict below. Lag/deviation are CHECKED downstream either way.
 std::vector<double> tangent(points.size(), 0.0);
 for (std::size_t i = 0; i + 1 < points.size(); ++i) {
  tangent[i] = std::atan2(points[i + 1].y - points[i].y, points[i + 1].x - points[i].x);
 }
 tangent.back() = points.size() > 1 ? tangent[points.size() - 2] : start_theta;
 const double kp = 1.0 / std::max(0.05, caps.servo_response_s * caps.max_steer_rate_radps);
 double theta = start_theta;
 double delta = 0.0;
 double rate = 0.0;
 double t = 0.0;
 Vec2 pos = points.front();
 out.traj.push_back({0.0, pos});
 std::size_t seg = 0;
 while (t * cruise_mps < total) {
  const double s = t * cruise_mps;
  while (seg + 1 < arc.size() && arc[seg + 1] < s) {
   ++seg;
  }
  const std::size_t k0 = std::min(seg, kappa.size() - 1);
  const double feed = std::atan(caps.wheel_base_m * kappa[k0]);
  if (std::abs(feed) > caps.max_steer_rad + 1e-9) {
   out.demand_clipped = true;
   out.traj.clear();
   return out;
  }
  const double demand =
    std::clamp(feed + wrap_angle(tangent[k0] - theta), -caps.max_steer_rad, caps.max_steer_rad);
  const double pwm = std::clamp(kp * (demand - delta), -1.0, 1.0);
  const double rate_demand =
    std::clamp(pwm * caps.max_steer_rate_radps, -caps.max_steer_rate_radps, caps.max_steer_rate_radps);
  const double lag = caps.servo_response_s > 0.0 ? 1.0 - std::exp(-kSimDtS / caps.servo_response_s) : 1.0;
  rate += (rate_demand - rate) * lag;
  delta = std::clamp(delta + rate * kSimDtS, -caps.max_steer_rad, caps.max_steer_rad);
  out.max_delta = std::max(out.max_delta, std::abs(delta));
  theta += cruise_mps * std::tan(delta) / std::max(0.05, caps.wheel_base_m) * kSimDtS;
  pos = {pos.x + cruise_mps * std::cos(theta) * kSimDtS, pos.y + cruise_mps * std::sin(theta) * kSimDtS};
  t += kSimDtS;
  out.traj.push_back({t, pos});
 }
 return out;
}

LegAssessment assess_transit(const GridMap& inflated_map, Vec2 start, double start_theta, Vec2 pocket,
                             double cruise_mps, const ChassisExecCaps& exec,
                             const std::vector<std::vector<DynamicObjectState>>& frames, double frame_dt_s,
                             double ego_radius_m, double margin_m) {
 const LegAssessment invalid{Feasibility::Verdict::kUnknown,
                             DepartReason::kInvalid,
                             std::numeric_limits<double>::infinity(),
                             std::numeric_limits<double>::infinity(),
                             {}};
 if (frames.empty() || cruise_mps <= 0.0 || (exec.can_spin && exec.yaw_rate_rads <= 0.0)) {
  return invalid;
 }
 const double dt = std::max(1e-6, frame_dt_s);
 const double horizon = static_cast<double>(frames.size() > 0 ? frames.size() - 1 : 0) * dt;
 // Geometric path first (A* on the caller-inflated grid); no path is proven
 // static infeasibility, not traffic, not unknown.
 std::vector<Vec2> route;
 if (std::hypot(pocket.x - start.x, pocket.y - start.y) < 1e-9) {
  route = {start};
 } else {
  const auto plan = astar_plan(inflated_map, start, pocket, 1.0);
  if (!plan.ok || plan.path.empty()) {
   return {Feasibility::Verdict::kInfeasible,
           DepartReason::kStaticBlocked,
           std::numeric_limits<double>::infinity(),
           std::numeric_limits<double>::infinity(),
           {}};
  }
  route.push_back(start);
  for (const auto& point : plan.path)
   route.push_back(point.position);
  route.push_back(pocket);
 }
 if (!exec.can_spin) {
  // Ackermann execution: fillet at Rmin, static-recheck the smoothed path,
  // servo-lag bicycle follow, static-recheck + sweep the SIMULATED path.
  if (!(exec.wheel_base_m > 0.0) || !(exec.max_steer_rad > 0.0) || !(exec.max_steer_rate_radps > 0.0) ||
      !(exec.servo_response_s >= 0.0)) {
   double nominal = 0.0;
   for (std::size_t i = 0; i + 1 < route.size(); ++i) {
    nominal += std::hypot(route[i + 1].x - route[i].x, route[i + 1].y - route[i].y);
   }
   return {Feasibility::Verdict::kUnknown,
           DepartReason::kUnassessed,
           nominal / std::max(1e-6, cruise_mps),
           std::numeric_limits<double>::infinity(),
           {}};
  }
  const double r_min = exec.wheel_base_m / std::tan(exec.max_steer_rad);
  const auto smooth = fillet_polyline(route, r_min);
 if (!smooth.valid) {
   return {Feasibility::Verdict::kInfeasible,
           DepartReason::kCapsViolated,
           std::numeric_limits<double>::infinity(),
           std::numeric_limits<double>::infinity(),
           {}};
  }
  if (path_hits_obstacle(smooth.points, inflated_map)) {
   return {Feasibility::Verdict::kInfeasible,
           DepartReason::kCapsViolated,
           std::numeric_limits<double>::infinity(),
           std::numeric_limits<double>::infinity(),
           {}};
  }
  const auto ack = execute_ackermann(smooth.points, smooth.curvature_inv_m, start_theta, cruise_mps, exec);
  if (ack.demand_clipped || ack.traj.empty()) {
   return {Feasibility::Verdict::kInfeasible,
           DepartReason::kCapsViolated,
           std::numeric_limits<double>::infinity(),
           std::numeric_limits<double>::infinity(),
           {}};
  }
  if (traj_hits_obstacle(ack.traj, inflated_map)) {
   return {Feasibility::Verdict::kInfeasible, DepartReason::kCapsViolated,
           std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), ack.traj};
  }
  const double ack_end = ack.traj.back().t_s;
  if (ack_end > horizon + 1e-9) {
   return {Feasibility::Verdict::kUnknown, DepartReason::kHorizonShort,
           std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), ack.traj};
  }
  const double ack_gap = timed_gap(ack.traj, frames, dt, ego_radius_m);
  if (ack_gap >= margin_m) {
   return {Feasibility::Verdict::kFeasible, DepartReason::kClear, ack_end, ack_gap, ack.traj};
  }
  const double ack_spot = timed_gap({{0.0, start}, {horizon, start}}, frames, dt, ego_radius_m);
  if (ack_spot < margin_m) {
   return {Feasibility::Verdict::kInfeasible, DepartReason::kWaitUnsafe, ack_end, ack_gap, ack.traj};
  }
  return {Feasibility::Verdict::kInfeasible, DepartReason::kBlockedInWindow, ack_end, ack_gap, ack.traj};
 }
 const auto spin_exec = execute_turn_drive_turn(route, start_theta, cruise_mps, exec.yaw_rate_rads);
 const double end_s = spin_exec.back().t_s;
 if (end_s > horizon + 1e-9) {
  return {Feasibility::Verdict::kUnknown, DepartReason::kHorizonShort,
          std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), spin_exec};
 }
 const double gap = timed_gap(spin_exec, frames, dt, ego_radius_m);
 if (gap >= margin_m) {
  return {Feasibility::Verdict::kFeasible, DepartReason::kClear, end_s, gap, spin_exec};
 }
 const double spot_gap = timed_gap({{0.0, start}, {horizon, start}}, frames, dt, ego_radius_m);
 if (spot_gap < margin_m) {
  return {Feasibility::Verdict::kInfeasible, DepartReason::kWaitUnsafe, end_s, gap, spin_exec};
 }
 return {Feasibility::Verdict::kInfeasible, DepartReason::kBlockedInWindow, end_s, gap, spin_exec};
}

LegAssessment assess_hold(Vec2 spot, double hold_from_s, double hold_until_s,
                          const std::vector<std::vector<DynamicObjectState>>& frames, double frame_dt_s,
                          double ego_radius_m, double margin_m) {
 if (frames.empty() || hold_from_s < 0.0 || hold_until_s < hold_from_s) {
  return {Feasibility::Verdict::kUnknown,
          DepartReason::kInvalid,
          std::numeric_limits<double>::infinity(),
          std::numeric_limits<double>::infinity(),
          {}};
 }
 const double dt = std::max(1e-6, frame_dt_s);
 const double horizon = static_cast<double>(frames.size() > 0 ? frames.size() - 1 : 0) * dt;
 if (hold_until_s > horizon + 1e-9) {
  return {Feasibility::Verdict::kUnknown,
          DepartReason::kHorizonShort,
          std::numeric_limits<double>::infinity(),
          std::numeric_limits<double>::infinity(),
          {}};
 }
 const double gap = timed_gap({{hold_from_s, spot}, {hold_until_s, spot}}, frames, dt, ego_radius_m);
 if (gap >= margin_m) {
  return {Feasibility::Verdict::kFeasible, DepartReason::kClear, hold_until_s, gap, {}};
 }
 return {Feasibility::Verdict::kInfeasible, DepartReason::kWaitUnsafe, hold_until_s, gap, {}};
}

LegAssessment assess_rejoin(const std::vector<Vec2>& path, std::size_t start_idx, double cruise_mps,
                            double depart_delay_s, const std::vector<std::vector<DynamicObjectState>>& frames,
                            double frame_dt_s, double ego_radius_m, double margin_m) {
 if (path.empty() || frames.empty() || cruise_mps <= 0.0 || depart_delay_s < 0.0) {
  return {Feasibility::Verdict::kUnknown,
          DepartReason::kInvalid,
          std::numeric_limits<double>::infinity(),
          std::numeric_limits<double>::infinity(),
          {}};
 }
 const double dt = std::max(1e-6, frame_dt_s);
 const double horizon = static_cast<double>(frames.size() > 0 ? frames.size() - 1 : 0) * dt;
 const double length = remaining_length(path, start_idx);
 if (depart_delay_s + length / std::max(1e-6, cruise_mps) > horizon + 1e-9) {
  return {Feasibility::Verdict::kUnknown,
          DepartReason::kHorizonShort,
          std::numeric_limits<double>::infinity(),
          std::numeric_limits<double>::infinity(),
          {}};
 }
 const double gap = predicted_gap(path, start_idx, cruise_mps, frames, dt, ego_radius_m, depart_delay_s);
 if (gap >= margin_m) {
  return {Feasibility::Verdict::kFeasible,
          DepartReason::kClear,
          depart_delay_s + length / std::max(1e-6, cruise_mps),
          gap,
          {}};
 }
 // Hold-prefix classification: a violation while still parked at the start
 // means the wait itself kills (WaitUnsafe); otherwise the corridor downstream
 // is what blocks (BlockedInWindow). The hold interval only, not the whole
 // window past departure.
 const Vec2 start = path[std::min(start_idx, path.size() - 1)];
 const double hold_gap = timed_gap({{0.0, start}, {depart_delay_s, start}}, frames, dt, ego_radius_m);
 if (hold_gap < margin_m) {
  return {Feasibility::Verdict::kInfeasible,
          DepartReason::kWaitUnsafe,
          std::numeric_limits<double>::infinity(),
          gap,
          {}};
 }
 return {Feasibility::Verdict::kInfeasible,
         DepartReason::kBlockedInWindow,
         std::numeric_limits<double>::infinity(),
         gap,
         {}};
}

PocketAssessment assess_pocket_wait(const GridMap& inflated_map, Vec2 start, double start_theta, Vec2 pocket,
                                    Vec2 merge, const std::vector<Vec2>& route_suffix, double cruise_mps,
                                    const ChassisExecCaps& exec, double hold_s,
                                    const std::vector<std::vector<DynamicObjectState>>& frames,
                                    double frame_dt_s, double ego_radius_m, double margin_m) {
 PocketAssessment out;
 out.transit = assess_transit(inflated_map, start, start_theta, pocket, cruise_mps, exec, frames, frame_dt_s,
                              ego_radius_m, margin_m);
 // Rejoin geometry: A* pocket to merge, then the given route suffix (whose
 // first point the caller aligns with merge). A failed rejoin plan is proven
 // static infeasibility of that leg.
 std::vector<Vec2> rejoin_pts;
 bool rejoin_planned = false;
 if (std::hypot(merge.x - pocket.x, merge.y - pocket.y) < 1e-9) {
  rejoin_pts = {pocket};
  rejoin_pts.insert(rejoin_pts.end(), route_suffix.begin(), route_suffix.end());
  rejoin_planned = true;
 } else {
  const auto plan = astar_plan(inflated_map, pocket, merge, 1.0);
  if (plan.ok && !plan.path.empty()) {
   rejoin_pts.push_back(pocket);
   for (const auto& point : plan.path)
    rejoin_pts.push_back(point.position);
   rejoin_pts.push_back(merge);
   rejoin_pts.insert(rejoin_pts.end(), route_suffix.begin(), route_suffix.end());
   rejoin_planned = true;
  }
 }
 // Chained schedule (frozen convention): hold starts at the transit nominal
 // end, rejoin departs at hold end. Legs without a schedule (inf end) cannot
 // chain: hold/rejoin inherit the transit cause as Unknown.
 if (out.transit.end_s < std::numeric_limits<double>::infinity()) {
  const double hold_end = out.transit.end_s + std::max(0.0, hold_s);
  out.hold = assess_hold(pocket, out.transit.end_s, hold_end, frames, frame_dt_s, ego_radius_m, margin_m);
  if (!rejoin_planned) {
   out.rejoin = {Feasibility::Verdict::kInfeasible,
                 DepartReason::kStaticBlocked,
                 std::numeric_limits<double>::infinity(),
                 std::numeric_limits<double>::infinity(),
                 {}};
  } else {
   out.rejoin =
     assess_rejoin(rejoin_pts, 0, cruise_mps, hold_end, frames, frame_dt_s, ego_radius_m, margin_m);
  }
 } else {
  const LegAssessment unchained{Feasibility::Verdict::kUnknown,
                                out.transit.reason,
                                std::numeric_limits<double>::infinity(),
                                std::numeric_limits<double>::infinity(),
                                {}};
  out.hold = unchained;
  out.rejoin = unchained;
 }
 if (out.transit.verdict == Feasibility::Verdict::kInfeasible ||
     out.hold.verdict == Feasibility::Verdict::kInfeasible ||
     out.rejoin.verdict == Feasibility::Verdict::kInfeasible) {
  out.overall = Feasibility::Verdict::kInfeasible;
 } else if (out.transit.verdict == Feasibility::Verdict::kUnknown ||
            out.hold.verdict == Feasibility::Verdict::kUnknown ||
            out.rejoin.verdict == Feasibility::Verdict::kUnknown) {
  out.overall = Feasibility::Verdict::kUnknown;
 } else {
  out.overall = Feasibility::Verdict::kFeasible;
 }
 // Chain minimum over legs that produced evidence (nominal model only).
 out.chain_min_gap_m = std::min({out.transit.min_gap_m, out.hold.min_gap_m, out.rejoin.min_gap_m});
 return out;
}

PocketSearch sample_pockets(const GridMap& inflated_map, const std::vector<Vec2>& route, Vec2 start,
                            double start_theta, double cruise_mps, const ChassisExecCaps& exec, double hold_s,
                            const PocketSampleParams& params,
                            const std::vector<std::vector<DynamicObjectState>>& frames, double frame_dt_s,
                            double ego_radius_m, double margin_m) {
 PocketSearch out;
 if (route.size() < 2 || params.max_candidates <= 0 || params.lateral_offsets_m.empty()) {
  return out;
 }
 // Arc-length table for downstream merge selection.
 std::vector<double> arc(route.size(), 0.0);
 for (std::size_t i = 1; i < route.size(); ++i) {
  arc[i] = arc[i - 1] + std::hypot(route[i].x - route[i - 1].x, route[i].y - route[i - 1].y);
 }
 std::set<std::pair<int, int>> tried;
 double next_vertex_s = 0.0;
 for (std::size_t i = 0; i < route.size() && !out.truncated; ++i) {
  if (arc[i] > params.window_m + 1e-9) {
   break;
  }
  if (arc[i] < next_vertex_s - 1e-9) {
   continue;
  }
  next_vertex_s = arc[i] + std::max(0.0, params.vertex_stride_m);
  const Vec2 prev = route[i == 0 ? 0 : i - 1];
  const Vec2 next = route[i + 1 >= route.size() ? route.size() - 1 : i + 1];
  Vec2 dir{next.x - prev.x, next.y - prev.y};
  const double len = std::hypot(dir.x, dir.y);
  if (len < 1e-9) {
   continue;
  }
  dir = {dir.x / len, dir.y / len};
  const Vec2 normal{-dir.y, dir.x};
  for (const double offset : params.lateral_offsets_m) {
   if (out.truncated) {
    break;
   }
   const Vec2 raw{route[i].x + normal.x * offset, route[i].y + normal.y * offset};
   const auto cell = inflated_map.worldToGrid(raw.x, raw.y);
   if (!tried.insert(cell).second || !inflated_map.isFreeCell(cell.first, cell.second)) {
    continue;
   }
   const Vec2 pocket = inflated_map.gridToWorld(cell.first, cell.second);
   if (std::hypot(pocket.x - start.x, pocket.y - start.y) < params.min_pocket_dist_m) {
    continue;
   }
   std::size_t merge_idx = route.size();
   for (std::size_t j = i + 1; j < route.size(); ++j) {
    if (arc[j] - arc[i] >= params.min_downstream_m - 1e-9) {
     merge_idx = j;
     break;
    }
   }
   if (merge_idx >= route.size()) {
    continue;
   }
   PocketCandidate candidate;
   candidate.pocket = pocket;
   candidate.merge = route[merge_idx];
   candidate.assessment = assess_pocket_wait(
     inflated_map, start, start_theta, pocket, route[merge_idx],
     std::vector<Vec2>(route.begin() + static_cast<std::ptrdiff_t>(merge_idx), route.end()), cruise_mps, exec,
     hold_s, frames, frame_dt_s, ego_radius_m, margin_m);
   out.candidates.push_back(std::move(candidate));
   if (static_cast<int>(out.candidates.size()) >= params.max_candidates) {
    out.truncated = true;
   }
  }
 }
 bool any_feasible = false;
 bool any_unknown = false;
 for (const auto& candidate : out.candidates) {
  if (candidate.assessment.overall == Feasibility::Verdict::kFeasible) {
   any_feasible = true;
  } else if (candidate.assessment.overall == Feasibility::Verdict::kUnknown) {
   any_unknown = true;
  }
 }
 if (any_feasible) {
  out.verdict = SearchVerdict::kFoundFeasible;
 } else if (out.candidates.empty() || out.truncated || any_unknown) {
  out.verdict = SearchVerdict::kIncomplete;
 } else {
  out.verdict = SearchVerdict::kAllRefused;
 }
 return out;
}

DepartureAssessment assess_departure(const std::vector<Vec2>& path, std::size_t start_idx, double cruise_mps,
                                     const std::vector<std::vector<DynamicObjectState>>& frames,
                                     double frame_dt_s, double ego_radius_m, double margin_m) {
 if (path.empty() || frames.empty() || cruise_mps <= 0.0) {
  return {std::numeric_limits<double>::infinity(), DepartReason::kInvalid};
 }
 const double dt = std::max(1e-6, frame_dt_s);
 // Last frame stands at (N-1)*dt: predict_frames emits future_s = 0..H
 // inclusive, so N frames cover H = (N-1)*dt seconds of absolute time.
 const double horizon = static_cast<double>(frames.size() > 0 ? frames.size() - 1 : 0) * dt;
 // Full-trajectory gate (reviewer requirement): a wait is only feasible if
 // the ego completes its whole remaining path inside the predicted horizon
 // AND the swept gap passes. A wait that merely pushes the collision past
 // the last frame is undecidable, not safe — skip it.
 double path_length = 0.0;
 for (std::size_t seg = start_idx; seg + 1 < path.size(); ++seg) {
  path_length += std::hypot(path[seg + 1].x - path[seg].x, path[seg + 1].y - path[seg].y);
 }
 constexpr double kSearchStepS = 0.2;
 for (double wait = 0.0; wait <= horizon + 1e-9; wait += kSearchStepS) {
  if (cruise_mps * (horizon - wait) < path_length - 1e-9) {
   continue;
  }
  const double gap = predicted_gap(path, start_idx, cruise_mps, frames, dt, ego_radius_m, wait);
  if (gap >= margin_m) {
   return {wait, wait <= 1e-9 ? DepartReason::kGoNow : DepartReason::kWait};
  }
 }
 // No wait is provably safe. Diagnose why, most urgent first: a swept
 // waiting spot means standing still is what kills (observable inside the
 // window regardless of traversal coverage); an uncovered traversal is a
 // window problem (unknown, not unsafe); otherwise the corridor is genuinely
 // blocked inside a sufficient window.
 const Vec2 start = path[std::min(start_idx, path.size() - 1)];
 const double spot_gap = predicted_gap({start}, 0, 0.0, frames, dt, ego_radius_m, 0.0);
 if (spot_gap < margin_m) {
  return {std::numeric_limits<double>::infinity(), DepartReason::kWaitUnsafe};
 }
 if (path_length > cruise_mps * horizon + 1e-9) {
  return {std::numeric_limits<double>::infinity(), DepartReason::kHorizonShort};
 }
 return {std::numeric_limits<double>::infinity(), DepartReason::kBlockedInWindow};
}

} // namespace sim
