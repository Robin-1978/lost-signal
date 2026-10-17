#include "autonomy/planning/validation/trajectory_validator.hpp"

#include "autonomy/planning/local/predictor.hpp"

#include <algorithm>
#include <cmath>

namespace sim {
namespace {

bool finite_sample(const TrajectorySample& sample) {
 return std::isfinite(sample.t_s) && std::isfinite(sample.position.x) && std::isfinite(sample.position.y) &&
        std::isfinite(sample.heading_rad) && std::isfinite(sample.forward_mps) &&
        std::isfinite(sample.lateral_mps) && std::isfinite(sample.yaw_rate_radps) &&
        std::isfinite(sample.steering_angle_rad);
}

bool finite_command(const DwaCmd& command) {
 return std::isfinite(command.v) && std::isfinite(command.w) && std::isfinite(command.score) &&
        std::isfinite(command.lateral_v);
}

TrajectorySample sample_at(const ExecutableTrajectory& trajectory, double time_s) {
 const auto after = std::lower_bound(
 trajectory.samples.begin(), trajectory.samples.end(), time_s,
 [](const TrajectorySample& sample, double value) { return sample.t_s < value; });
 if (after == trajectory.samples.begin())
 return *after;
 if (after == trajectory.samples.end())
 return trajectory.samples.back();
 const auto& b = *after;
 const auto& a = *(after - 1);
 const double blend = (time_s - a.t_s) / std::max(1e-9, b.t_s - a.t_s);
 return {
 time_s,
 {a.position.x + (b.position.x - a.position.x) * blend,
 a.position.y + (b.position.y - a.position.y) * blend},
 wrap_angle(a.heading_rad + wrap_angle(b.heading_rad - a.heading_rad) * blend),
 a.forward_mps + (b.forward_mps - a.forward_mps) * blend,
 a.lateral_mps + (b.lateral_mps - a.lateral_mps) * blend,
 a.yaw_rate_radps + (b.yaw_rate_radps - a.yaw_rate_radps) * blend,
 a.steering_angle_rad + (b.steering_angle_rad - a.steering_angle_rad) * blend,
 };
}

std::vector<TrajectorySample> emergency_stop_samples(
 const ExecutableTrajectory& trajectory, double delay_s,
 double decel_mps2, double integration_step_s) {
 const double start_s = std::clamp(
 delay_s, 0.0, trajectory.samples.back().t_s);
 std::vector<TrajectorySample> out{sample_at(trajectory, start_s)};
 auto state = out.front();
 for (int step = 0; step < 10000 && std::abs(state.forward_mps) > 1e-6; ++step) {
 const double duration_s = std::min(
 integration_step_s, std::abs(state.forward_mps) / decel_mps2);
 const double next_speed = state.forward_mps > 0.0
 ? std::max(0.0, state.forward_mps - decel_mps2 * duration_s)
 : std::min(0.0, state.forward_mps + decel_mps2 * duration_s);
 // Symmetric wheel braking preserves instantaneous curvature while speed
 // decays. Once translation reaches zero, residual spin does not change the
 // circular RO-1 collision footprint.
 const double ratio = std::abs(state.forward_mps) > 1e-9
 ? next_speed / state.forward_mps
 : 0.0;
 const double next_yaw_rate = state.yaw_rate_radps * ratio;
 const double average_speed = 0.5 * (state.forward_mps + next_speed);
 const double average_yaw_rate = 0.5 * (state.yaw_rate_radps + next_yaw_rate);
 const double middle_heading = state.heading_rad + 0.5 * average_yaw_rate * duration_s;
 state.position.x += average_speed * std::cos(middle_heading) * duration_s;
 state.position.y += average_speed * std::sin(middle_heading) * duration_s;
 state.heading_rad = wrap_angle(state.heading_rad + average_yaw_rate * duration_s);
 state.t_s += duration_s;
 state.forward_mps = next_speed;
 state.yaw_rate_radps = next_yaw_rate;
 out.push_back(state);
 }
 // A safe stop is a hold, not only the instant at which velocity reaches
 // zero. Keep the final pose occupied through the nominal action horizon so
 // an approaching actor cannot make "brake now" look safe by arriving one
 // sample after the vehicle stops.
 while (state.t_s + 1e-9 < trajectory.samples.back().t_s) {
 state.t_s = std::min(
 trajectory.samples.back().t_s, state.t_s + integration_step_s);
 state.forward_mps = 0.0;
 state.lateral_mps = 0.0;
 state.yaw_rate_radps = 0.0;
 out.push_back(state);
 }
 return out;
}

bool segment_is_free(const GridMap& map, Vec2 from, Vec2 to) {
 const double length = std::hypot(to.x - from.x, to.y - from.y);
 const int subdivisions = std::max(1, static_cast<int>(std::ceil(length / (0.5 * map.res()))));
 for (int step = 0; step <= subdivisions; ++step) {
  const double blend = static_cast<double>(step) / subdivisions;
  if (!map.isFree(from.x + (to.x - from.x) * blend, from.y + (to.y - from.y) * blend))
   return false;
 }
 return true;
}

double clearance_at(Vec2 ego, double time_s, const std::vector<std::vector<DynamicObjectState>>& frames,
                    double frame_dt_s, double ego_radius_m) {
 if (frames.empty())
  return std::numeric_limits<double>::infinity();
 const double frame_position = std::max(0.0, time_s / frame_dt_s);
 const std::size_t before = std::min(frames.size() - 1, static_cast<std::size_t>(std::floor(frame_position)));
 const std::size_t after = std::min(frames.size() - 1, before + 1);
 const double blend = std::clamp(frame_position - static_cast<double>(before), 0.0, 1.0);
 double clearance = std::numeric_limits<double>::infinity();
 const std::size_t common = std::min(frames[before].size(), frames[after].size());
 for (std::size_t actor = 0; actor < common; ++actor) {
  const auto& a = frames[before][actor];
  const auto& b = frames[after][actor];
  const Vec2 position{a.p.x + (b.p.x - a.p.x) * blend, a.p.y + (b.p.y - a.p.y) * blend};
 const double radius = std::max(a.occupied_radius_m(), b.occupied_radius_m());
  clearance = std::min(clearance, std::hypot(ego.x - position.x, ego.y - position.y) - ego_radius_m - radius);
 }
 return clearance;
}

} // namespace

const char* trajectory_verdict_name(TrajectoryVerdict verdict) {
 switch (verdict) {
 case TrajectoryVerdict::Accepted:
  return "accepted";
 case TrajectoryVerdict::MinimumRiskEscape:
  return "minimum-risk-escape";
 case TrajectoryVerdict::CommittedTransit:
 return "committed-transit";
 case TrajectoryVerdict::InvalidStructure:
  return "invalid-structure";
 case TrajectoryVerdict::StaticCollision:
  return "static-collision";
 case TrajectoryVerdict::DynamicCollision:
  return "dynamic-collision";
 case TrajectoryVerdict::PredictionHorizonShort:
  return "prediction-horizon-short";
 case TrajectoryVerdict::UnsafeEmergencyStop:
 return "unsafe-emergency-stop";
 case TrajectoryVerdict::BehaviorConstraint:
 return "behavior-constraint";
 }
 return "unknown";
}

TrajectoryValidation
validate_executable_trajectory(const ExecutableTrajectory& trajectory, const GridMap& planning_map,
                               const std::vector<std::vector<DynamicObjectState>>& predicted_frames,
                               double prediction_frame_dt_s, double robot_radius_m, double required_margin_m,
 StaticFeasibility terrain_feasible, const TrajectoryConstraints& constraints) {
 TrajectoryValidation out;
 const auto accept_committed_transit = [&]() {
 if (!constraints.committed_transit ||
 trajectory.command.v + 1e-9 < constraints.committed_progress_floor_mps)
 return false;
 out.verdict = TrajectoryVerdict::CommittedTransit;
 out.diagnostic =
 "margin-compliant committed transit is safer than braking inside the conflict";
 return true;
 };
 if (!trajectory.command.valid || !finite_command(trajectory.command) || trajectory.samples.size() < 2 ||
     std::abs(trajectory.samples.front().t_s) > 1e-12 || !(prediction_frame_dt_s > 0.0)) {
  out.diagnostic = "trajectory must contain a valid command and start at t=0";
  return out;
 }
 for (std::size_t index = 0; index < trajectory.samples.size(); ++index) {
  const auto& sample = trajectory.samples[index];
  if (!finite_sample(sample) || (index > 0 && !(sample.t_s > trajectory.samples[index - 1].t_s))) {
   out.diagnostic = "trajectory samples must be finite and strictly time ordered";
   return out;
  }
  const Vec2 previous = index > 0 ? trajectory.samples[index - 1].position : sample.position;
  if (!segment_is_free(planning_map, previous, sample.position)) {
   out.verdict = TrajectoryVerdict::StaticCollision;
   out.diagnostic = "trajectory segment enters the inflated static map";
   return out;
  }
  if (index > 0 && terrain_feasible &&
      !terrain_feasible(trajectory.samples[index - 1].position, sample.position)) {
   out.verdict = TrajectoryVerdict::StaticCollision;
   out.diagnostic = "trajectory violates terrain feasibility";
   return out;
  }
 if (constraints.stop_plane_active) {
 const double initial_longitudinal =
 (trajectory.samples.front().position.x - constraints.stop_point.x) * constraints.stop_direction.x +
 (trajectory.samples.front().position.y - constraints.stop_point.y) * constraints.stop_direction.y;
 const double allowed_longitudinal =
 std::max(constraints.stop_tolerance_m, initial_longitudinal);
 const double sample_longitudinal =
 (sample.position.x - constraints.stop_point.x) * constraints.stop_direction.x +
 (sample.position.y - constraints.stop_point.y) * constraints.stop_direction.y;
 // If lag already put the body beyond the plane, permit only non-worsening
 // recovery. Otherwise no local score can grant entry before commitment.
 if (sample_longitudinal > allowed_longitudinal + 1e-9) {
 out.verdict = TrajectoryVerdict::BehaviorConstraint;
 out.diagnostic = "trajectory crosses the active behavior stop plane";
 return out;
 }
 }
 }

 if (!predicted_frames.empty()) {
  const double covered_s = static_cast<double>(predicted_frames.size() - 1) * prediction_frame_dt_s;
  if (trajectory.samples.back().t_s > covered_s + 1e-9) {
   out.verdict = TrajectoryVerdict::PredictionHorizonShort;
   out.diagnostic = "prediction does not cover the executable trajectory";
   return out;
  }
  std::vector<TimedPose> timed;
  timed.reserve(trajectory.samples.size());
  for (const auto& sample : trajectory.samples)
   timed.push_back({sample.t_s, sample.position});
  out.min_dynamic_clearance_m = timed_gap(timed, predicted_frames, prediction_frame_dt_s, robot_radius_m);
  if (out.min_dynamic_clearance_m + 1e-9 < required_margin_m) {
   const double initial_clearance = clearance_at(trajectory.samples.front().position, 0.0, predicted_frames,
                                                 prediction_frame_dt_s, robot_radius_m);
   const double terminal_clearance =
     clearance_at(trajectory.samples.back().position, trajectory.samples.back().t_s, predicted_frames,
                  prediction_frame_dt_s, robot_radius_m);
   const double displacement =
     std::hypot(trajectory.samples.back().position.x - trajectory.samples.front().position.x,
                trajectory.samples.back().position.y - trajectory.samples.front().position.y);
   // Once already inside a soft margin, stopping may be the unsafe action.
 // Accept a separately named escape only when physical overlap never occurs
 // and the endpoint is measurably safer. An approaching actor can make
 // clearance shrink briefly even while the ego takes the only collision-free
 // exit, so monotonic separation is too strong once the soft margin is
 // already violated.
   if (initial_clearance + 1e-9 < required_margin_m && out.min_dynamic_clearance_m >= -1e-9 &&
 displacement >= 0.05 && terminal_clearance >= initial_clearance + 0.05) {
    out.verdict = TrajectoryVerdict::MinimumRiskEscape;
    out.diagnostic = "trajectory escapes an existing soft-margin incursion";
    return out;
   }
   out.verdict = TrajectoryVerdict::DynamicCollision;
   out.diagnostic = "same-time swept clearance is below the required margin";
   return out;
  }
 }

 if (constraints.emergency_decel_mps2 > 0.0 &&
 trajectory.command_space == LocalCommandSpace::DifferentialTwist) {
 if (constraints.emergency_stop_delay_s > trajectory.samples.back().t_s + 1e-9) {
 if (accept_committed_transit())
 return out;
 out.verdict = TrajectoryVerdict::PredictionHorizonShort;
 out.diagnostic = "trajectory does not cover the emergency-stop response delay";
 return out;
 }
 const auto stop = emergency_stop_samples(
 trajectory, constraints.emergency_stop_delay_s,
 constraints.emergency_decel_mps2, prediction_frame_dt_s);
 for (std::size_t index = 1; index < stop.size(); ++index) {
 if (!segment_is_free(planning_map, stop[index - 1].position, stop[index].position) ||
 (terrain_feasible && !terrain_feasible(
 stop[index - 1].position, stop[index].position))) {
 if (accept_committed_transit())
 return out;
 out.verdict = TrajectoryVerdict::UnsafeEmergencyStop;
 out.diagnostic = "emergency braking tail enters static or infeasible terrain";
 return out;
 }
 }
 if (!predicted_frames.empty()) {
 const double covered_s = static_cast<double>(predicted_frames.size() - 1) *
 prediction_frame_dt_s;
 if (stop.back().t_s > covered_s + 1e-9) {
 if (accept_committed_transit())
 return out;
 out.verdict = TrajectoryVerdict::PredictionHorizonShort;
 out.diagnostic = "prediction does not cover the emergency braking tail";
 return out;
 }
 std::vector<TimedPose> timed_stop;
 timed_stop.reserve(stop.size());
 for (const auto& sample : stop)
 timed_stop.push_back({sample.t_s, sample.position});
 const double stopping_gap = timed_gap(
 timed_stop, predicted_frames, prediction_frame_dt_s, robot_radius_m);
 const double nominal_gap = out.min_dynamic_clearance_m;
 out.min_dynamic_clearance_m = std::min(
 out.min_dynamic_clearance_m, stopping_gap);
 const double initial_clearance = clearance_at(
 trajectory.samples.front().position, 0.0, predicted_frames,
 prediction_frame_dt_s, robot_radius_m);
 // Existing soft-margin incursions use the separately named escape
 // contract above; braking in place can be less safe than leaving them.
 if (initial_clearance + 1e-9 >= required_margin_m &&
 stopping_gap + 1e-9 < required_margin_m) {
 if (accept_committed_transit())
 return out;
 const double terminal_clearance = clearance_at(
 trajectory.samples.back().position, trajectory.samples.back().t_s,
 predicted_frames, prediction_frame_dt_s, robot_radius_m);
 const double displacement = std::hypot(
 trajectory.samples.back().position.x - trajectory.samples.front().position.x,
 trajectory.samples.back().position.y - trajectory.samples.front().position.y);
 if (nominal_gap + 1e-9 >= required_margin_m &&
 displacement >= 0.05 && terminal_clearance >= initial_clearance + 0.05) {
 out.min_dynamic_clearance_m = nominal_gap;
 out.verdict = TrajectoryVerdict::MinimumRiskEscape;
 out.diagnostic =
 "margin-compliant motion escapes a threat that would invade the emergency hold pose";
 return out;
 }
 out.verdict = TrajectoryVerdict::UnsafeEmergencyStop;
 out.diagnostic = "emergency braking tail violates dynamic clearance";
 return out;
 }
 }
 }

 out.verdict = TrajectoryVerdict::Accepted;
 out.diagnostic = "accepted";
 return out;
}

ValidatedTrajectorySelection
select_executable_trajectory(const DwaResult& proposals, const GridMap& planning_map,
 const std::vector<std::vector<DynamicObjectState>>& predicted_frames,
 double prediction_frame_dt_s, double robot_radius_m, double required_margin_m,
 StaticFeasibility terrain_feasible, const TrajectoryConstraints& constraints) {
 ValidatedTrajectorySelection out;
 int best_rank = 0;
 double best_score = -std::numeric_limits<double>::infinity();
 for (const auto& rollout : proposals.rollouts) {
 if (rollout.collision || !rollout.trajectory.command.valid)
 continue;
 const auto validation = validate_executable_trajectory(
 rollout.trajectory, planning_map, predicted_frames, prediction_frame_dt_s,
 robot_radius_m, required_margin_m, terrain_feasible, constraints);
 if (!validation.accepted())
 continue;
 ++out.accepted_candidates;
 const int rank = validation.verdict == TrajectoryVerdict::Accepted ? 2 : 1;
 if (rank < best_rank || (rank == best_rank && rollout.trajectory.command.score <= best_score))
 continue;
 best_rank = rank;
 best_score = rollout.trajectory.command.score;
 out.trajectory = rollout.trajectory;
 out.validation = validation;
 }
 if (out.accepted_candidates == 0) {
 out.validation = validate_executable_trajectory(
 proposals.best, planning_map, predicted_frames, prediction_frame_dt_s,
 robot_radius_m, required_margin_m, terrain_feasible, constraints);
 }
 return out;
}

} // namespace sim
