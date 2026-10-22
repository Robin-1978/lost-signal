#include "robot/autonomy/planning/validation/trajectory_validator.hpp"

#include "robot/autonomy/planning/local/predictor.hpp"

#include <algorithm>
#include <cmath>

namespace sim {
namespace {

constexpr double kGravityMps2 = 9.81;

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

Vec2 timed_pose_at(const std::vector<TimedPose>& trajectory, double time_s) {
 const auto after = std::lower_bound(
 trajectory.begin(), trajectory.end(), time_s,
 [](const TimedPose& pose, double value) { return pose.t_s < value; });
 if (after == trajectory.begin())
 return after->p;
 if (after == trajectory.end())
 return trajectory.back().p;
 const auto& b = *after;
 const auto& a = *(after - 1);
 const double blend = (time_s - a.t_s) /
 std::max(1e-9, b.t_s - a.t_s);
 return {
 a.p.x + (b.p.x - a.p.x) * blend,
 a.p.y + (b.p.y - a.p.y) * blend,
 };
}

double timed_heading_at(
 const std::vector<TimedPose>& trajectory, double time_s,
 double sample_interval_s) {
 if (trajectory.size() < 2)
 return 0.0;
 const double half_interval_s = std::max(1e-3, 0.5 * sample_interval_s);
 const Vec2 before = timed_pose_at(
 trajectory, std::max(trajectory.front().t_s, time_s - half_interval_s));
 const Vec2 after = timed_pose_at(
 trajectory, std::min(trajectory.back().t_s, time_s + half_interval_s));
 if (std::hypot(after.x - before.x, after.y - before.y) > 1e-9)
 return std::atan2(after.y - before.y, after.x - before.x);
 // During a dwell, use the next non-zero leg as the pose contract. This is
 // the direction the body must acquire before translation resumes.
 const auto next = std::upper_bound(
 trajectory.begin(), trajectory.end(), time_s,
 [](double value, const TimedPose& pose) { return value < pose.t_s; });
 for (auto cursor = next; cursor != trajectory.end(); ++cursor) {
 if (std::hypot(cursor->p.x - before.x, cursor->p.y - before.y) > 1e-9)
 return std::atan2(cursor->p.y - before.y, cursor->p.x - before.x);
 }
 for (auto cursor = next; cursor != trajectory.begin();) {
 --cursor;
 if (std::hypot(after.x - cursor->p.x, after.y - cursor->p.y) > 1e-9)
 return std::atan2(after.y - cursor->p.y, after.x - cursor->p.x);
 }
 return 0.0;
}

double timed_forward_speed_at(const std::vector<TimedPose>& trajectory,
 double time_s) {
 if (trajectory.size() < 2)
 return 0.0;
 // Use the right-hand segment at an exact phase boundary. Stop-turn-go
 // schedules encode a corner as a stationary dwell after the arriving leg;
 // using the left derivative there would tell a settled executor to retain
 // the speed of the leg it has already completed.
 auto next = std::upper_bound(
 trajectory.begin(), trajectory.end(), time_s + 1e-12,
 [](double value, const TimedPose& pose) { return value < pose.t_s; });
 if (next == trajectory.begin())
 next = trajectory.begin() + 1;
 if (next == trajectory.end())
 return 0.0;
 const auto& before = *(next - 1);
 const double duration_s = next->t_s - before.t_s;
 if (!(duration_s > 1e-12))
 return 0.0;
 return std::hypot(next->p.x - before.p.x, next->p.y - before.p.y) /
 duration_s;
}

struct EmergencyStopTrajectory {
 std::vector<TrajectorySample> samples;
 bool complete{true};
};

EmergencyStopTrajectory emergency_stop_samples(
 const ExecutableTrajectory& trajectory, double delay_s,
 double decel_mps2, double integration_step_s,
 double differential_track_m, double hold_until_s,
 const SafeStopDecelQuery& decel_query) {
 const double start_s = std::clamp(
 delay_s, 0.0, trajectory.samples.back().t_s);
 EmergencyStopTrajectory result;
 result.samples.push_back(sample_at(trajectory, start_s));
 auto state = result.samples.front();
 double left_mps = state.forward_mps;
 double right_mps = state.forward_mps;
 if (differential_track_m > 0.0) {
 left_mps -= state.yaw_rate_radps * differential_track_m * 0.5;
 right_mps += state.yaw_rate_radps * differential_track_m * 0.5;
 }
 const auto toward_zero = [](double value, double decel, double duration_s) {
 return value > 0.0
 ? std::max(0.0, value - decel * duration_s)
 : std::min(0.0, value + decel * duration_s);
 };
 for (int step = 0; step < 10000 &&
 std::max(std::abs(left_mps), std::abs(right_mps)) > 1e-6; ++step) {
 double local_decel_mps2 = decel_query
 ? decel_query(state.position, state.heading_rad)
 : decel_mps2;
 // Also sample the no-braking endpoint of this integration interval. Taking
 // the smaller authority prevents a step that crosses onto a steep or slick
 // patch from spending the whole interval with the previous patch's braking
 // capability.
 if (decel_query) {
 const double current_yaw_rate = differential_track_m > 0.0
 ? (right_mps - left_mps) / differential_track_m
 : state.yaw_rate_radps;
 const double projected_heading = state.heading_rad +
 0.5 * current_yaw_rate * integration_step_s;
 const Vec2 projected_position{
 state.position.x + state.forward_mps * std::cos(projected_heading) * integration_step_s,
 state.position.y + state.forward_mps * std::sin(projected_heading) * integration_step_s,
 };
 local_decel_mps2 = std::min(
 local_decel_mps2, decel_query(projected_position, projected_heading));
 }
 if (!(local_decel_mps2 > 1e-9) || !std::isfinite(local_decel_mps2)) {
 result.complete = false;
 return result;
 }
 const double duration_s = std::min(
 integration_step_s,
 std::max(std::abs(left_mps), std::abs(right_mps)) / local_decel_mps2);
 const double next_left_mps = toward_zero(left_mps, local_decel_mps2, duration_s);
 const double next_right_mps = toward_zero(right_mps, local_decel_mps2, duration_s);
 const double current_speed = 0.5 * (left_mps + right_mps);
 const double next_speed = 0.5 * (next_left_mps + next_right_mps);
 const double current_yaw_rate = differential_track_m > 0.0
 ? (right_mps - left_mps) / differential_track_m
 : state.yaw_rate_radps;
 const double next_yaw_rate = differential_track_m > 0.0
 ? (next_right_mps - next_left_mps) / differential_track_m
 : (std::abs(state.forward_mps) > 1e-9
 ? state.yaw_rate_radps * next_speed / state.forward_mps
 : 0.0);
 const double average_speed = 0.5 * (current_speed + next_speed);
 const double average_yaw_rate = 0.5 * (current_yaw_rate + next_yaw_rate);
 const double middle_heading = state.heading_rad + 0.5 * average_yaw_rate * duration_s;
 state.position.x += average_speed * std::cos(middle_heading) * duration_s;
 state.position.y += average_speed * std::sin(middle_heading) * duration_s;
 state.heading_rad = wrap_angle(state.heading_rad + average_yaw_rate * duration_s);
 state.t_s += duration_s;
 state.forward_mps = next_speed;
 state.yaw_rate_radps = next_yaw_rate;
 left_mps = next_left_mps;
 right_mps = next_right_mps;
 result.samples.push_back(state);
 }
 if (std::max(std::abs(left_mps), std::abs(right_mps)) > 1e-6) {
 result.complete = false;
 return result;
 }
 // A safe stop is a hold, not only the instant at which velocity reaches
 // zero. Keep the final pose occupied through the nominal action horizon so
 // an approaching actor cannot make "brake now" look safe by arriving one
 // sample after the vehicle stops.
 while (state.t_s + 1e-9 < hold_until_s) {
 state.t_s = std::min(
 hold_until_s, state.t_s + integration_step_s);
 state.forward_mps = 0.0;
 state.lateral_mps = 0.0;
 state.yaw_rate_radps = 0.0;
 result.samples.push_back(state);
 }
 return result;
}

double point_segment_distance(Vec2 point, Vec2 from, Vec2 to) {
 const Vec2 delta{to.x - from.x, to.y - from.y};
 const double length_sq = delta.x * delta.x + delta.y * delta.y;
 if (length_sq <= 1e-12)
 return std::hypot(point.x - from.x, point.y - from.y);
 const double blend = std::clamp(
 ((point.x - from.x) * delta.x + (point.y - from.y) * delta.y) /
 length_sq, 0.0, 1.0);
 return std::hypot(
 point.x - from.x - blend * delta.x,
 point.y - from.y - blend * delta.y);
}

double reference_distance(Vec2 point, const std::vector<Vec2>& reference) {
 if (reference.empty())
 return std::numeric_limits<double>::infinity();
 if (reference.size() == 1)
 return std::hypot(point.x - reference.front().x, point.y - reference.front().y);
 double best = std::numeric_limits<double>::infinity();
 for (std::size_t index = 0; index + 1 < reference.size(); ++index) {
 best = std::min(best, point_segment_distance(
 point, reference[index], reference[index + 1]));
 }
 return best;
}

double segment_max_reference_distance(Vec2 from, Vec2 to,
 const std::vector<Vec2>& reference, double sample_spacing_m) {
 const double length = std::hypot(to.x - from.x, to.y - from.y);
 const int subdivisions = std::max(
 1, static_cast<int>(std::ceil(length / std::max(1e-3, sample_spacing_m))));
 double maximum = 0.0;
 for (int step = 0; step <= subdivisions; ++step) {
 const double blend = static_cast<double>(step) / subdivisions;
 maximum = std::max(maximum, reference_distance(
 {from.x + (to.x - from.x) * blend,
 from.y + (to.y - from.y) * blend}, reference));
 }
 return maximum;
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

double stability_risk(const TrajectorySample* previous_sample,
 const TrajectorySample& sample, const TrajectoryConstraints& constraints) {
 const TerrainSample terrain = constraints.terrain_sample(sample.position);
 const double longitudinal = longitudinal_grade(terrain, sample.heading_rad);
 const double cross_grade = lateral_grade(terrain, sample.heading_rad);
 double forward_accel_mps2 = 0.0;
 double lateral_accel_mps2 = sample.forward_mps * sample.yaw_rate_radps;
 if (previous_sample) {
 const double duration_s = sample.t_s - previous_sample->t_s;
 forward_accel_mps2 =
 (sample.forward_mps - previous_sample->forward_mps) / duration_s -
 sample.yaw_rate_radps * sample.lateral_mps;
 lateral_accel_mps2 +=
 (sample.lateral_mps - previous_sample->lateral_mps) / duration_s;
 }
 const double bump_accel_mps2 = roughness_lateral_acceleration(
 terrain.roughness, std::hypot(sample.forward_mps, sample.lateral_mps),
 constraints.support_width_m);
 const double plane_scale = std::sqrt(
 1.0 + terrain.grade_x * terrain.grade_x + terrain.grade_y * terrain.grade_y);
 const double normal_accel_mps2 = kGravityMps2 / plane_scale;
 const double longitudinal_load_mps2 =
 kGravityMps2 * longitudinal / plane_scale + forward_accel_mps2;
 const double lateral_load_mps2 =
 kGravityMps2 * cross_grade / plane_scale + lateral_accel_mps2;
 const Vec2 unloaded_projection{constraints.center_of_mass_x_m, 0.0};
 const auto projected_risk = [&](double signed_bump_mps2) {
 const Vec2 load_projection{
 constraints.center_of_mass_x_m - constraints.center_of_mass_height_m *
 longitudinal_load_mps2 / normal_accel_mps2,
 -constraints.center_of_mass_height_m *
 (lateral_load_mps2 + signed_bump_mps2) / normal_accel_mps2,
 };
 return support_polygon_risk(
 load_projection, unloaded_projection, constraints.support_polygon);
 };
 // Roughness has no authored phase. Test both signs instead of inventing a
 // deterministic pose disturbance that could accidentally improve margin.
 return std::max(
 projected_risk(bump_accel_mps2), projected_risk(-bump_accel_mps2));
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
 case TrajectoryVerdict::DynamicStabilityViolation:
 return "dynamic-stability-violation";
 }
 return "unknown";
}

TrajectoryValidation
validate_executable_trajectory(const ExecutableTrajectory& trajectory, const GridMap& planning_map,
                               const std::vector<std::vector<DynamicObjectState>>& predicted_frames,
                               double prediction_frame_dt_s, double robot_radius_m, double required_margin_m,
 StaticFeasibility terrain_feasible, const TrajectoryConstraints& constraints) {
 TrajectoryValidation out;
 bool nominal_dynamic_violation = false;
 double nominal_terminal_clearance_m =
 std::numeric_limits<double>::infinity();
 const bool dynamic_objects_present = std::any_of(
 predicted_frames.begin(), predicted_frames.end(),
 [](const auto& frame) { return !frame.empty(); });
 const bool stability_active = constraints.terrain_sample &&
 constraints.support_width_m > 0.0 &&
 constraints.support_polygon.size() >= 3 &&
 std::isfinite(constraints.center_of_mass_x_m) &&
 constraints.center_of_mass_height_m > 0.0 &&
 constraints.max_dynamic_rollover_risk > 0.0;
 const bool committed_corridor_active =
 constraints.committed_reference_path.size() >= 2 &&
 std::isfinite(constraints.committed_tracking_corridor_m) &&
 constraints.committed_tracking_corridor_m > 0.0;
 const bool committed_timing_active = committed_corridor_active &&
 constraints.committed_reference_trajectory.size() >= 2 &&
 std::abs(constraints.committed_reference_trajectory.front().t_s) <= 1e-12;
 if ((constraints.terrain_sample || constraints.support_width_m > 0.0 ||
 !constraints.support_polygon.empty() || constraints.center_of_mass_x_m != 0.0 ||
 constraints.center_of_mass_height_m > 0.0 ||
 constraints.max_dynamic_rollover_risk > 0.0) &&
 !stability_active) {
 out.diagnostic = "dynamic stability requires terrain, support width, COM height and risk limit";
 return out;
 }
 const auto has_contingency_progress = [&]() {
 const bool committed_forward = constraints.committed_transit &&
 constraints.committed_progress_floor_mps > 0.0 &&
 trajectory.command.v > 1e-9 &&
 trajectory.command.v + 1e-9 >= constraints.committed_progress_floor_mps;
 const bool emergency_reverse = constraints.dynamic_reverse_escape &&
 trajectory.command.v <= -constraints.dynamic_reverse_speed_floor_mps + 1e-9;
 return committed_forward || emergency_reverse;
 };
 const auto accept_committed_transit = [&]() {
 if (!has_contingency_progress())
 return false;
 out.verdict = TrajectoryVerdict::CommittedTransit;
 out.diagnostic = constraints.dynamic_reverse_escape
 ? "validated reverse escape is safer than being struck while stopped"
 : nominal_dynamic_violation
 ? "committed transit is the lower-risk exit after the dynamic forecast changed"
 : "margin-compliant committed transit is safer than braking inside the conflict";
 return true;
 };
 if (!trajectory.command.valid || !finite_command(trajectory.command) || trajectory.samples.size() < 2 ||
     std::abs(trajectory.samples.front().t_s) > 1e-12 || !(prediction_frame_dt_s > 0.0)) {
  out.diagnostic = "trajectory must contain a valid command and start at t=0";
  return out;
 }
 const double initial_reference_distance_m = committed_corridor_active
 ? reference_distance(
 trajectory.samples.front().position, constraints.committed_reference_path)
 : 0.0;
 const double allowed_reference_distance_m = std::max(
 constraints.committed_tracking_corridor_m, initial_reference_distance_m);
 for (std::size_t index = 0; index < trajectory.samples.size(); ++index) {
  const auto& sample = trajectory.samples[index];
  if (!finite_sample(sample) || (index > 0 && !(sample.t_s > trajectory.samples[index - 1].t_s))) {
   out.diagnostic = "trajectory samples must be finite and strictly time ordered";
   return out;
  }
  const Vec2 previous = index > 0 ? trajectory.samples[index - 1].position : sample.position;
 // DWA publishes only the next control interval and replans after it. Its
 // later constant-command rollout is a collision prediction, not a promise
 // to hold one curvature through every corner in the behavior polyline.
 // Enforce the admission tube over the command commitment interval; the next
 // cycle advances the same invariant from the newly measured pose.
 const double segment_reference_distance_m = committed_corridor_active
 ? segment_max_reference_distance(
 previous, sample.position, constraints.committed_reference_path,
 0.5 * planning_map.res())
 : 0.0;
 if (committed_corridor_active && sample.t_s <= prediction_frame_dt_s + 1e-9 &&
 segment_reference_distance_m > allowed_reference_distance_m + 1e-9) {
 out.verdict = TrajectoryVerdict::BehaviorConstraint;
 out.diagnostic =
 "trajectory leaves the spatial corridor proved at crossing admission: " +
 std::to_string(segment_reference_distance_m) + " m exceeds " +
 std::to_string(allowed_reference_distance_m) + " m";
 return out;
 }
 if (!planning_map.segmentIsFree(previous, sample.position)) {
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
 if (!constraints.live_obstacle_points_in_base.empty()) {
 const auto& origin = trajectory.samples.front();
 const auto relative_point = [&](const auto& candidate) {
 const double dx = candidate.position.x - origin.position.x;
 const double dy = candidate.position.y - origin.position.y;
 return Vec2{
 std::cos(origin.heading_rad) * dx +
 std::sin(origin.heading_rad) * dy,
 -std::sin(origin.heading_rad) * dx +
 std::cos(origin.heading_rad) * dy,
 };
 };
 const Vec2 previous_relative = index > 0
 ? relative_point(trajectory.samples[index - 1])
 : relative_point(sample);
 const Vec2 current_relative = relative_point(sample);
 const double live_limit_m = robot_radius_m +
 std::max(0.0, constraints.live_obstacle_margin_m);
 for (const auto& point : constraints.live_obstacle_points_in_base) {
 if (!std::isfinite(point.x) || !std::isfinite(point.y))
 continue;
 if (point_segment_distance(point, previous_relative,
 current_relative) < live_limit_m) {
 out.verdict = TrajectoryVerdict::StaticCollision;
 out.diagnostic =
 "trajectory enters a live sensor-frame obstacle";
 return out;
 }
 }
 }
 if (stability_active) {
 const auto* prior = index > 0 ? &trajectory.samples[index - 1] : nullptr;
 const double risk = stability_risk(prior, sample, constraints);
 out.max_dynamic_rollover_risk = std::max(out.max_dynamic_rollover_risk, risk);
 if (!std::isfinite(risk) || risk > constraints.max_dynamic_rollover_risk + 1e-12) {
 out.verdict = TrajectoryVerdict::DynamicStabilityViolation;
 out.diagnostic = "trajectory exceeds the declared dynamic rollover envelope";
 return out;
 }
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

 if (committed_corridor_active &&
 initial_reference_distance_m > constraints.committed_tracking_corridor_m + 1e-9 &&
 reference_distance(
 sample_at(trajectory, std::min(
 prediction_frame_dt_s, trajectory.samples.back().t_s)).position,
 constraints.committed_reference_path) >
 initial_reference_distance_m + 1e-9) {
 out.verdict = TrajectoryVerdict::BehaviorConstraint;
 out.diagnostic = "trajectory worsens an existing committed-corridor deviation";
 return out;
 }

 // DWA authorizes only the next control interval. At a stop-turn-go corner,
 // its constant-command rollout ends in a hold even though the committed
 // behavior proof continues along the next leg. Reconstruct that exact
 // receding-horizon contract: execute this candidate for one interval, then
 // consume the remaining latched timed reference. This is also the fresh
 // same-time recheck against the current actor forecast.
 bool committed_timed_continuation_safe = false;
 double committed_timed_gap_m = -std::numeric_limits<double>::infinity();
 std::string committed_timing_diagnostic;
 if (committed_timing_active) {
 bool valid_timing = true;
 const auto& reference = constraints.committed_reference_trajectory;
 for (std::size_t index = 1; index < reference.size(); ++index) {
 if (!std::isfinite(reference[index].t_s) ||
 !(reference[index].t_s > reference[index - 1].t_s) ||
 !std::isfinite(reference[index].p.x) ||
 !std::isfinite(reference[index].p.y)) {
 valid_timing = false;
 committed_timing_diagnostic =
 "committed timed reference is not finite and strictly time ordered";
 break;
 }
 }
 const double commitment_s = std::min(
 prediction_frame_dt_s, trajectory.samples.back().t_s);
 const auto candidate_commit = sample_at(trajectory, commitment_s);
 const Vec2 reference_commit = timed_pose_at(reference, commitment_s);
 out.committed_position_error_m = std::hypot(
 candidate_commit.position.x - reference_commit.x,
 candidate_commit.position.y - reference_commit.y);
 out.committed_heading_error_rad = std::abs(wrap_angle(
 candidate_commit.heading_rad - timed_heading_at(
 reference, commitment_s, prediction_frame_dt_s)));
 const double reference_commit_speed_mps =
 timed_forward_speed_at(reference, commitment_s);
 out.committed_speed_error_mps = std::abs(
 candidate_commit.forward_mps - reference_commit_speed_mps);
 const double initial_tracking_error_m = std::hypot(
 trajectory.samples.front().position.x - reference.front().p.x,
 trajectory.samples.front().position.y - reference.front().p.y);
 const double allowed_tracking_error_m = std::max(
 constraints.committed_tracking_corridor_m, initial_tracking_error_m);
 const double preview_end_s = std::min({
 trajectory.samples.back().t_s, reference.back().t_s,
 std::max(commitment_s, 0.5)});
 double preview_cost = 0.0;
 std::size_t preview_samples = 0;
 for (double preview_s = commitment_s;
 preview_s <= preview_end_s + 1e-9;
 preview_s += prediction_frame_dt_s) {
 const auto candidate_preview = sample_at(trajectory, preview_s);
 const auto reference_preview = timed_pose_at(reference, preview_s);
 const double position_error_m = std::hypot(
 candidate_preview.position.x - reference_preview.x,
 candidate_preview.position.y - reference_preview.y);
 const double heading_error_rad = std::abs(wrap_angle(
 candidate_preview.heading_rad - timed_heading_at(
 reference, preview_s, prediction_frame_dt_s)));
 const double reference_speed_mps =
 timed_forward_speed_at(reference, preview_s);
 const double speed_error_mps = std::abs(
 candidate_preview.forward_mps - reference_speed_mps);
 preview_cost += position_error_m /
 std::max(1e-6, allowed_tracking_error_m) + heading_error_rad +
 speed_error_mps / std::max(0.25, reference_speed_mps);
 ++preview_samples;
 }
 out.committed_tracking_cost = preview_samples > 0
 ? preview_cost / static_cast<double>(preview_samples)
 : std::numeric_limits<double>::infinity();
if (out.committed_position_error_m >
 allowed_tracking_error_m + 1e-9) {
 valid_timing = false;
 committed_timing_diagnostic =
 "candidate timed-reference error " +
 std::to_string(out.committed_position_error_m) +
 " m exceeds " + std::to_string(allowed_tracking_error_m) + " m";
 }
 std::vector<TimedPose> continuation;
 if (valid_timing) {
 for (const auto& sample : trajectory.samples) {
 if (sample.t_s >= commitment_s - 1e-12)
 break;
 continuation.push_back({sample.t_s, sample.position});
 }
 continuation.push_back({commitment_s, candidate_commit.position});
 Vec2 previous = candidate_commit.position;
 for (const auto& pose : reference) {
 if (pose.t_s <= commitment_s + 1e-12)
 continue;
 if (!planning_map.segmentIsFree(previous, pose.p) ||
 (terrain_feasible && !terrain_feasible(previous, pose.p))) {
 valid_timing = false;
 committed_timing_diagnostic =
 "remaining timed crossing reference enters static or infeasible terrain";
 break;
 }
 continuation.push_back(pose);
 previous = pose.p;
 }
 }
 if (valid_timing && dynamic_objects_present) {
 committed_timed_gap_m = timed_gap(
 continuation, predicted_frames, prediction_frame_dt_s,
 robot_radius_m);
 committed_timed_continuation_safe =
 committed_timed_gap_m + 1e-9 >= required_margin_m;
 if (!committed_timed_continuation_safe)
 committed_timing_diagnostic =
 "remaining timed crossing execution dynamic gap " +
 std::to_string(committed_timed_gap_m) + " m is below " +
 std::to_string(required_margin_m) + " m";
 } else if (valid_timing) {
 committed_timed_gap_m = std::numeric_limits<double>::infinity();
 committed_timed_continuation_safe = true;
 }
 }

 if (dynamic_objects_present) {
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
 out.escape_clearance_gain_m = terminal_clearance - initial_clearance;
    out.verdict = TrajectoryVerdict::MinimumRiskEscape;
    out.diagnostic = "trajectory escapes an existing soft-margin incursion";
    return out;
   }
 if (!has_contingency_progress() && !committed_timed_continuation_safe) {
 out.verdict = TrajectoryVerdict::DynamicCollision;
 out.diagnostic = committed_timing_diagnostic.empty()
 ? "same-time swept clearance is below the required margin"
 : committed_timing_diagnostic;
 return out;
 }
 // A commitment does not waive this violation. Defer its verdict until the
 // independently simulated brake-and-hold trajectory is available below;
 // only a strictly lower-risk exit may continue after the forecast changes.
 nominal_dynamic_violation = true;
 nominal_terminal_clearance_m = terminal_clearance;
  }
 }


 if (committed_timed_continuation_safe) {
 out.min_dynamic_clearance_m = committed_timed_gap_m;
 out.preserves_committed_schedule = true;
 out.verdict = TrajectoryVerdict::CommittedTransit;
 out.diagnostic =
 "candidate preserves the current timed crossing execution";
 return out;
 }

 if (constraints.emergency_stop_required &&
 trajectory.command_space == LocalCommandSpace::DifferentialTwist &&
 !constraints.safe_stop_decel && constraints.emergency_decel_mps2 <= 0.0 &&
 (std::abs(trajectory.samples.front().forward_mps) > 1e-6 ||
 std::abs(trajectory.samples.front().yaw_rate_radps) > 1e-6 ||
 std::abs(trajectory.command.v) > 1e-6 ||
 std::abs(trajectory.command.w) > 1e-6)) {
 out.verdict = TrajectoryVerdict::UnsafeEmergencyStop;
 out.diagnostic = "current support and slope envelope has no safe braking authority";
 return out;
 }
 if ((constraints.safe_stop_decel || constraints.emergency_decel_mps2 > 0.0) &&
 trajectory.command_space == LocalCommandSpace::DifferentialTwist) {
 if (constraints.emergency_stop_delay_s > trajectory.samples.back().t_s + 1e-9) {
 if (accept_committed_transit())
 return out;
 out.verdict = TrajectoryVerdict::PredictionHorizonShort;
 out.diagnostic = "trajectory does not cover the emergency-stop response delay";
 return out;
 }
 // A stopped robot still occupies space. When the caller supplies a longer
 // maintained-intent forecast than the executable rollout, validate the
 // stopped pose through that complete forecast. Otherwise an actor arriving
 // just beyond the local rollout could make a command look passively safe,
 // even though braking leaves the robot in its path.
 const double dynamic_horizon_s = !dynamic_objects_present
 ? trajectory.samples.back().t_s
 : static_cast<double>(predicted_frames.size() - 1) * prediction_frame_dt_s;
 const auto stop_result = emergency_stop_samples(
 trajectory, constraints.emergency_stop_delay_s,
 constraints.emergency_decel_mps2, prediction_frame_dt_s,
 constraints.differential_track_m, dynamic_horizon_s,
 constraints.safe_stop_decel);
 if (!stop_result.complete) {
 if (accept_committed_transit())
 return out;
 out.verdict = TrajectoryVerdict::UnsafeEmergencyStop;
 out.diagnostic = "emergency braking tail encounters no safe braking authority";
 return out;
 }
 const auto& stop = stop_result.samples;
 for (std::size_t index = 1; index < stop.size(); ++index) {
 if (!planning_map.segmentIsFree(stop[index - 1].position, stop[index].position) ||
 (terrain_feasible && !terrain_feasible(
 stop[index - 1].position, stop[index].position))) {
 if (accept_committed_transit())
 return out;
 out.verdict = TrajectoryVerdict::UnsafeEmergencyStop;
 out.diagnostic = "emergency braking tail enters static or infeasible terrain";
 return out;
 }
 if (stability_active) {
 const double risk = stability_risk(&stop[index - 1], stop[index], constraints);
 out.max_dynamic_rollover_risk = std::max(out.max_dynamic_rollover_risk, risk);
 if (!std::isfinite(risk) || risk > constraints.max_dynamic_rollover_risk + 1e-12) {
 if (accept_committed_transit())
 return out;
 out.verdict = TrajectoryVerdict::UnsafeEmergencyStop;
 out.diagnostic = "emergency braking tail exceeds the dynamic support-polygon envelope";
 return out;
 }
 }
 }
 if (dynamic_objects_present) {
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
 if (nominal_dynamic_violation) {
 const double stopped_terminal_clearance_m = clearance_at(
 stop.back().position, stop.back().t_s, predicted_frames,
 prediction_frame_dt_s, robot_radius_m);
 const bool improves_minimum_gap =
 nominal_gap > stopping_gap + 1e-9;
 const bool preserves_minimum_and_improves_exit =
 nominal_gap + 1e-9 >= stopping_gap &&
 nominal_terminal_clearance_m > stopped_terminal_clearance_m + 1e-9;
 if (improves_minimum_gap || preserves_minimum_and_improves_exit) {
 out.min_dynamic_clearance_m = nominal_gap;
 out.escape_clearance_gain_m = std::max(
 nominal_gap - stopping_gap,
 nominal_terminal_clearance_m - stopped_terminal_clearance_m);
 if (accept_committed_transit())
 return out;
 }
 out.verdict = TrajectoryVerdict::DynamicCollision;
 out.diagnostic =
 "committed motion is not safer than the independently simulated brake-and-hold trajectory";
 return out;
 }
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

 if (nominal_dynamic_violation) {
 out.verdict = TrajectoryVerdict::DynamicCollision;
 out.diagnostic =
 "dynamic clearance is violated and no braking trajectory is available for minimum-risk comparison";
 return out;
 }

 out.verdict = TrajectoryVerdict::Accepted;
 // Keep the reason an otherwise safe local action did not preserve an
 // active behavior schedule. The caller must distinguish "safe for this
 // rollout" from "continues the admitted timed maneuver"; reducing both to
 // the word accepted hides exactly the feedback that can revoke a crossing
 // commitment on the next behavior cycle.
 out.diagnostic = committed_timing_active &&
 !committed_timed_continuation_safe &&
 !committed_timing_diagnostic.empty()
 ? "accepted locally; " + committed_timing_diagnostic
 : "accepted";
 return out;
}

ValidatedTrajectorySelection
select_executable_trajectory(const DwaResult& proposals, const GridMap& planning_map,
 const std::vector<std::vector<DynamicObjectState>>& predicted_frames,
 double prediction_frame_dt_s, double robot_radius_m, double required_margin_m,
 StaticFeasibility terrain_feasible, const TrajectoryConstraints& constraints) {
 ValidatedTrajectorySelection out;
 const bool committed_schedule_required =
 constraints.committed_reference_path.size() >= 2 &&
 constraints.committed_reference_trajectory.size() >= 2;
 int best_rank = 0;
 double best_escape_gain_m = -std::numeric_limits<double>::infinity();
 double best_schedule_tracking_cost =
 std::numeric_limits<double>::infinity();
 std::optional<ValidatedTrajectorySelection> best_schedule_miss;
 for (const auto& rollout : proposals.rollouts) {
 if (rollout.collision || !rollout.trajectory.command.valid)
 continue;
 const auto validation = validate_executable_trajectory(
 rollout.trajectory, planning_map, predicted_frames, prediction_frame_dt_s,
 robot_radius_m, required_margin_m, terrain_feasible, constraints);
 if (!validation.accepted())
 continue;
 if (committed_schedule_required &&
 !validation.preserves_committed_schedule &&
 !validation.committed_transit() &&
 !validation.minimum_risk_escape()) {
 if (!best_schedule_miss || dwa_command_preferred(
 rollout.trajectory.command,
 best_schedule_miss->trajectory.command)) {
 best_schedule_miss = ValidatedTrajectorySelection{
 rollout.trajectory, validation, 0};
 }
 continue;
 }
 ++out.accepted_candidates;
 // A crossing admission proves a complete time-indexed execution, while an
 // ordinary Accepted verdict proves only this local rollout. Prefer a
 // candidate that preserves the admitted schedule; otherwise a higher DWA
 // score can select a short safe hold, silently spend the traffic gap, and
 // invalidate the behavior proof. Minimum-risk contingencies remain below
 // fully nominal local motion when no admitted schedule is preserved.
 const int rank = validation.preserves_committed_schedule
 ? 4
 : (validation.committed_transit()
 ? 3
 : (validation.verdict == TrajectoryVerdict::Accepted ? 2 : 1));
 const bool better_escape = rank == 1 && best_rank == 1 &&
 (validation.escape_clearance_gain_m > best_escape_gain_m + 1e-9 ||
 (std::abs(validation.escape_clearance_gain_m - best_escape_gain_m) <= 1e-9 &&
 dwa_command_preferred(rollout.trajectory.command, out.trajectory.command)));
 const double schedule_tracking_cost = validation.committed_tracking_cost;
 constexpr double kTrackingCostEquivalence = 1e-9;
 const bool better_schedule = rank == 4 && best_rank == 4 &&
 (schedule_tracking_cost + kTrackingCostEquivalence <
 best_schedule_tracking_cost ||
 (std::abs(schedule_tracking_cost - best_schedule_tracking_cost) <=
 kTrackingCostEquivalence &&
 dwa_command_preferred(
 rollout.trajectory.command, out.trajectory.command)));
 const bool better_nominal = rank >= 2 && rank != 4 &&
 best_rank == rank &&
 dwa_command_preferred(rollout.trajectory.command, out.trajectory.command);
 if (rank < best_rank || (rank == best_rank && !better_escape &&
 !better_schedule && !better_nominal))
 continue;
 best_rank = rank;
 best_escape_gain_m = validation.escape_clearance_gain_m;
 best_schedule_tracking_cost = schedule_tracking_cost;
 out.trajectory = rollout.trajectory;
 out.validation = validation;
 }
 if (out.accepted_candidates == 0) {
 if (best_schedule_miss) {
 out = std::move(*best_schedule_miss);
 out.validation.verdict = TrajectoryVerdict::BehaviorConstraint;
 out.validation.preserves_committed_schedule = false;
 out.validation.diagnostic =
 "committed schedule has no executable candidate; " +
 out.validation.diagnostic;
 return out;
 }
 out.validation = validate_executable_trajectory(
 proposals.best, planning_map, predicted_frames, prediction_frame_dt_s,
 robot_radius_m, required_margin_m, terrain_feasible, constraints);
 }
 return out;
}

} // namespace sim
