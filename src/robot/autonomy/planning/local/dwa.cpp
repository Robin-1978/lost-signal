#include "robot/autonomy/planning/local/dwa.hpp"
#include "robot/autonomy/planning/local/command_window.hpp"
#include "robot/autonomy/planning/local/dynamic_clearance.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

namespace sim {
namespace {

using DynamicCollision = std::function<bool(double, double, double)>;

struct PathProjection {
 double distance_m{0.0};
 double along_m{0.0};
 double tangent_rad{0.0};
};

PathProjection project_to_path(Vec2 point, const std::vector<Vec2>& path) {
 PathProjection best;
 best.distance_m = std::numeric_limits<double>::infinity();
 if (path.empty())
 return best;
 if (path.size() == 1) {
 best.distance_m = std::hypot(point.x - path.front().x, point.y - path.front().y);
 return best;
 }
 double accumulated_m = 0.0;
 for (std::size_t index = 0; index + 1 < path.size(); ++index) {
 const Vec2 delta{
 path[index + 1].x - path[index].x,
 path[index + 1].y - path[index].y,
 };
 const double length_m = std::hypot(delta.x, delta.y);
 if (length_m < 1e-9)
 continue;
 const double projection = std::clamp(
 ((point.x - path[index].x) * delta.x +
 (point.y - path[index].y) * delta.y) /
 (length_m * length_m), 0.0, 1.0);
 const Vec2 closest{
 path[index].x + projection * delta.x,
 path[index].y + projection * delta.y,
 };
 const double distance_m = std::hypot(point.x - closest.x, point.y - closest.y);
 if (distance_m < best.distance_m) {
 best.distance_m = distance_m;
 best.along_m = accumulated_m + projection * length_m;
 best.tangent_rad = std::atan2(delta.y, delta.x);
 }
 accumulated_m += length_m;
 }
 return best;
}

Vec2 drive_axle_point(Vec2 base_link, double heading_rad, double axle_x_m) {
 return {
 base_link.x + axle_x_m * std::cos(heading_rad),
 base_link.y + axle_x_m * std::sin(heading_rad),
 };
}

std::optional<CommandWindow> differential_angular_window(
 CommandWindow body_window, double velocity_mps, double current_left_mps,
 double current_right_mps, double track_m, double max_wheel_speed_mps,
 double max_wheel_delta_mps) {
 if (!(track_m > 0.0))
 return body_window;
 const double hard_limit = max_wheel_speed_mps > 0.0
 ? max_wheel_speed_mps
 : std::numeric_limits<double>::infinity();
 const auto left_window = next_command_window(
 current_left_mps, -hard_limit, hard_limit, max_wheel_delta_mps);
 const auto right_window = next_command_window(
 current_right_mps, -hard_limit, hard_limit, max_wheel_delta_mps);
 const double half_track_m = track_m * 0.5;

 // left = v - w*b/2, right = v + w*b/2. Intersect the body-yaw
 // dynamic window with both wheel intervals before sampling. Sampling first
 // and rejecting afterwards can miss a reachable interval boundary forever
 // when a coarse yaw grid falls on either side of it.
 CommandWindow result{
 std::max({body_window.minimum,
 (velocity_mps - left_window.maximum) / half_track_m,
 (right_window.minimum - velocity_mps) / half_track_m}),
 std::min({body_window.maximum,
 (velocity_mps - left_window.minimum) / half_track_m,
 (right_window.maximum - velocity_mps) / half_track_m}),
 };
 if (result.minimum > result.maximum + 1e-9)
 return std::nullopt;
 if (result.minimum > result.maximum)
 result.minimum = result.maximum = 0.5 * (result.minimum + result.maximum);
 return result;
}

std::vector<Vec2> drive_axle_path(
 const std::vector<Vec2>& base_path, double axle_x_m) {
 if (base_path.size() < 2 || std::abs(axle_x_m) <= 1e-12)
 return base_path;
 std::vector<Vec2> result;
 result.reserve(base_path.size());
 for (std::size_t index = 0; index < base_path.size(); ++index) {
 Vec2 tangent;
 if (index + 1 < base_path.size()) {
 tangent = {base_path[index + 1].x - base_path[index].x,
 base_path[index + 1].y - base_path[index].y};
 } else {
 tangent = {base_path[index].x - base_path[index - 1].x,
 base_path[index].y - base_path[index - 1].y};
 }
 const double norm = std::hypot(tangent.x, tangent.y);
 if (norm <= 1e-12) {
 result.push_back(base_path[index]);
 continue;
 }
 result.push_back({
 base_path[index].x + axle_x_m * tangent.x / norm,
 base_path[index].y + axle_x_m * tangent.y / norm,
 });
 }
 return result;
}

DwaResult compute(Vec2 pos, double theta, double cur_v, double cur_w, Vec2 goal, const GridMap& map,
                  DwaWeights weights, const DynamicCollision& dynamic_collision, double v_max, double w_max,
                  double accel, double predict_time, double dt, double yaw_accel,
 const StaticFeasibility& terrain_feasible, double min_velocity_mps = -0.5,
 std::optional<double> target_heading_rad = std::nullopt,
 const DwaReference& reference = {}, double differential_track_m = 0.0,
 double max_wheel_speed_mps = 0.0,
 double actuator_response_delay_s = 0.0,
 double max_decel_mps2 = -1.0,
 double drive_axle_x_m = 0.0,
 std::optional<DifferentialWheelSpeeds> measured_wheel_speeds = std::nullopt) {
 DwaResult output;
 output.best.command.valid = false;
 output.best.command.score = -1e18;
 const double min_velocity = std::max(-0.5, min_velocity_mps);
 const double decel = max_decel_mps2 > 0.0 ? max_decel_mps2 : accel;
 if (v_max + 1e-9 < min_velocity || dt <= 0.0)
 return output;
 // DWA's dynamic window is the set of commands reachable by the next
 // control update, not by the end of the complete prediction horizon. The
 // rollout may be several seconds long, but publishing its terminal target
 // immediately would bypass the acceleration model used to validate it.
 // When the current speed is already outside a newly reduced policy cap,
 // retain the nearest rate-limited recovery command instead of demanding an
 // impossible instantaneous clamp.
 // Encoder-side motion is the authoritative starting point when the plant
 // publishes it. A filtered body estimate can lag a slide by a factor of ten,
 // which makes a window that is already past its reachable band look like a
 // gentle manoeuvre and produces commands the wheels cannot execute.
 const bool have_measured_wheels = measured_wheel_speeds.has_value();
 const double window_cur_v = have_measured_wheels
 ? 0.5 * (measured_wheel_speeds->left_mps + measured_wheel_speeds->right_mps)
 : cur_v;
 const auto velocity_window = next_velocity_command_window(
 window_cur_v, min_velocity, v_max, accel * dt, decel * dt);
 const auto velocity_samples = command_samples(
 velocity_window, 0.25, {window_cur_v, 0.0, min_velocity, v_max});
 // The longitudinal window above already arrests measured motion in the
 // wrong gear without authorizing an immediate forward lurch. Do not zero
 // yaw at the same time: a spin-capable chassis must still be able to rotate
 // out of a heading error while that arrest is in progress. Tying the two
 // made centimetre-per-second grade creep a permanent deadlock. The rollout
 // still starts from the measured twist, and the independent validator still
 // checks the swept footprint, so keeping yaw reachable does not waive safety.
 const double heading_target = target_heading_rad.value_or(
 std::atan2(goal.y - pos.y, goal.x - pos.x));
 const auto control_reference = drive_axle_path(
 reference.path, drive_axle_x_m);
 const Vec2 control_goal = drive_axle_point(
 goal, heading_target, drive_axle_x_m);
 const double effective_turn_time_s = std::max(
 dt, predict_time - std::max(0.0, actuator_response_delay_s));
 const double goal_directed_yaw_rate = std::clamp(
 wrap_angle(heading_target - theta) / effective_turn_time_s,
 -w_max, w_max);
 for (double velocity : velocity_samples) {
 const auto body_angular_window =
 next_command_window(cur_w, -w_max, w_max, yaw_accel * dt);
 const double current_left_mps = measured_wheel_speeds
 ? measured_wheel_speeds->left_mps
 : cur_v - cur_w * differential_track_m * 0.5;
 const double current_right_mps = measured_wheel_speeds
 ? measured_wheel_speeds->right_mps
 : cur_v + cur_w * differential_track_m * 0.5;
 const auto reachable_angular_window = differential_track_m > 0.0
 ? differential_angular_window(
 body_angular_window, velocity, current_left_mps, current_right_mps,
 differential_track_m, max_wheel_speed_mps, accel * dt)
 : std::optional<CommandWindow>{body_angular_window};
 if (!reachable_angular_window)
 continue;
 const auto angular_samples = command_samples(
 *reachable_angular_window, 0.3,
 {cur_w, 0.0, goal_directed_yaw_rate, -w_max, w_max});
 for (double angular : angular_samples) {
 if (differential_track_m > 0.0 && max_wheel_speed_mps > 0.0) {
 const double left_mps = velocity - angular * differential_track_m * 0.5;
 const double right_mps = velocity + angular * differential_track_m * 0.5;
 if (std::max(std::abs(left_mps), std::abs(right_mps)) >
 max_wheel_speed_mps + 1e-9) {
 continue;
 }
 }
 if (differential_track_m > 0.0 && accel > 0.0) {
 const double target_left_mps = velocity - angular * differential_track_m * 0.5;
 const double target_right_mps = velocity + angular * differential_track_m * 0.5;
 const double max_wheel_delta_mps = accel * dt;
 if (std::abs(target_left_mps - current_left_mps) > max_wheel_delta_mps + 1e-9 ||
 std::abs(target_right_mps - current_right_mps) > max_wheel_delta_mps + 1e-9) {
 continue;
 }
 }
   double x = pos.x;
   double y = pos.y;
   double heading = theta;
 std::vector<TrajectorySample> samples{{
 0.0, {x, y}, heading, cur_v, -drive_axle_x_m * cur_w, cur_w, 0.0}};
   double rollout_v = cur_v;
   double rollout_w = cur_w;
   double free_time = predict_time;
   bool collision = false;
   for (double time = 0.0; time < predict_time; time += dt) {
    const double step_dt = std::min(dt, predict_time - time);
    const Vec2 previous{x, y};
 // A command does not instantaneously change wheel speed. Preserve the
 // measured twist during the declared closed-loop response delay, then apply
 // physical acceleration limits. Without this interval, a nominal in-place
 // turn can look collision-free while the real chassis still translates.
 const double responsive_dt = std::max(
 0.0, time + step_dt - std::max(time, actuator_response_delay_s));
 rollout_v = velocity_step_toward(
 rollout_v, velocity, accel, decel, responsive_dt);
 rollout_w += clamp(angular - rollout_w,
 -yaw_accel * responsive_dt, yaw_accel * responsive_dt);
 const double rollout_lateral_v = -drive_axle_x_m * rollout_w;
 x += (rollout_v * std::cos(heading) -
 rollout_lateral_v * std::sin(heading)) * step_dt;
 y += (rollout_v * std::sin(heading) +
 rollout_lateral_v * std::cos(heading)) * step_dt;
    heading += rollout_w * step_dt;
    const double sample_time = time + step_dt;
 samples.push_back({sample_time, {x, y}, wrap_angle(heading),
 rollout_v, rollout_lateral_v, rollout_w, 0.0});
    if (!map.isFree(x, y) || dynamic_collision(x, y, sample_time) ||
        (terrain_feasible && !terrain_feasible(previous, {x, y}))) {
     collision = true;
     free_time = time;
     break;
    }
   }
 const Vec2 terminal_control = drive_axle_point(
 {x, y}, heading, drive_axle_x_m);
 const double goal_score = -std::hypot(
 control_goal.x - terminal_control.x,
 control_goal.y - terminal_control.y);
 const double goal_heading = target_heading_rad.value_or(
 std::atan2(goal.y - y, goal.x - x));
   const double heading_score = std::cos(wrap_angle(goal_heading - heading));
 // Lethal occupancy is a hard feasibility boundary, while distance inside
 // free space is a graded preference. Returning the same constant for every
 // collision-free rollout makes w_clear ineffective and lets the controller
 // drift to a cell edge until every useful turn becomes infeasible. Normalize
 // terminal clearance by one cell: this is the resolved gradient available
 // from a binary costmap, and capping it prevents open-space clearance from
 // overwhelming route progress or speed.
 const double resolved_static_clearance = map.clearance(x, y);
 const double normalized_static_clearance = std::clamp(
 resolved_static_clearance / std::max(1e-9, map.res()), 0.0, 1.0);
 // Clearance is a bounded preference among collision-free trajectories. It
 // must not outweigh real route progress and make a stationary rollout a
 // permanent optimum beside an inflated obstacle: the independent validator
 // already owns the hard swept-clearance contract. Express the complete
 // free-cell gradient as a small fraction of one collision-free horizon so
 // it breaks otherwise similar choices without turning policy comfort into a
 // second hard wall.
 constexpr double kStaticClearancePreferenceRangeS = 0.15;
 const double clearance_score = collision
 ? free_time
 : predict_time + 1.0 +
 kStaticClearancePreferenceRangeS * normalized_static_clearance;
 double path_score = 0.0;
 double progress_score = 0.0;
 if (!control_reference.empty()) {
 double distance_sum_m = 0.0;
 for (const auto& sample : samples)
 distance_sum_m += project_to_path(
 drive_axle_point(sample.position, sample.heading_rad, drive_axle_x_m),
 control_reference).distance_m;
 const auto start_projection = project_to_path(
 drive_axle_point(pos, theta, drive_axle_x_m), control_reference);
 const auto end_projection = project_to_path(
 terminal_control, control_reference);
 // An explicit terminal body heading is a pose constraint. Applying the
 // ordinary path-tangent critic at the same time can make a stopped robot
 // prefer its current yaw forever, even though position is already settled
 // and the behavior gate requires alignment. The terminal-heading critic
 // above owns orientation in that case; path distance still keeps the body
 // on the reference geometry.
 path_score = -distance_sum_m / static_cast<double>(samples.size());
 if (!target_heading_rad) {
 path_score += 0.35 * std::cos(
 wrap_angle(end_projection.tangent_rad - heading));
 }
 progress_score = end_projection.along_m - start_projection.along_m;
 }
 const bool angular_reversal =
 angular * reference.previous_angular_command_radps < -0.04;
 // When the route follower requests an in-place recovery, heading progress
 // is the only useful motion. A fixed steering-effort penalty can otherwise
 // beat the small heading improvement visible in a short rollout and make
 // the zero command a permanent local optimum. Moving rollouts retain the
 // damping term; oscillation has its own history-aware critic below.
 const double angular_effort_penalty = v_max <= 1e-9
 ? 0.0
 : 0.025 * std::abs(angular);
 // The velocity critic rewards progress in the requested gear. Using signed
 // body velocity directly rewards zero over every reverse candidate, which
 // can deadlock an otherwise feasible stop-line or cusp recovery even when
 // goal distance and path progress both improve.
 const bool reverse_only_window = v_max <= 1e-9 && min_velocity < -1e-9;
 const double preferred_speed = reverse_only_window ? -velocity : velocity;
   const double score = weights.w_goal * (goal_score + 0.7 * heading_score) +
 weights.w_clear * clearance_score + weights.w_speed * preferred_speed -
 angular_effort_penalty + weights.w_path * path_score +
 weights.w_progress * progress_score -
 weights.w_oscillation * (angular_reversal ? 1.0 : 0.0);
 const bool pose_heading_priority =
 v_max <= 1e-9 && min_velocity >= -1e-9 && target_heading_rad.has_value();
 const double terminal_heading_error_rad = std::abs(wrap_angle(
 heading_target - heading));
 DwaCmd command{
 velocity, angular, !collision, score, 0.0,
 pose_heading_priority, terminal_heading_error_rad};
   ExecutableTrajectory trajectory{LocalCommandSpace::DifferentialTwist, command, std::move(samples)};
   output.rollouts.push_back({trajectory, collision});
 if (!collision && dwa_command_preferred(command, output.best.command)) {
    output.best = std::move(trajectory);
   }
  }
 }
 // A raster corner can make every currently reachable translating command
 // collide even though several in-place rotations are safe. In that case the
 // local problem has changed from trajectory tracking to a rotation shim:
 // first minimize heading error toward the local path target, then use the
 // ordinary score and stable command order as tie-breakers. Without this
 // mode, a high-clearance zero command can beat every useful rotation and
 // create a permanent local minimum at the corner.
 const bool has_collision_free_translation = std::any_of(
 output.rollouts.begin(), output.rollouts.end(), [](const auto& rollout) {
 return !rollout.collision && rollout.trajectory.command.valid &&
 std::hypot(rollout.trajectory.command.v,
 rollout.trajectory.command.lateral_v) > 1e-6;
 });
 if (!has_collision_free_translation &&
 std::hypot(goal.x - pos.x, goal.y - pos.y) > 1e-6) {
 output.best = {};
 output.best.command.valid = false;
 output.best.command.score = -1e18;
 for (auto& rollout : output.rollouts) {
 auto& command = rollout.trajectory.command;
 if (rollout.collision || !command.valid)
 continue;
 const double current_heading_error_rad = wrap_angle(
 heading_target - theta);
 const double angular_stopping_angle_rad =
 yaw_accel > 1e-9 ? cur_w * cur_w / (2.0 * yaw_accel) : 0.0;
 const bool measured_turns_toward_target =
 current_heading_error_rad * cur_w > 1e-6;
 const bool reversal_before_braking_needed =
 measured_turns_toward_target &&
 std::abs(current_heading_error_rad) >
 angular_stopping_angle_rad + 0.02 &&
 current_heading_error_rad * command.w < -1e-6;
 command.pose_heading_priority = !reversal_before_braking_needed;
 // Rotation is a receding-horizon stage, not a command held for the complete
 // time needed to reach a distant translational target. Compare heading after
 // a short, actuator-aware preview; using the final sample of a multi-second
 // rollout rewards tiny yaw commands that are numerically exact at the end
 // but cannot overcome real contact/controller dynamics promptly.
 constexpr double kRotationShimPreviewS = 0.5;
 const auto preview = std::lower_bound(
 rollout.trajectory.samples.begin(), rollout.trajectory.samples.end(),
 kRotationShimPreviewS,
 [](const TrajectorySample& sample, double time_s) {
 return sample.t_s < time_s;
 });
 const double preview_heading_rad = rollout.trajectory.samples.empty()
 ? theta
 : (preview == rollout.trajectory.samples.end()
 ? rollout.trajectory.samples.back().heading_rad
 : preview->heading_rad);
 command.terminal_heading_error_rad = std::abs(wrap_angle(
 heading_target - preview_heading_rad));
 if (dwa_command_preferred(command, output.best.command))
 output.best = rollout.trajectory;
 }
 }
 return output;
}

} // namespace

bool dwa_command_preferred(const DwaCmd& candidate, const DwaCmd& incumbent) {
 if (!candidate.valid)
 return false;
 if (!incumbent.valid)
 return true;
 if (candidate.pose_heading_priority != incumbent.pose_heading_priority)
 return candidate.pose_heading_priority;
 if (candidate.pose_heading_priority && incumbent.pose_heading_priority) {
 constexpr double kHeadingEquivalenceRad = 1e-9;
 if (candidate.terminal_heading_error_rad + kHeadingEquivalenceRad <
 incumbent.terminal_heading_error_rad)
 return true;
 if (incumbent.terminal_heading_error_rad + kHeadingEquivalenceRad <
 candidate.terminal_heading_error_rad)
 return false;
 }
 // This is a numerical-equivalence test, not a planning tolerance. Sixty-
 // four ulps at the current score scale absorb ordinary accumulation-order
 // noise without hiding a physically meaningful score difference.
 const double scale = std::max({1.0, std::abs(candidate.score), std::abs(incumbent.score)});
 const double tolerance = 64.0 * std::numeric_limits<double>::epsilon() * scale;
 if (candidate.score > incumbent.score + tolerance)
 return true;
 if (incumbent.score > candidate.score + tolerance)
 return false;
 const auto less_with_tolerance = [tolerance](double lhs, double rhs) {
 return lhs < rhs - tolerance;
 };
 if (less_with_tolerance(std::abs(candidate.w), std::abs(incumbent.w)))
 return true;
 if (less_with_tolerance(std::abs(incumbent.w), std::abs(candidate.w)))
 return false;
 if (less_with_tolerance(std::hypot(candidate.v, candidate.lateral_v),
 std::hypot(incumbent.v, incumbent.lateral_v)))
 return true;
 if (less_with_tolerance(std::hypot(incumbent.v, incumbent.lateral_v),
 std::hypot(candidate.v, candidate.lateral_v)))
 return false;
 // Final signed keys give a total order when equal-magnitude commands have
 // opposite signs. They carry no safety claim; route direction and
 // feasibility have already been enforced before score arbitration.
 if (candidate.w != incumbent.w)
 return candidate.w < incumbent.w;
 if (candidate.v != incumbent.v)
 return candidate.v < incumbent.v;
 return candidate.lateral_v < incumbent.lateral_v;
}

DwaResult dwa_compute(Vec2 pos, double theta, double cur_v, double cur_w, Vec2 goal, const GridMap& map,
                      DwaWeights weights, double v_max, double w_max, double accel, double predict_time,
                      double dt, double robot_radius, double yaw_accel, StaticFeasibility terrain_feasible) {
 (void)robot_radius;
 return compute(
   pos, theta, cur_v, cur_w, goal, map, weights, [](double, double, double) { return false; }, v_max, w_max,
 accel, predict_time, dt, yaw_accel, terrain_feasible);
}

DwaResult dwa_compute_dyn(Vec2 pos, double theta, double cur_v, double cur_w, Vec2 goal, const GridMap& map,
                          DwaWeights weights, const std::vector<DynamicObjectState>& actors, double v_max,
                          double w_max, double accel, double predict_time, double dt, double safety_margin,
                          double robot_radius, double yaw_accel) {
 const auto dynamic_collision = [&](double x, double y, double time) {
  for (const auto& actor : actors) {
   const double px = actor.p.x + actor.v.x * time;
   const double py = actor.p.y + actor.v.y * time;
   if (std::hypot(x - px, y - py) < actor.radius + robot_radius + safety_margin) {
    return true;
   }
  }
  return false;
 };
 return compute(pos, theta, cur_v, cur_w, goal, map, weights, dynamic_collision, v_max, w_max, accel,
                predict_time, dt, yaw_accel, {});
}

DwaResult dwa_compute_trajectory(Vec2 pos, double theta, double cur_v, double cur_w, Vec2 goal,
                                 const GridMap& map, DwaWeights weights,
                                 const std::vector<std::vector<DynamicObjectState>>& predicted_frames,
                                 double v_max, double w_max, double accel, double predict_time, double dt,
                                 double frame_dt, double safety_margin, double robot_radius, double yaw_accel,
 StaticFeasibility terrain_feasible, double min_velocity_mps,
 std::optional<double> target_heading_rad, const DwaReference& reference,
 double differential_track_m, double max_wheel_speed_mps,
 double actuator_response_delay_s, double max_decel_mps2,
 double drive_axle_x_m,
 std::optional<DifferentialWheelSpeeds> measured_wheel_speeds) {
 (void)safety_margin;
 const auto dynamic_collision = [&](double x, double y, double time) {
 const auto clearance = dynamic_clearance_at(
 {x, y}, time, predicted_frames, frame_dt, robot_radius);
 // Candidate generation removes physical collisions only. The independent
 // trajectory validator owns uncertainty and the required soft margin so it
 // can distinguish an unsafe rollout from a named minimum-risk escape. A
 // sampler-level soft-margin veto used to discard every escape whose gap
 // briefly shrank while an actor approached.
 return clearance.physical_m < 0.0;
 };
 return compute(pos, theta, cur_v, cur_w, goal, map, weights, dynamic_collision, v_max, w_max, accel,
 predict_time, dt, yaw_accel, terrain_feasible, min_velocity_mps,
 target_heading_rad, reference, differential_track_m, max_wheel_speed_mps,
 actuator_response_delay_s, max_decel_mps2, drive_axle_x_m,
 measured_wheel_speeds);
}

} // namespace sim
