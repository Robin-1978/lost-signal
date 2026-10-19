#include "robot/autonomy/planning/local/holonomic_local.hpp"
#include "robot/autonomy/planning/local/command_window.hpp"
#include "robot/autonomy/planning/local/dynamic_clearance.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace sim {
namespace {

struct PathProjection {
 double distance_m{0.0};
 double along_m{0.0};
};

PathProjection project_to_path(Vec2 point, const std::vector<Vec2>& path) {
 PathProjection best{std::numeric_limits<double>::infinity(), 0.0};
 double accumulated_m = 0.0;
 for (std::size_t index = 0; index + 1 < path.size(); ++index) {
 const Vec2 delta{path[index + 1].x - path[index].x,
 path[index + 1].y - path[index].y};
 const double length_m = std::hypot(delta.x, delta.y);
 if (length_m < 1e-9)
 continue;
 const double projection = std::clamp(
 ((point.x - path[index].x) * delta.x +
 (point.y - path[index].y) * delta.y) /
 (length_m * length_m), 0.0, 1.0);
 const Vec2 closest{path[index].x + projection * delta.x,
 path[index].y + projection * delta.y};
 const double distance_m = std::hypot(point.x - closest.x, point.y - closest.y);
 if (distance_m < best.distance_m) {
 best.distance_m = distance_m;
 best.along_m = accumulated_m + projection * length_m;
 }
 accumulated_m += length_m;
 }
 if (path.size() == 1)
 best.distance_m = std::hypot(point.x - path.front().x, point.y - path.front().y);
 return best;
}

void append_unique(std::vector<double>& samples, double value) {
 if (std::none_of(samples.begin(), samples.end(), [value](double existing) {
 return std::abs(existing - value) < 1e-9;
 })) {
 samples.push_back(value);
 }
}

} // namespace

DwaResult holonomic_local_compute(Vec2 position, double heading_rad, double current_forward_mps,
                                  double current_left_mps, double current_yaw_radps, Vec2 goal,
                                  const GridMap& map, DwaWeights weights,
                                  const std::vector<std::vector<DynamicObjectState>>& predicted_frames,
 const HolonomicLocalConfig& config, StaticFeasibility terrain_feasible,
 const DwaReference& reference) {
 DwaResult output;
 output.best.command.score = -1e18;
 const double minimum_forward = std::min(config.min_forward_speed_mps, config.max_speed_mps);
 std::vector<double> velocity_samples{
 minimum_forward,
 std::max(minimum_forward, config.max_speed_mps * 0.5),
 config.max_speed_mps,
 };
 std::vector<double> lateral_samples{
 -config.max_speed_mps * 0.6, 0.0, config.max_speed_mps * 0.6};
 std::vector<double> yaw_samples{-config.max_yaw_radps, -config.max_yaw_radps * 0.5, 0.0,
                               config.max_yaw_radps * 0.5, config.max_yaw_radps};

 const double terminal_distance = config.terminal_position
 ? std::hypot(config.terminal_position->x - position.x,
 config.terminal_position->y - position.y)
 : std::numeric_limits<double>::infinity();
 const double planar_speed = std::hypot(current_forward_mps, current_left_mps);
 const double braking_distance = planar_speed * planar_speed /
 (2.0 * std::max(1e-6, config.max_accel_mps2));
 const double terminal_activation_m = std::max(
 1.0, config.terminal_position_tolerance_m + braking_distance +
 0.5 * config.max_speed_mps);
 const bool terminal_mode = config.terminal_position &&
 terminal_distance <= terminal_activation_m;
 if (terminal_mode) {
 // A swerve or mecanum chassis can correct a small overshoot directly. The
 // final position error is transformed into body coordinates so the sampler
 // includes signed fine-motion commands instead of choosing between a large
 // lateral step and another forward-only pass around the goal.
 const double dx = config.terminal_position->x - position.x;
 const double dy = config.terminal_position->y - position.y;
 const double desired_forward = std::clamp(
 1.2 * (std::cos(heading_rad) * dx + std::sin(heading_rad) * dy),
 -config.max_speed_mps, config.max_speed_mps);
 const double desired_left = std::clamp(
 1.2 * (-std::sin(heading_rad) * dx + std::cos(heading_rad) * dy),
 -config.max_speed_mps, config.max_speed_mps);
 velocity_samples.clear();
 lateral_samples.clear();
 append_unique(velocity_samples, 0.0);
 append_unique(velocity_samples, desired_forward * 0.5);
 append_unique(velocity_samples, desired_forward);
 append_unique(lateral_samples, 0.0);
 append_unique(lateral_samples, desired_left * 0.5);
 append_unique(lateral_samples, desired_left);

 yaw_samples.clear();
 append_unique(yaw_samples, 0.0);
 if (config.terminal_heading_rad) {
 const double heading_error = wrap_angle(*config.terminal_heading_rad - heading_rad);
 const double desired_yaw = std::clamp(
 1.5 * heading_error, -config.max_yaw_radps, config.max_yaw_radps);
 append_unique(yaw_samples, desired_yaw * 0.5);
 append_unique(yaw_samples, desired_yaw);
 }
 }
 for (double target_forward : velocity_samples) {
  for (double target_left : lateral_samples) {
   for (double target_yaw : yaw_samples) {
 // The sampled values describe policy targets. Publish only the planar
 // velocity vector and yaw rate reachable by the next control update; the
 // longer rollout horizon is evidence about that command, not extra actuator
 // response time available before publication.
 double forward_delta = target_forward - current_forward_mps;
 double left_delta = target_left - current_left_mps;
 const double planar_delta = std::hypot(forward_delta, left_delta);
 const double max_planar_delta = std::max(0.0, config.max_accel_mps2 * config.dt_s);
 if (planar_delta > max_planar_delta && planar_delta > 1e-12) {
 const double scale = max_planar_delta / planar_delta;
 forward_delta *= scale;
 left_delta *= scale;
 }
 target_forward = current_forward_mps + forward_delta;
 target_left = current_left_mps + left_delta;
 target_yaw = next_rate_limited_command(
 current_yaw_radps, target_yaw, -config.max_yaw_radps,
 config.max_yaw_radps, config.max_yaw_accel_radps2 * config.dt_s);
    double x = position.x, y = position.y, heading = heading_rad;
    double forward = current_forward_mps, left = current_left_mps, yaw = current_yaw_radps;
    bool collision = false;
    double free_time = config.predict_time_s;
    std::vector<TrajectorySample> samples{
      {0.0, {x, y}, heading, current_forward_mps, current_left_mps, current_yaw_radps, 0.0},
    };
    for (double time = 0.0; time < config.predict_time_s; time += config.dt_s) {
     const double step_dt = std::min(config.dt_s, config.predict_time_s - time);
     const Vec2 previous{x, y};
     forward +=
       clamp(target_forward - forward, -config.max_accel_mps2 * step_dt, config.max_accel_mps2 * step_dt);
     left += clamp(target_left - left, -config.max_accel_mps2 * step_dt, config.max_accel_mps2 * step_dt);
     yaw +=
       clamp(target_yaw - yaw, -config.max_yaw_accel_radps2 * step_dt, config.max_yaw_accel_radps2 * step_dt);
     const double mid = heading + yaw * step_dt * 0.5;
     x += (forward * std::cos(mid) - left * std::sin(mid)) * step_dt;
     y += (forward * std::sin(mid) + left * std::cos(mid)) * step_dt;
     heading = wrap_angle(heading + yaw * step_dt);
     const double sample_time = time + step_dt;
     samples.push_back({sample_time, {x, y}, heading, forward, left, yaw, 0.0});
 const bool dynamic_hit = dynamic_clearance_at(
 {x, y}, sample_time, predicted_frames, config.frame_dt_s,
 config.robot_radius_m).physical_m < 0.0;
     if (!map.isFree(x, y) || dynamic_hit || (terrain_feasible && !terrain_feasible(previous, {x, y}))) {
      collision = true;
      free_time = time;
      break;
     }
    }
 const Vec2 scoring_goal = terminal_mode ? *config.terminal_position : goal;
 const double terminal_position_error = std::hypot(scoring_goal.x - x, scoring_goal.y - y);
 const double goal_score = -terminal_position_error;
 const double goal_heading = terminal_mode
 ? config.terminal_heading_rad.value_or(heading_rad)
 : std::atan2(goal.y - y, goal.x - x);
 const double heading_score = std::cos(wrap_angle(goal_heading - heading));
    const double clearance_score = collision ? free_time : config.predict_time_s + 1.0;
 double path_score = 0.0;
 double progress_score = 0.0;
 if (reference.path.size() >= 2) {
 double distance_sum_m = 0.0;
 for (const auto& sample : samples)
 distance_sum_m += project_to_path(sample.position, reference.path).distance_m;
 const auto start_projection = project_to_path(position, reference.path);
 const auto end_projection = project_to_path({x, y}, reference.path);
 path_score = -distance_sum_m / static_cast<double>(samples.size());
 progress_score = end_projection.along_m - start_projection.along_m;
 }
 const bool angular_reversal =
 target_yaw * reference.previous_angular_command_radps < -0.04;
 double score = 0.0;
 if (terminal_mode) {
 const double terminal_speed = std::hypot(forward, left);
 const double terminal_heading_error = config.terminal_heading_rad
 ? std::abs(wrap_angle(*config.terminal_heading_rad - heading))
 : 0.0;
 const bool settled = terminal_position_error <= config.terminal_position_tolerance_m &&
 terminal_speed <= config.terminal_max_speed_mps &&
 std::abs(yaw) <= config.terminal_max_yaw_rate_radps &&
 terminal_heading_error <= config.terminal_heading_tolerance_rad;
 score = weights.w_goal * (4.0 * goal_score + 0.7 * heading_score) +
 weights.w_clear * clearance_score -
 2.0 * terminal_speed - 0.35 * std::abs(yaw) -
 4.0 * terminal_heading_error +
 (settled ? 3.0 : 0.0) + 0.2 * weights.w_path * path_score -
 weights.w_oscillation * (angular_reversal ? 1.0 : 0.0);
 } else {
 score = weights.w_goal * (goal_score + 0.7 * heading_score) +
 weights.w_clear * clearance_score +
 weights.w_speed * std::hypot(target_forward, target_left) -
 0.02 * std::abs(target_yaw) + weights.w_path * path_score +
 weights.w_progress * progress_score -
 weights.w_oscillation * (angular_reversal ? 1.0 : 0.0);
 }
    DwaCmd command{target_forward, target_yaw, !collision, score, target_left};
    ExecutableTrajectory trajectory{LocalCommandSpace::HolonomicTwist, command, std::move(samples)};
    output.rollouts.push_back({trajectory, collision});
    if (!collision && score > output.best.command.score) {
     output.best = std::move(trajectory);
    }
   }
  }
 }
 return output;
}

} // namespace sim
