#include "robot/autonomy/planning/local/dwa.hpp"
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

DwaResult compute(Vec2 pos, double theta, double cur_v, double cur_w, Vec2 goal, const GridMap& map,
                  DwaWeights weights, const DynamicCollision& dynamic_collision, double v_max, double w_max,
                  double accel, double predict_time, double dt, double yaw_accel,
 const StaticFeasibility& terrain_feasible, double min_velocity_mps = -0.5,
 std::optional<double> target_heading_rad = std::nullopt,
 const DwaReference& reference = {}, double differential_track_m = 0.0,
 double max_wheel_speed_mps = 0.0) {
 DwaResult output;
 output.best.command.valid = false;
 output.best.command.score = -1e18;
 const double min_velocity = std::max(-0.5, min_velocity_mps);
 std::vector<double> velocity_samples;
 for (double velocity = min_velocity; velocity <= v_max + 1e-9; velocity += 0.25)
  velocity_samples.push_back(velocity);
 if (v_max >= min_velocity &&
     (velocity_samples.empty() || std::abs(velocity_samples.back() - v_max) > 1e-9)) {
  velocity_samples.push_back(v_max);
 }
 for (double velocity : velocity_samples) {
  if (std::abs(velocity - cur_v) > accel * predict_time) {
   continue;
  }
 std::vector<double> angular_samples;
 const auto append_angular = [&](double angular) {
 angular = std::clamp(angular, -w_max, w_max);
 if (std::none_of(angular_samples.begin(), angular_samples.end(),
 [&](double existing) { return std::abs(existing - angular) <= 1e-9; })) {
 angular_samples.push_back(angular);
 }
 };
 for (double angular = -w_max; angular <= w_max + 1e-9; angular += 0.3)
 append_angular(angular);
 append_angular(-w_max);
 append_angular(0.0);
 append_angular(cur_w);
 append_angular(w_max);
 for (double angular : angular_samples) {
 if (differential_track_m > 0.0 && max_wheel_speed_mps > 0.0) {
 const double left_mps = velocity - angular * differential_track_m * 0.5;
 const double right_mps = velocity + angular * differential_track_m * 0.5;
 if (std::max(std::abs(left_mps), std::abs(right_mps)) >
 max_wheel_speed_mps + 1e-9) {
 continue;
 }
 }
   double x = pos.x;
   double y = pos.y;
   double heading = theta;
   std::vector<TrajectorySample> samples{{0.0, {x, y}, heading, cur_v, 0.0, cur_w, 0.0}};
   double rollout_v = cur_v;
   double rollout_w = cur_w;
   double free_time = predict_time;
   bool collision = false;
   for (double time = 0.0; time < predict_time; time += dt) {
    const double step_dt = std::min(dt, predict_time - time);
    const Vec2 previous{x, y};
    rollout_v += clamp(velocity - rollout_v, -accel * step_dt, accel * step_dt);
    rollout_w += clamp(angular - rollout_w, -yaw_accel * step_dt, yaw_accel * step_dt);
    x += rollout_v * std::cos(heading) * step_dt;
    y += rollout_v * std::sin(heading) * step_dt;
    heading += rollout_w * step_dt;
    const double sample_time = time + step_dt;
    samples.push_back({sample_time, {x, y}, wrap_angle(heading), rollout_v, 0.0, rollout_w, 0.0});
    if (!map.isFree(x, y) || dynamic_collision(x, y, sample_time) ||
        (terrain_feasible && !terrain_feasible(previous, {x, y}))) {
     collision = true;
     free_time = time;
     break;
    }
   }
   const double goal_score = -std::hypot(goal.x - x, goal.y - y);
 const double goal_heading = target_heading_rad.value_or(
 std::atan2(goal.y - y, goal.x - x));
   const double heading_score = std::cos(wrap_angle(goal_heading - heading));
   const double clearance_score = collision ? free_time : predict_time + 1.0;
 double path_score = 0.0;
 double progress_score = 0.0;
 if (!reference.path.empty()) {
 double distance_sum_m = 0.0;
 for (const auto& sample : samples)
 distance_sum_m += project_to_path(sample.position, reference.path).distance_m;
 const auto start_projection = project_to_path(pos, reference.path);
 const auto end_projection = project_to_path({x, y}, reference.path);
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
   const double score = weights.w_goal * (goal_score + 0.7 * heading_score) +
                        weights.w_clear * clearance_score + weights.w_speed * velocity -
 0.025 * std::abs(angular) + weights.w_path * path_score +
 weights.w_progress * progress_score -
 weights.w_oscillation * (angular_reversal ? 1.0 : 0.0);
   DwaCmd command{velocity, angular, !collision, score};
   ExecutableTrajectory trajectory{LocalCommandSpace::DifferentialTwist, command, std::move(samples)};
   output.rollouts.push_back({trajectory, collision});
   if (!collision && score > output.best.command.score) {
    output.best = std::move(trajectory);
   }
  }
 }
 return output;
}

} // namespace

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
 double differential_track_m, double max_wheel_speed_mps) {
 const auto dynamic_collision = [&](double x, double y, double time) {
 const auto clearance = dynamic_clearance_at(
 {x, y}, time, predicted_frames, frame_dt, robot_radius);
 if (clearance.physical_m < 0.0)
 return true;
 if (clearance.conservative_m >= safety_margin)
 return false;
 // If prediction changes while the robot is already inside a soft envelope,
 // allow only a non-worsening escape. The independent validator below still
 // requires measurable terminal improvement and never permits overlap.
 const auto initial = dynamic_clearance_at(
 pos, 0.0, predicted_frames, frame_dt, robot_radius);
 return initial.conservative_m >= safety_margin ||
 clearance.conservative_m + 1e-9 < initial.conservative_m;
 };
 return compute(pos, theta, cur_v, cur_w, goal, map, weights, dynamic_collision, v_max, w_max, accel,
 predict_time, dt, yaw_accel, terrain_feasible, min_velocity_mps,
 target_heading_rad, reference, differential_track_m, max_wheel_speed_mps);
}

} // namespace sim
