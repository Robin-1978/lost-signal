#include "autonomy/planning/local/dwa.hpp"

#include <algorithm>
#include <cmath>
#include <functional>

namespace sim {
namespace {

using DynamicCollision = std::function<bool(double, double, double)>;

DwaResult compute(Vec2 pos, double theta, double cur_v, double cur_w, Vec2 goal, const GridMap& map,
                  DwaWeights weights, const DynamicCollision& dynamic_collision, double v_max, double w_max,
 double accel, double predict_time, double dt, double yaw_accel,
 const StaticFeasibility& terrain_feasible) {
 DwaResult output;
 output.best.valid = false;
 output.best.score = -1e18;
 for (double velocity = -0.5; velocity <= v_max + 1e-9; velocity += 0.25) {
  if (std::abs(velocity - cur_v) > accel * predict_time) {
   continue;
  }
  for (double angular = -w_max; angular <= w_max + 1e-9; angular += 0.3) {
   double x = pos.x;
   double y = pos.y;
   double heading = theta;
   std::vector<Vec2> points{{x, y}};
   double rollout_v = cur_v;
   double rollout_w = cur_w;
   double free_time = predict_time;
   bool collision = false;
   for (double time = 0.0; time < predict_time; time += dt) {
 const Vec2 previous{x, y};
    rollout_v += clamp(velocity - rollout_v, -accel * dt, accel * dt);
    rollout_w += clamp(angular - rollout_w, -yaw_accel * dt, yaw_accel * dt);
    x += rollout_v * std::cos(heading) * dt;
    y += rollout_v * std::sin(heading) * dt;
    heading += rollout_w * dt;
    points.push_back({x, y});
 if (!map.isFree(x, y) || dynamic_collision(x, y, time) ||
 (terrain_feasible && !terrain_feasible(previous, {x, y}))) {
     collision = true;
     free_time = time;
     break;
    }
   }
   const double goal_score = -std::hypot(goal.x - x, goal.y - y);
   const double goal_heading = std::atan2(goal.y - y, goal.x - x);
   const double heading_score = std::cos(wrap_angle(goal_heading - heading));
   const double clearance_score = collision ? free_time : predict_time + 1.0;
   const double score = weights.w_goal * (goal_score + 0.7 * heading_score) +
                        weights.w_clear * clearance_score + weights.w_speed * velocity -
                        0.025 * std::abs(angular);
   DwaCmd command{velocity, angular, !collision, score};
   output.rollouts.push_back({command, points, collision});
   if (!collision && score > output.best.score) {
    output.best = command;
    output.best_path = std::move(points);
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
                          DwaWeights weights, const std::vector<DynamicObjectState>& actors, double v_max, double w_max,
                          double accel, double predict_time, double dt, double safety_margin,
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
                                 const std::vector<std::vector<DynamicObjectState>>& predicted_frames, double v_max,
                                 double w_max, double accel, double predict_time, double dt, double frame_dt,
 double safety_margin, double robot_radius, double yaw_accel,
 StaticFeasibility terrain_feasible) {
 const auto dynamic_collision = [&](double x, double y, double time) {
  if (predicted_frames.empty()) {
   return false;
  }
  const auto frame_index = std::min(predicted_frames.size() - 1,
                                    static_cast<std::size_t>(std::max(0.0, std::floor(time / frame_dt))));
  for (const auto& actor : predicted_frames[frame_index]) {
   if (std::hypot(x - actor.p.x, y - actor.p.y) < actor.radius + robot_radius + safety_margin) {
    return true;
   }
  }
  return false;
 };
 return compute(pos, theta, cur_v, cur_w, goal, map, weights, dynamic_collision, v_max, w_max, accel,
 predict_time, dt, yaw_accel, terrain_feasible);
}

} // namespace sim
