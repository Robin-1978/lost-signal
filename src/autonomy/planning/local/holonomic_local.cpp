#include "autonomy/planning/local/holonomic_local.hpp"
#include "autonomy/planning/local/dynamic_clearance.hpp"

#include <algorithm>
#include <cmath>

namespace sim {

DwaResult holonomic_local_compute(Vec2 position, double heading_rad, double current_forward_mps,
                                  double current_left_mps, double current_yaw_radps, Vec2 goal,
                                  const GridMap& map, DwaWeights weights,
                                  const std::vector<std::vector<DynamicObjectState>>& predicted_frames,
                                  const HolonomicLocalConfig& config, StaticFeasibility terrain_feasible) {
 DwaResult output;
 output.best.command.score = -1e18;
 const double minimum_forward = std::min(config.min_forward_speed_mps, config.max_speed_mps);
 const double velocity_samples[] = {
 minimum_forward,
 std::max(minimum_forward, config.max_speed_mps * 0.5),
 config.max_speed_mps,
 };
 const double lateral_samples[] = {-config.max_speed_mps * 0.6, 0.0, config.max_speed_mps * 0.6};
 const double yaw_samples[] = {-config.max_yaw_radps, -config.max_yaw_radps * 0.5, 0.0,
                               config.max_yaw_radps * 0.5, config.max_yaw_radps};
 for (double target_forward : velocity_samples) {
  for (double target_left : lateral_samples) {
   for (double target_yaw : yaw_samples) {
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
 config.robot_radius_m).conservative_m < config.safety_margin_m;
     if (!map.isFree(x, y) || dynamic_hit || (terrain_feasible && !terrain_feasible(previous, {x, y}))) {
      collision = true;
      free_time = time;
      break;
     }
    }
    const double goal_score = -std::hypot(goal.x - x, goal.y - y);
    const double clearance_score = collision ? free_time : config.predict_time_s + 1.0;
    const double score = weights.w_goal * goal_score + weights.w_clear * clearance_score +
                         weights.w_speed * std::hypot(target_forward, target_left) -
                         0.02 * std::abs(target_yaw);
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
