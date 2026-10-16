#include "autonomy/planning/local/ackermann_local.hpp"

#include <algorithm>
#include <cmath>

namespace sim {

DwaResult ackermann_local_compute(Vec2 position, double heading_rad, double current_speed_mps,
 double current_steer_rad, Vec2 goal, const GridMap& map, DwaWeights weights,
 const std::vector<std::vector<DynamicObjectState>>& predicted_frames,
 const AckermannLocalConfig& config, StaticFeasibility terrain_feasible) {
 DwaResult output;
 output.best.score = -1e18;
 const auto dynamic_collision = [&](double x, double y, double time) {
 if (predicted_frames.empty())
 return false;
 const std::size_t frame = std::min(predicted_frames.size() - 1,
 static_cast<std::size_t>(std::max(0.0, std::floor(time / config.frame_dt_s))));
 for (const auto& actor : predicted_frames[frame]) {
 if (std::hypot(x - actor.p.x, y - actor.p.y) <
 actor.radius + config.robot_radius_m + config.safety_margin_m) {
 return true;
 }
 }
 return false;
 };

 for (double target_speed = -0.5; target_speed <= config.max_speed_mps + 1e-9;
 target_speed += 0.25) {
 if (std::abs(target_speed - current_speed_mps) >
 config.max_accel_mps2 * config.predict_time_s) {
 continue;
 }
 constexpr int kSteerSamples = 9;
 for (int steer_sample = 0; steer_sample < kSteerSamples; ++steer_sample) {
 const double target_steer = -config.max_steer_rad +
 2.0 * config.max_steer_rad * steer_sample / (kSteerSamples - 1);
 double x = position.x;
 double y = position.y;
 double heading = heading_rad;
 double speed = current_speed_mps;
 double steer = current_steer_rad;
 bool collision = false;
 double free_time = config.predict_time_s;
 std::vector<Vec2> points{{x, y}};
 for (double time = 0.0; time < config.predict_time_s; time += config.dt_s) {
 const Vec2 previous{x, y};
 speed += clamp(target_speed - speed, -config.max_accel_mps2 * config.dt_s,
 config.max_accel_mps2 * config.dt_s);
 steer += clamp(target_steer - steer, -config.max_steer_rate_radps * config.dt_s,
 config.max_steer_rate_radps * config.dt_s);
 const double yaw_rate = speed * std::tan(steer) / std::max(0.05, config.wheel_base_m);
 const double heading_mid = heading + 0.5 * yaw_rate * config.dt_s;
 x += speed * std::cos(heading_mid) * config.dt_s;
 y += speed * std::sin(heading_mid) * config.dt_s;
 heading = wrap_angle(heading + yaw_rate * config.dt_s);
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
 const double clearance_score = collision ? free_time : config.predict_time_s + 1.0;
 const double score = weights.w_goal * (goal_score + 0.7 * heading_score) +
 weights.w_clear * clearance_score + weights.w_speed * target_speed -
 0.04 * std::abs(target_steer) - 0.02 * std::abs(target_steer - current_steer_rad);
 DwaCmd command{target_speed, target_steer, !collision, score};
 output.rollouts.push_back({command, points, collision});
 if (!collision && score > output.best.score) {
 output.best = command;
 output.best_path = std::move(points);
 }
 }
 }
 return output;
}

} // namespace sim
