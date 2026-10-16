#include "autonomy/planning/local/holonomic_local.hpp"

#include <algorithm>
#include <cmath>

namespace sim {

DwaResult holonomic_local_compute(Vec2 position, double heading_rad, double current_forward_mps,
 double current_left_mps, double current_yaw_radps, Vec2 goal, const GridMap& map,
 DwaWeights weights, const std::vector<std::vector<DynamicObjectState>>& predicted_frames,
 const HolonomicLocalConfig& config, StaticFeasibility terrain_feasible) {
 DwaResult output;
 output.best.score = -1e18;
 const double velocity_samples[] = {0.0, config.max_speed_mps * 0.5, config.max_speed_mps};
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
 std::vector<Vec2> points{{x, y}};
 for (double time = 0.0; time < config.predict_time_s; time += config.dt_s) {
 const Vec2 previous{x, y};
 forward += clamp(target_forward - forward, -config.max_accel_mps2 * config.dt_s,
 config.max_accel_mps2 * config.dt_s);
 left += clamp(target_left - left, -config.max_accel_mps2 * config.dt_s,
 config.max_accel_mps2 * config.dt_s);
 yaw += clamp(target_yaw - yaw, -config.max_yaw_accel_radps2 * config.dt_s,
 config.max_yaw_accel_radps2 * config.dt_s);
 const double mid = heading + yaw * config.dt_s * 0.5;
 x += (forward * std::cos(mid) - left * std::sin(mid)) * config.dt_s;
 y += (forward * std::sin(mid) + left * std::cos(mid)) * config.dt_s;
 heading = wrap_angle(heading + yaw * config.dt_s);
 points.push_back({x, y});
 bool dynamic_hit = false;
 if (!predicted_frames.empty()) {
 const auto frame = std::min(predicted_frames.size() - 1,
 static_cast<std::size_t>(std::max(0.0, std::floor(time / config.frame_dt_s))));
 for (const auto& actor : predicted_frames[frame]) {
 if (std::hypot(x - actor.p.x, y - actor.p.y) <
 actor.radius + config.robot_radius_m + config.safety_margin_m) {
 dynamic_hit = true;
 break;
 }
 }
 }
 if (!map.isFree(x, y) || dynamic_hit ||
 (terrain_feasible && !terrain_feasible(previous, {x, y}))) {
 collision = true;
 free_time = time;
 break;
 }
 }
 const double goal_score = -std::hypot(goal.x - x, goal.y - y);
 const double clearance_score = collision ? free_time : config.predict_time_s + 1.0;
 const double score = weights.w_goal * goal_score + weights.w_clear * clearance_score +
 weights.w_speed * std::hypot(target_forward, target_left) - 0.02 * std::abs(target_yaw);
 DwaCmd command{target_forward, target_yaw, !collision, score, target_left};
 output.rollouts.push_back({command, points, collision});
 if (!collision && score > output.best.score) {
 output.best = command;
 output.best_path = std::move(points);
 }
 }
 }
 }
 return output;
}

} // namespace sim
