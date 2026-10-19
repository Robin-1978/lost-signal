#include "robot/autonomy/planning/local/ackermann_local.hpp"
#include "robot/autonomy/planning/local/command_window.hpp"
#include "robot/autonomy/planning/local/dynamic_clearance.hpp"
#include "robot/core/actuator_response.hpp"

#include <algorithm>
#include <cmath>

namespace sim {

DwaResult ackermann_local_compute(Vec2 position, double heading_rad, double current_speed_mps,
 double current_steer_rad, double current_steer_rate_radps,
 Vec2 goal, const GridMap& map, DwaWeights weights,
                                  const std::vector<std::vector<DynamicObjectState>>& predicted_frames,
                                  const AckermannLocalConfig& config, StaticFeasibility terrain_feasible) {
 DwaResult output;
 output.best.command.score = -1e18;
 const auto dynamic_collision = [&](double x, double y, double time) {
 return dynamic_clearance_at(
 {x, y}, time, predicted_frames, config.frame_dt_s,
 config.robot_radius_m).physical_m < 0.0;
 };

 const double reverse_limit = std::min(config.max_speed_mps, config.max_reverse_speed_mps);
 double minimum_speed = 0.0;
 double maximum_speed = config.max_speed_mps;
 std::vector<double> preferred_speeds{
 0.0, 0.5 * config.max_speed_mps, config.max_speed_mps};
 if (config.preferred_direction == TravelDirection::Reverse) {
 minimum_speed = -reverse_limit;
 maximum_speed = 0.0;
 preferred_speeds = {-reverse_limit, -0.5 * reverse_limit, 0.0};
 } else {
 // Gear is a global-route contract. A forward Hybrid A* section must not
 // become reverse merely because the old fixed 0.25 m/s sample stride starts
 // below zero. Include the exact cap so low-speed turns still have a moving
 // candidate when max_speed is smaller than that stride.
 minimum_speed = std::min(
 std::max(0.0, config.min_forward_speed_mps), config.max_speed_mps);
 preferred_speeds = {
 minimum_speed,
 std::max(minimum_speed, 0.5 * config.max_speed_mps),
 config.max_speed_mps};
 }
 const auto speed_window = next_command_window(
 current_speed_mps, minimum_speed, maximum_speed,
 config.max_accel_mps2 * config.dt_s);
 const auto speed_samples = command_samples(
 speed_window, 0.25, preferred_speeds);
 for (double target_speed : speed_samples) {
  constexpr int kSteerSamples = 9;
  for (int steer_sample = 0; steer_sample < kSteerSamples; ++steer_sample) {
   const double target_steer =
     -config.max_steer_rad + 2.0 * config.max_steer_rad * steer_sample / (kSteerSamples - 1);
   double x = position.x;
   double y = position.y;
   double heading = heading_rad;
   double speed = current_speed_mps;
   double steer = current_steer_rad;
 double steer_rate = current_steer_rate_radps;
   bool collision = false;
   double free_time = config.predict_time_s;
   std::vector<TrajectorySample> samples{
     {0.0,
      {x, y},
      heading,
      current_speed_mps,
      0.0,
      current_speed_mps * std::tan(current_steer_rad) / std::max(0.05, config.wheel_base_m),
      current_steer_rad},
   };
   for (double time = 0.0; time < config.predict_time_s; time += config.dt_s) {
    const double step_dt = std::min(config.dt_s, config.predict_time_s - time);
    const Vec2 previous{x, y};
    speed += clamp(target_speed - speed, -config.max_accel_mps2 * step_dt, config.max_accel_mps2 * step_dt);
 const double servo_pwm = position_servo_pwm(
 target_steer, steer, config.max_steer_rate_radps,
 config.steering_response_s);
 const double demanded_steer_rate = servo_pwm * config.max_steer_rate_radps;
 steer_rate = first_order_rate_step(
 steer_rate, demanded_steer_rate, config.steering_response_s, step_dt);
 steer = clamp(steer + steer_rate * step_dt,
 -config.max_steer_rad, config.max_steer_rad);
    const double yaw_rate = speed * std::tan(steer) / std::max(0.05, config.wheel_base_m);
    const double heading_mid = heading + 0.5 * yaw_rate * step_dt;
    x += speed * std::cos(heading_mid) * step_dt;
    y += speed * std::sin(heading_mid) * step_dt;
    heading = wrap_angle(heading + yaw_rate * step_dt);
    const double sample_time = time + step_dt;
    samples.push_back({sample_time, {x, y}, heading, speed, 0.0, yaw_rate, steer});
    if (!map.isFree(x, y) || dynamic_collision(x, y, sample_time) ||
        (terrain_feasible && !terrain_feasible(previous, {x, y}))) {
     collision = true;
     free_time = time;
     break;
    }
   }
   const double goal_score = -std::hypot(goal.x - x, goal.y - y);
   const double goal_heading = std::atan2(goal.y - y, goal.x - x);
   const bool moving_in_reverse =
     target_speed < -1e-9 || (std::abs(target_speed) <= 1e-9 && current_speed_mps < 0.0);
   const double motion_heading = wrap_angle(heading + (moving_in_reverse ? kPi : 0.0));
   const double heading_score = std::cos(wrap_angle(goal_heading - motion_heading));
   const double clearance_score = collision ? free_time : config.predict_time_s + 1.0;
   const double progress_speed =
     config.preferred_direction == TravelDirection::Reverse ? -target_speed : target_speed;
   const double score = weights.w_goal * (goal_score + 0.7 * heading_score) +
                        weights.w_clear * clearance_score + weights.w_speed * progress_speed -
                        0.04 * std::abs(target_steer) - 0.02 * std::abs(target_steer - current_steer_rad);
   DwaCmd command{target_speed, target_steer, !collision, score};
   ExecutableTrajectory trajectory{LocalCommandSpace::SteeringAngle, command, std::move(samples)};
   output.rollouts.push_back({trajectory, collision});
   if (!collision && score > output.best.command.score) {
    output.best = std::move(trajectory);
   }
  }
 }
 return output;
}

} // namespace sim
