#include "sim/viewer/follow_camera.hpp"

#include <algorithm>
#include <cmath>

namespace lost_signal::sim::viewer {
namespace {

constexpr double kDirectionSwitchSpeedMps = 0.12;
constexpr double kMinimumSmoothingS = 0.05;

double positive_dt(double dt_s) {
  return std::isfinite(dt_s) ? std::clamp(dt_s, 0.0, 0.25) : 0.0;
}

double smoothing_blend(double dt_s, double smoothing_s) {
  const double dt = positive_dt(dt_s);
  if (dt <= 0.0) return 0.0;
  if (!std::isfinite(smoothing_s) || smoothing_s <= 0.0) return 1.0;
  const double tau = std::max(kMinimumSmoothingS, smoothing_s);
  return 1.0 - std::exp(-dt / tau);
}

double finite_or(double value, double fallback) {
  return std::isfinite(value) ? value : fallback;
}

}  // namespace

void FollowCameraFilter::reset() {
  initialized_ = false;
  filtered_forward_mps_ = 0.0;
  filtered_lateral_mps_ = 0.0;
  filtered_speed_mps_ = 0.0;
  lookahead_m_ = 0.0;
  travel_direction_ = 1.0;
}

void FollowCameraFilter::update(double& target_x_m, double& target_y_m,
                                double& distance_m,
                                const FollowCameraSample& sample,
                                const SceneCamera& config,
                                double base_distance_m, double wall_dt_s) {
  const double forward = finite_or(sample.forward_mps, 0.0);
  const double lateral = finite_or(sample.lateral_mps, 0.0);
  const double speed = std::hypot(forward, lateral);
  if (!initialized_) {
    filtered_forward_mps_ = forward;
    filtered_lateral_mps_ = lateral;
    filtered_speed_mps_ = speed;
    if (std::abs(forward) >= kDirectionSwitchSpeedMps) {
      travel_direction_ = forward < 0.0 ? -1.0 : 1.0;
    }
    lookahead_m_ = travel_direction_ * std::abs(filtered_forward_mps_) *
                   std::max(0.0, finite_or(config.lookahead_s, 0.0));
    initialized_ = true;
  } else {
    const double blend = smoothing_blend(wall_dt_s, config.follow_smoothing_s);
    filtered_forward_mps_ +=
        (forward - filtered_forward_mps_) * blend;
    filtered_lateral_mps_ +=
        (lateral - filtered_lateral_mps_) * blend;
    filtered_speed_mps_ +=
        (std::hypot(filtered_forward_mps_, filtered_lateral_mps_) -
         filtered_speed_mps_) * blend;
    // Keep the old view direction while the base is nearly stopped.  A
    // genuine reverse command must persist through the hysteresis threshold
    // before the lookahead changes sides.
    if (std::abs(filtered_forward_mps_) >= kDirectionSwitchSpeedMps) {
      travel_direction_ = filtered_forward_mps_ < 0.0 ? -1.0 : 1.0;
    }
  }

  const double lookahead_s =
      std::max(0.0, finite_or(config.lookahead_s, 0.0));
  const double desired_lookahead = travel_direction_ *
                                   std::abs(filtered_forward_mps_) *
                                   lookahead_s;
  const double blend = smoothing_blend(wall_dt_s, config.follow_smoothing_s);
  lookahead_m_ += (desired_lookahead - lookahead_m_) * blend;

  const double yaw = finite_or(sample.yaw_rad, 0.0);
  const double desired_x = finite_or(sample.x_m, target_x_m) +
                           lookahead_m_ * std::cos(yaw);
  const double desired_y = finite_or(sample.y_m, target_y_m) +
                           lookahead_m_ * std::sin(yaw);
  target_x_m += (desired_x - target_x_m) * blend;
  target_y_m += (desired_y - target_y_m) * blend;

  const double minimum_distance =
      finite_or(config.minimum_distance_m, base_distance_m);
  const double maximum_distance =
      std::max(minimum_distance,
               finite_or(config.maximum_distance_m, minimum_distance));
  const double distance_gain =
      std::max(0.0, finite_or(config.speed_distance_gain_s, 0.0));
  const double desired_distance = std::clamp(
      finite_or(base_distance_m, distance_m) +
          filtered_speed_mps_ * distance_gain,
      minimum_distance, maximum_distance);
  distance_m += (desired_distance - distance_m) * blend;
}

}  // namespace lost_signal::sim::viewer
