#pragma once

#include "sim/viewer/scene_manifest.hpp"

namespace lost_signal::sim::viewer {

// The live frame is sampled at the fixed physics cadence.  Camera input is
// deliberately a small presentation contract so noisy stop/reverse samples
// cannot move the target independently of the rendered chassis.
struct FollowCameraSample {
  double x_m = 0.0;
  double y_m = 0.0;
  double yaw_rad = 0.0;
  double forward_mps = 0.0;
  double lateral_mps = 0.0;
};

class FollowCameraFilter {
 public:
  void reset();

  // Smooth the target and distance in-place.  The caller owns the orbit
  // angles and the manually selected base distance.  This filter is
  // presentation-only and never changes a recorded SIM frame.
  void update(double& target_x_m, double& target_y_m, double& distance_m,
              const FollowCameraSample& sample, const SceneCamera& config,
              double base_distance_m, double wall_dt_s);

 private:
  bool initialized_ = false;
  double filtered_forward_mps_ = 0.0;
  double filtered_lateral_mps_ = 0.0;
  double filtered_speed_mps_ = 0.0;
  double lookahead_m_ = 0.0;
  double travel_direction_ = 1.0;
};

}  // namespace lost_signal::sim::viewer
