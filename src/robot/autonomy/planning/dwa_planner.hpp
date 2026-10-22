#pragma once

#include <cstddef>
#include <vector>

#include "robot/autonomy/estimation/estimator.hpp"
#include "robot/autonomy/mapping/mapper.hpp"
#include "robot/autonomy/perception/perception.hpp"
#include "robot/autonomy/planning/global_planner.hpp"
#include "robot/autonomy/planning/local_planner.hpp"
#include "robot/autonomy/planning/prediction.hpp"
#include "robot/core/geometry.hpp"

namespace lost_signal::robot::autonomy {

// DWA local planner on native chain types. Same rollout + critic math as the
// main-stack DWA (dynamic window, response-delay rollout, goal / heading /
// clearance / path / progress / oscillation critics), but:
//   - map queries go to LocalMap (optimistic is_blocked; safety layers own
//     the conservative veto on raw scan);
//   - dynamic checks consume predicted DynamicTrack frames;
//   - output is one LocalCandidate, never a trajectory object;
//   - wheel-space windows and actuator identification stay out: the
//     IChassis wrapper owns feasibility against the real base.
struct DwaWeights {
  double w_goal = 1.0;
  double w_clear = 2.0;
  double w_speed = 0.3;
  double w_path = 1.2;
  double w_progress = 0.8;
  double w_oscillation = 0.25;
};

struct DwaPlannerConfig {
  DwaWeights weights;
  double accel_mps2 = 2.0;
  double decel_mps2 = 2.0;
  double yaw_accel_radps2 = 8.0;
  double predict_time_s = 1.5;
  double dt_s = 0.1;
  double robot_radius_m = 0.38;
  double response_delay_s = 0.0;
  double safety_margin_m = 0.0;
  double lookahead_m = 2.0;
};

class DwaPlanner : public ILocalPlanner {
 public:
  explicit DwaPlanner(DwaPlannerConfig config = {});

  void reset() override;
  LocalPlan plan(const EstimatedState& est,
                 const PerceptionResult& perception, const LocalMap& map,
                 const Route& route,
                 const platform::ChassisLimits& limits) override;

 private:
  DwaPlannerConfig config_;
  double previous_w_ = 0.0;
};

}  // namespace lost_signal::robot::autonomy
