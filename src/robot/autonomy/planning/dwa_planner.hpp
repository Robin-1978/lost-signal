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
// - map queries go to LocalMap; unknown and out-of-window cells are blocked
//   until a sensor ray or ROBOT-side SLAM replay marks them free;
// - dynamic checks consume predicted DynamicTrack frames;
// - output is one LocalCandidate, never a trajectory object;
// - wheel-space windows and actuator identification stay out: the IChassis
// wrapper owns feasibility against the real base.
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
  // Extra margin added to the chassis footprint for rollout collision.
  // The footprint itself comes from ChassisLimits at plan() time.
 double safety_margin_m = platform::kDefaultNavigationClearanceMarginM;
 // Outside the footprint-plus-margin lethal layer, cost falls linearly to
 // zero over this distance. This is a preference, not another hard wall.
 double soft_inflation_radius_m =
 platform::kDefaultNavigationSoftInflationRadiusM;
 double response_delay_s = 0.1;
 double velocity_sample_step_mps = 0.25;
 double yaw_rate_sample_step_radps = 0.3;
 double lateral_sample_step_mps = 0.25;
  double lookahead_m = 2.0;
 // Stateful route pruning may advance only this far per decision. It keeps
 // a self-overlapping path or an Ackermann reverse cusp from being skipped
 // by a geometrically closer point later in the route.
 double route_progress_search_m = 0.75;
 // Route progress cannot advance faster than measured chassis displacement
 // times this allowance for curved arc length. This prevents a stationary
 // robot from walking the cursor through a nearby return leg over time.
 double route_progress_distance_scale = 2.0;
 // Maximum cross-track distance at which stateful route progress may advance.
 // This belongs to the controller contract and can be tuned with the global
 // route resolution; crossing it requests a route from the current estimate.
 double route_progress_max_distance_m = 0.75;
 // If localization moves farther than this from the bounded route window,
 // reacquire globally once instead of following stale progress forever.
 double route_reacquire_distance_m = 2.0;
 // Preserve position authority until the final point is actually reached;
 // only then may terminal yaw replace the bearing-to-goal target.
 double terminal_position_tolerance_m = 0.25;
 double terminal_heading_tolerance_rad = 0.20;
 double terminal_maximum_position_sigma_m = 0.25;
 double terminal_maximum_heading_sigma_rad = 0.15;
 // A stopped terminal mismatch must persist for the same kind of
 // consecutive-evidence window used by arrival before requesting a new
 // global pose maneuver.
 int terminal_replan_settle_cycles = 3;
 // Holonomic lateral command slew limit. Lateral velocity is a command
 // state even when the estimator has no independent lateral sensor.
 double lateral_accel_mps2 = 2.0;
};

class DwaPlanner : public ILocalPlanner {
 public:
  explicit DwaPlanner(DwaPlannerConfig config = {});

  void reset() override;
  LocalPlan plan(const EstimatedState& est,
 const PerceptionResult& perception,
 const PredictionBatch& prediction,
 const LocalMap& map,
                 const Route& route,
                 const platform::ChassisLimits& limits) override;
 LocalPlan plan(const EstimatedState& est,
 const PerceptionResult& perception, const LocalMap& map,
 const Route& route, const platform::ChassisLimits& limits) {
 const double now_s = est.stamp_ns != core::kInvalidTime
 ? est.stamp_ns * 1e-9
 : perception.stamp_ns * 1e-9;
 return plan(est, perception,
 predict_perception(perception, now_s,
 {.horizon_s = config_.predict_time_s, .frame_dt_s = config_.dt_s}),
 map, route, limits);
 }
  // Terrain gate: consulted per rollout segment (empty = obstacle-only).
  void set_terrain_gate(TerrainGate gate) override { gate_ = std::move(gate); }

 private:
  DwaPlannerConfig config_;
  TerrainGate gate_;
  double previous_w_ = 0.0;
 double previous_lateral_ = 0.0;
 const RoutePoint* route_data_ = nullptr;
 std::size_t route_size_ = 0;
 std::size_t route_segment_ = 0;
 double route_fraction_ = 0.0;
 bool route_pose_valid_ = false;
 double route_pose_x_m_ = 0.0;
 double route_pose_y_m_ = 0.0;
 // A direction change is a physical stop-and-shift operation. Entering the
 // cusp capture region arms braking; a later decision may shift only after
 // measured longitudinal speed has settled. Keeping this state separate
 // from geometric route progress prevents an idle car from skipping onto a
 // nearby reverse leg on the first projection into the capture region.
 bool cusp_braking_armed_ = false;
 std::size_t cusp_segment_ = 0;
 int terminal_mismatch_settle_count_ = 0;
};

}  // namespace lost_signal::robot::autonomy
