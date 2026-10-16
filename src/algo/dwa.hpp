#pragma once
#include "grid_map.hpp"
#include "dynobs.hpp"
#include <functional>
#include <vector>
namespace sim {
using StaticFeasibility = std::function<bool(Vec2, Vec2)>;
struct DwaWeights {
 double w_goal{1.0};
 double w_clear{2.0};
 double w_speed{0.3};
};
struct DwaCmd {
 double v{0};
 double w{0};
 bool valid{false};
 double score{0};
 double lateral_v{0};
};
struct DwaRollout {
 DwaCmd command;
 std::vector<Vec2> points;
 bool collision{false};
};
// Differential-drive DWA-lite: sample the (v,w) window, simulate forward for predict_T, and score each candidate.
// Weights are method constants; the player selects a driving style rather than tuning individual weights.
struct DwaResult {
 DwaCmd best;
 std::vector<Vec2> best_path;
 std::vector<DwaRollout> rollouts;
};
// robot_radius and yaw_accel come from robot JSON through the mission layer;
// DWA never invents vehicle geometry.
DwaResult dwa_compute(Vec2 pos, double theta, double cur_v, double cur_w, Vec2 goal, const GridMap& map,
                      DwaWeights wt, double v_max = 2.0, double w_max = 1.5, double accel = 2.0,
                      double predict_T = 1.5, double dt = 0.1, double robot_radius = 0.38,
 double yaw_accel = 8.0, StaticFeasibility terrain_feasible = {});
// Time-varying form: linearly extrapolate dynamic actors as p + v*t for collision checks. An empty list equals the static form.
DwaResult dwa_compute_dyn(Vec2 pos, double theta, double cur_v, double cur_w, Vec2 goal, const GridMap& map,
                          DwaWeights wt, const std::vector<DynState>& dyn, double v_max = 2.0,
                          double w_max = 2.0, double accel = 3.0, double predict_T = 2.5, double dt = 0.1,
                          double safety_margin = 0.0, double robot_radius = 0.38, double yaw_accel = 8.0);

// Time-indexed actor predictions. Each outer element is a future frame at
// frame_dt intervals and contains every actor state at that time. This lets a
// simulator represent patrol turns instead of extending current velocity forever.
DwaResult dwa_compute_trajectory(Vec2 pos, double theta, double cur_v, double cur_w, Vec2 goal,
                                 const GridMap& map, DwaWeights wt,
                                 const std::vector<std::vector<DynState>>& predicted_frames,
                                 double v_max = 2.0, double w_max = 2.0, double accel = 3.0,
                                 double predict_T = 2.5, double dt = 0.1, double frame_dt = 0.1,
                                 double safety_margin = 0.0, double robot_radius = 0.38,
 double yaw_accel = 8.0, StaticFeasibility terrain_feasible = {});
} // namespace sim
