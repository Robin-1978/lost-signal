#pragma once
#include "grid_map.hpp"
#include "dynobs.hpp"
#include <vector>
namespace sim {
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
};
struct DwaRollout {
 DwaCmd command;
 std::vector<Vec2> points;
 bool collision{false};
};
// Differential-drive DWA-lite: 在(v,w)窗口采样, 前向仿真predict_T, 打分.
// 权重是算法方法常量(随风格档整体缩放清除项), 玩家只选驾驶风格不调权重。
struct DwaResult {
 DwaCmd best;
 std::vector<Vec2> best_path;
 std::vector<DwaRollout> rollouts;
};
DwaResult dwa_compute(Vec2 pos, double theta, double cur_v, double cur_w, Vec2 goal, const GridMap& map,
                      DwaWeights wt, double v_max = 2.0, double w_max = 1.5, double accel = 2.0,
                      double predict_T = 1.5, double dt = 0.1);
// 时变版: dyn按线性外推(p+v*t)参与碰撞检查. 空dyn等价于静态版.
DwaResult dwa_compute_dyn(Vec2 pos, double theta, double cur_v, double cur_w, Vec2 goal, const GridMap& map,
                          DwaWeights wt, const std::vector<DynState>& dyn, double v_max = 2.0,
                          double w_max = 2.0, double accel = 3.0, double predict_T = 2.5, double dt = 0.1,
                          double safety_margin = 0.0);

// Time-indexed actor predictions. Each outer element is a future frame at
// frame_dt intervals and contains every actor state at that time. This lets a
// simulator represent patrol turns instead of extending current velocity forever.
DwaResult dwa_compute_trajectory(Vec2 pos, double theta, double cur_v, double cur_w, Vec2 goal,
                                 const GridMap& map, DwaWeights wt,
                                 const std::vector<std::vector<DynState>>& predicted_frames,
                                 double v_max = 2.0, double w_max = 2.0, double accel = 3.0,
                                 double predict_T = 2.5, double dt = 0.1, double frame_dt = 0.1,
                                 double safety_margin = 0.0);
} // namespace sim
