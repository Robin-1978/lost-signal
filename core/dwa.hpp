#pragma once
#include "grid_map.hpp"
#include "dynobs.hpp"
#include <vector>
namespace sim {
struct DwaWeights { double w_goal{1.0}; double w_clear{2.0}; double w_speed{0.3}; };
struct DwaCmd { double v{0}; double w{0}; bool valid{false}; double score{0}; };
// Differential-drive DWA-lite: 在(v,w)窗口采样, 前向仿真predict_T, 打分.
// 玩家调 w_goal/w_clear/w_speed 就能看到"冲目标"vs"保安全"vs"求快"的打架。
struct DwaResult { DwaCmd best; std::vector<std::pair<Vec2,double>> rollouts; /* endpoint+score */ };
DwaResult dwa_compute(Vec2 pos, double theta, double cur_v, double cur_w,
  Vec2 goal, const GridMap& map, DwaWeights wt,
  double v_max=2.0, double w_max=1.5, double accel=2.0, double predict_T=1.5, double dt=0.1);
// 时变版: dyn按线性外推(p+v*t)参与碰撞检查. 空dyn等价于静态版.
DwaResult dwa_compute_dyn(Vec2 pos, double theta, double cur_v, double cur_w,
  Vec2 goal, const GridMap& map, DwaWeights wt, const std::vector<DynState>& dyn,
  double v_max=2.0, double w_max=2.0, double accel=3.0, double predict_T=2.5, double dt=0.1);
}
