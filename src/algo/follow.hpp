#pragma once
#include "math.hpp"
#include "grid_map.hpp"
#include "pid.hpp"
#include "hw/chassis/caps.hpp"
#include <vector>
#include <cmath>
// 路径跟随共用逻辑: 前视 Ld 纯追踪 + 弯道速度调度 + 倒车迟滞,
// 由 guide_step 统一服务差速底盘(输出目标角速度, 不冒充电机 PWM).
namespace sim {
// 把idx推过已接近 / 已越过(跑到身后)的航点. 后者治"目标在身后死循环".
// 开头先做前向重同步: 若窗口内有明显更近的后续点(切弯/跟丢), 直接跳过去, 防止la乱指.
// 只允许前进(idx只增), 回退靠倒车恢复, 不靠idx回拨.
inline void advance_wp(const std::vector<Vec2>& path, size_t& idx, double ex, double ey, double eth,
                       double accept, double Ld) {
 (void)Ld;
 size_t hi = std::min(path.size(), idx + 24);
 size_t best = idx;
 double bd = std::hypot(path[idx].x - ex, path[idx].y - ey);
 for (size_t j = idx + 1; j < hi; j++) {
  double d = std::hypot(path[j].x - ex, path[j].y - ey);
  if (d < bd) {
   bd = d;
   best = j;
  }
 }
 if (best != idx && bd < 1.0)
  idx = best; // 重同步: 确实在更后面的track上才跳
 while (idx < path.size()) {
  double dx = path[idx].x - ex, dy = path[idx].y - ey;
  double dist = std::hypot(dx, dy);
  double along = (dx * std::cos(eth) + dy * std::sin(eth));
  if (dist < accept)
   idx++;
  else if (along < -0.1 && dist < 1.0 && idx + 1 < path.size())
   idx++; // 已越过(限1m内, 防拐角批量跳点)
  else
   break;
 }
}
// 线段是否全在free区(取costmap=膨胀地图,保净空)
inline bool los_clear(const GridMap& m, Vec2 a, Vec2 b) {
 double d = std::hypot(b.x - a.x, b.y - a.y);
 int n = std::max(1, (int)(d / (m.res() * 0.5)));
 for (int i = 0; i <= n; i++) {
  double t = (double)i / n;
  if (!m.isFree(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t))
   return false;
 }
 return true;
}
// 前视点三级策略:
// 1) track上第一个出Ld圆的点(纯追踪正解, 不跳段);
// 2) 若它无LOS(正入弯), 退回Ld内LOS最远点(贴角慢行);
// 3) 若Ld内全无LOS, 回退idx附近(含身后)最近有LOS点; 实在没有才用path[idx]兜底.
inline Vec2 lookahead_pt(const std::vector<Vec2>& path, size_t idx, double ex, double ey, double Ld,
                         const GridMap& cost) {
 Vec2 self{ex, ey};
 if (idx >= path.size())
  return path.back();
 size_t hi = std::min(path.size(), idx + 24);
 for (size_t i = idx; i < hi; i++) {
  if (std::hypot(path[i].x - ex, path[i].y - ey) >= Ld) {
   if (los_clear(cost, self, path[i]))
    return path[i];
   break; // 出圆但被挡: 转第2级, 不继续往远跳
  }
 }
 Vec2 best = path.back();
 bool found = false;
 // 注意: 离机器人欧氏距离沿路径不是单调的(过冲/折返处), 不能见d>Ld就break,
 // 否则会在Ld边界反复横跳. 只看窗口内满足d<=Ld且LOS的最后一点.
 for (size_t i = idx; i < hi; i++) {
  double d = std::hypot(path[i].x - ex, path[i].y - ey);
  if (d <= Ld && los_clear(cost, self, path[i])) {
   best = path[i];
   found = true;
  }
 }
 if (!found) {
  size_t j0 = idx > 4 ? idx - 4 : 0;
  for (size_t j = idx; j-- > j0;) { // 身后最近有LOS点
   if (los_clear(cost, self, path[j])) {
    best = path[j];
    found = true;
    break;
   }
   if (j == 0)
    break;
  }
  if (!found && idx < path.size())
   best = path[idx];
 }
 (void)found;
 return best;
}
// err大降速: cos调度,保底10%蠕行(自行车模型v=0无法转向)
inline double sched_v(double err, double cruise) {
 double c = std::cos(err);
 if (c < 0.1)
  c = 0.1;
 return cruise * c;
}
// 弯道预见减速（proper fix：看门狗/倒车恢复是 current fix，只管闯祸之后）。
// 预览窗口内每个路径转折按横向加速度物理限速：v ≤ √(a_lat·R)，
// R ≈ L_out/Δ（转出段长度除以转角）。直道（Δ=0）不限速；急弯提前减速，
// 进弯不再依赖看门狗救场。a_lat 由调用方按轮胎×地面给出（内容），
// 0.5 半抓地余量是方法常量（留一半抓地给扰动和模型误差）。
// 距离沿路径累积；转角取前后段方向差，与机器人当前位置无关——
// 只看“路有多弯”，不看“人偏多少”（后者是 sched_v 的事）。
inline double corner_speed_cap(const std::vector<Vec2>& path, size_t idx, double cruise, double alat_max,
                               double preview_m) {
 if (idx >= path.size() || cruise <= 0.0 || preview_m <= 0.0)
  return cruise;
 double cap = cruise;
 double along = 0.0;
 for (size_t i = idx + 1; i + 1 < path.size(); ++i) {
  along += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  if (along > preview_m)
   break;
  const double incoming = std::atan2(path[i].y - path[i - 1].y, path[i].x - path[i - 1].x);
  const double outgoing = std::atan2(path[i + 1].y - path[i].y, path[i + 1].x - path[i].x);
  const double turn = std::abs(wrap_angle(outgoing - incoming));
  if (turn < 1e-6)
   continue;
  const double leg = std::hypot(path[i + 1].x - path[i].x, path[i + 1].y - path[i].y);
  if (leg < 1e-6)
   continue;
  const double allowed = std::sqrt(std::max(0.0, alat_max) * leg / turn);
  if (allowed < cap)
   cap = allowed;
 }
 return std::max(cap, 0.1 * cruise);
}
inline double heading_err(Vec2 wp, double ex, double ey, double eth) {
 return wrap_angle(std::atan2(wp.y - ey, wp.x - ex) - eth);
}
// 跟随器状态(跨步保持): 航点下标 + 倒车锁存 + 倒车计时.
// 倒车必须带迟滞(1.4进/0.7出): 实测单阈值1.2会在err≈-1.2处正反抖振成极限环.
// 看门狗: 连续倒车超过 3 秒必定是锁存卡死(目标一直在身后), 切原地转向
// 面向目标。差速底盘本来就能原地打转, 没必要一直倒车。
struct GuideState {
 size_t idx{0};
 bool reversing{false};
 double reverse_time{0.0};
 bool spinning{false};
};
// ex/ey/eth: 用于推进+算err的位姿(真值或odom估计).
// steer单位由调用方PID的out_lim决定: 自行车=delta_max(rad), 差速=w_max(rad/s).
// tv: 前进速度(m/s), 倒车恢复时为负.
struct GuideOut {
 double err{0}, steer{0}, tv{0};
 bool reached{false};
};
inline GuideOut guide_step(const std::vector<Vec2>& path, GuideState& st, double ex, double ey, double eth,
                           PID& pid, double Ld, double cruise, double dt, const GridMap& cost,
                           const ChassisCaps& caps) {
 advance_wp(path, st.idx, ex, ey, eth, 0.6, Ld);
 if (st.idx >= path.size())
  return {0, 0, 0, true};
 Vec2 la = lookahead_pt(path, st.idx, ex, ey, Ld, cost);
 double err = heading_err(la, ex, ey, eth);
 double steer_fwd = pid.update(err, dt);
 if (!st.reversing && std::abs(err) > 1.4)
  st.reversing = true;
 else if (st.reversing && std::abs(err) < 0.7)
  st.reversing = false;
 double tv, steer;
 if (st.spinning) {
  // 看门狗后的原地转向：转到基本对准目标才退出，否则下一步又会倒车。
  // 只有能原地打转的底盘才进得来（见下）。
  if (std::abs(err) < 0.5) {
   st.spinning = false;
   st.reverse_time = 0.0;
  } else {
   return {err, steer_fwd, 0.0, false};
  }
 }
 if (st.reversing) {
  st.reverse_time += dt;
  if (st.reverse_time > 3.0) {
   // 看门狗：停倒车。能原地打转就转过去对准；不能的（如阿克曼）
   // 继续沿倒车恢复（带转向的倒车本就是它的恢复方式），全 K 转弯
   // 规划是后续工作，这里不假装会。
   st.reversing = false;
   st.reverse_time = 0.0;
   if (caps.can_spin) {
    st.spinning = true;
    return {err, steer_fwd, 0.0, false};
   }
  }
  tv = -0.4;
  steer = -steer_fwd;
 } // 目标在身后:倒车回转
 else {
  st.reverse_time = 0.0;
  steer = steer_fwd;
  tv = sched_v(err, cruise);
 }
 return {err, steer, tv, false};
}
} // namespace sim
