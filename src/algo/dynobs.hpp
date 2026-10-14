#pragma once
#include "math.hpp"
#include <vector>
#include <cmath>
// 动障碍(巡逻叉车): a<->b乒乓, 纯时间函数, 确定性可复现.
// 半径含车体; 碰撞判定时再加机器人半径.
namespace sim {
// NOTE: vehicle geometry lives in robot JSON (collision_radius_m); do not
// reintroduce a hardcoded robot radius here.
struct Patrol {
 Vec2 a{0, 0}, b{0, 0};
 double speed{0.8};
 double radius{0.45};
};
struct DynState {
 Vec2 p{0, 0}, v{0, 0};
 double radius{0.45};
};
// Pedestrian wander: closed waypoint loop at constant speed, pure time
// function like patrol_state (deterministic, reproducible). Pedestrians are
// kinematic social agents: proximity eats clearance, contact never fails the
// mission (that call lives in MissionSimulation, not here).
struct Wander {
 std::vector<Vec2> points;
 double speed{0.5};
 double radius{0.25};
};
struct WanderState {
 Vec2 p{0, 0}, v{0, 0};
 double radius{0.25};
};
inline WanderState wander_state(const Wander& w, double t) {
 WanderState s;
 s.radius = w.radius;
 if (w.points.size() < 2 || w.speed < 1e-9) {
  if (!w.points.empty()) s.p = w.points.front();
  s.v = {0, 0};
  return s;
 }
 double total = 0.0;
 std::vector<double> seg;
 seg.reserve(w.points.size());
 for (std::size_t i = 0; i < w.points.size(); ++i) {
  const double len = std::hypot(w.points[(i + 1) % w.points.size()].x - w.points[i].x,
                                w.points[(i + 1) % w.points.size()].y - w.points[i].y);
  seg.push_back(len);
  total += len;
 }
 if (total < 1e-9) {
  s.p = w.points.front();
  s.v = {0, 0};
  return s;
 }
 double d = std::fmod(t * w.speed, total);
 for (std::size_t i = 0; i < w.points.size(); ++i) {
  if (d <= seg[i]) {
   const Vec2& a = w.points[i];
   const Vec2& b = w.points[(i + 1) % w.points.size()];
   const double f = seg[i] < 1e-9 ? 0.0 : d / seg[i];
   s.p = {a.x + (b.x - a.x) * f, a.y + (b.y - a.y) * f};
   const double n = std::hypot(b.x - a.x, b.y - a.y);
   s.v = n < 1e-9 ? Vec2{0, 0} : Vec2{(b.x - a.x) / n * w.speed, (b.y - a.y) / n * w.speed};
   return s;
  }
  d -= seg[i];
 }
 s.p = w.points.front();
 s.v = {0, 0};
 return s;
}
// t时刻状态(含解析速度, DWA做线性外推)
inline DynState patrol_state(const Patrol& pt, double t) {
 DynState s;
 s.radius = pt.radius;
 double L = std::hypot(pt.b.x - pt.a.x, pt.b.y - pt.a.y);
 if (L < 1e-9 || pt.speed < 1e-9) {
  s.p = pt.a;
  s.v = {0, 0};
  return s;
 }
 double dx = (pt.b.x - pt.a.x) / L, dy = (pt.b.y - pt.a.y) / L;
 double ph = std::fmod(t * pt.speed, 2 * L);
 if (ph < L) {
  s.p = {pt.a.x + dx * ph, pt.a.y + dy * ph};
  s.v = {dx * pt.speed, dy * pt.speed};
 } else {
  double q = ph - L;
  s.p = {pt.b.x - dx * q, pt.b.y - dy * q};
  s.v = {-dx * pt.speed, -dy * pt.speed};
 }
 return s;
}
} // namespace sim
