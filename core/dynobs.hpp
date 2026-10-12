#pragma once
#include "math.hpp"
#include <vector>
#include <cmath>
// 动障碍(巡逻叉车): a<->b乒乓, 纯时间函数, 确定性可复现.
// 半径含车体; 碰撞判定时再加机器人半径.
namespace sim {
constexpr double kRobotRadius = 0.3;
struct Patrol { Vec2 a{0,0}, b{0,0}; double speed{0.8}; double radius{0.45}; };
struct DynState { Vec2 p{0,0}, v{0,0}; double radius{0.45}; };
// t时刻状态(含解析速度, DWA做线性外推)
inline DynState patrol_state(const Patrol& pt, double t){
  DynState s; s.radius=pt.radius;
  double L=std::hypot(pt.b.x-pt.a.x,pt.b.y-pt.a.y);
  if(L<1e-9 || pt.speed<1e-9){ s.p=pt.a; s.v={0,0}; return s; }
  double dx=(pt.b.x-pt.a.x)/L, dy=(pt.b.y-pt.a.y)/L;
  double ph=std::fmod(t*pt.speed,2*L);
  if(ph<L){ s.p={pt.a.x+dx*ph,pt.a.y+dy*ph}; s.v={dx*pt.speed,dy*pt.speed}; }
  else { double q=ph-L; s.p={pt.b.x-dx*q,pt.b.y-dy*q}; s.v={-dx*pt.speed,-dy*pt.speed}; }
  return s;
}
}
