#pragma once
#include "math.hpp"
#include "grid_map.hpp"
#include "bicycle.hpp"
#include "pid.hpp"
#include <vector>
#include <cmath>
// 路径跟随共用逻辑(Core,可测).
// 关键修正: 自行车最小转弯半径 R=L/tan(delta_max)~2.9m, 不能追最近航点(0.5m间距),
// 必须用前视Ld纯追踪 + 弯道速度调度, 否则恒速PID必然绕圈/穿墙. 这本身就是教学点.
namespace sim {
struct FollowOut { double err{0}; double steer{0}; double v{0}; bool reached{false}; bool blocked{false}; };
// 把idx推过已接近 / 已越过(跑到身后)的航点. 后者治"目标在身后死循环".
// 开头先做前向重同步: 若窗口内有明显更近的后续点(切弯/跟丢), 直接跳过去, 防止la乱指.
// 只允许前进(idx只增), 回退靠倒车恢复, 不靠idx回拨.
inline void advance_wp(const std::vector<Vec2>& path, size_t& idx, double ex, double ey, double eth, double accept, double Ld){
  (void)Ld;
  size_t hi=std::min(path.size(),idx+24);
  size_t best=idx;
  double bd=std::hypot(path[idx].x-ex,path[idx].y-ey);
  for(size_t j=idx+1;j<hi;j++){
    double d=std::hypot(path[j].x-ex,path[j].y-ey);
    if(d<bd){ bd=d; best=j; }
  }
  if(best!=idx && bd<1.0) idx=best; // 重同步: 确实在更后面的track上才跳
  while(idx<path.size()){
    double dx=path[idx].x-ex, dy=path[idx].y-ey;
    double dist=std::hypot(dx,dy);
    double along=(dx*std::cos(eth)+dy*std::sin(eth));
    if(dist<accept) idx++;
    else if(along<-0.1 && dist<1.0 && idx+1<path.size()) idx++; // 已越过(限1m内, 防拐角批量跳点)
    else break;
  }
}
// 线段是否全在free区(取costmap=膨胀地图,保净空)
inline bool los_clear(const GridMap& m, Vec2 a, Vec2 b){
  double d=std::hypot(b.x-a.x,b.y-a.y);
  int n=std::max(1,(int)(d/(m.res()*0.5)));
  for(int i=0;i<=n;i++){
    double t=(double)i/n;
    if(!m.isFree(a.x+(b.x-a.x)*t, a.y+(b.y-a.y)*t)) return false;
  }
  return true;
}
// 前视点三级策略:
// 1) track上第一个出Ld圆的点(纯追踪正解, 不跳段);
// 2) 若它无LOS(正入弯), 退回Ld内LOS最远点(贴角慢行);
// 3) 若Ld内全无LOS, 回退idx附近(含身后)最近有LOS点; 实在没有才用path[idx]兜底.
inline Vec2 lookahead_pt(const std::vector<Vec2>& path, size_t idx, double ex, double ey, double Ld, const GridMap& cost){
  Vec2 self{ex,ey};
  if(idx>=path.size()) return path.back();
  size_t hi=std::min(path.size(),idx+24);
  for(size_t i=idx;i<hi;i++){
    if(std::hypot(path[i].x-ex,path[i].y-ey)>=Ld){
      if(los_clear(cost,self,path[i])) return path[i];
      break; // 出圆但被挡: 转第2级, 不继续往远跳
    }
  }
  Vec2 best=path.back(); bool found=false;
  // 注意: 离机器人欧氏距离沿路径不是单调的(过冲/折返处), 不能见d>Ld就break,
  // 否则会在Ld边界反复横跳. 只看窗口内满足d<=Ld且LOS的最后一点.
  for(size_t i=idx;i<hi;i++){
    double d=std::hypot(path[i].x-ex,path[i].y-ey);
    if(d<=Ld && los_clear(cost,self,path[i])){ best=path[i]; found=true; }
  }
  if(!found){
    size_t j0=idx>4?idx-4:0;
    for(size_t j=idx;j-->j0;){ // 身后最近有LOS点
      if(los_clear(cost,self,path[j])){ best=path[j]; found=true; break; }
      if(j==0) break;
    }
    if(!found && idx<path.size()) best=path[idx];
  }
  (void)found;
  return best;
}
// err大降速: cos调度,保底10%蠕行(自行车模型v=0无法转向)
inline double sched_v(double err, double cruise){
  double c=std::cos(err);
  if(c<0.1) c=0.1;
  return cruise*c;
}
inline double heading_err(Vec2 wp, double ex, double ey, double eth){
  return wrap_angle(std::atan2(wp.y-ey,wp.x-ex)-eth);
}
// 跟随器状态(跨步保持): 航点下标 + 倒车锁存.
// 倒车必须带迟滞(1.4进/0.7出): 实测单阈值1.2会在err≈-1.2处正反抖振成极限环.
struct GuideState { size_t idx{0}; bool reversing{false}; };
// ex/ey/eth: 用于推进+算err的位姿(真值或odom估计).
// steer单位由调用方PID的out_lim决定: 自行车=delta_max(rad), 差速=w_max(rad/s).
// tv: 前进速度(m/s), 倒车恢复时为负.
struct GuideOut { double err{0}, steer{0}, tv{0}; bool reached{false}; };
inline GuideOut guide_step(const std::vector<Vec2>& path, GuideState& st,
  double ex, double ey, double eth, PID& pid,
  double Ld, double cruise, double dt, const GridMap& cost){
  advance_wp(path,st.idx,ex,ey,eth,0.6,Ld);
  if(st.idx>=path.size()) return {0,0,0,true};
  Vec2 la=lookahead_pt(path,st.idx,ex,ey,Ld,cost);
  double err=heading_err(la,ex,ey,eth);
  double steer_fwd=pid.update(err,dt);
  if(!st.reversing && std::abs(err)>1.4) st.reversing=true;
  else if(st.reversing && std::abs(err)<0.7) st.reversing=false;
  double tv, steer;
  if(st.reversing){ tv=-0.4; steer=-steer_fwd; } // 目标在身后:倒车回转
  else { steer=steer_fwd; tv=sched_v(err,cruise); }
  return {err,steer,tv,false};
}
// map做碰撞guard.
inline FollowOut follow_step(const std::vector<Vec2>& path, GuideState& st,
  Bicycle& bike, BikeState& bs_true, PID& pid,
  double ex, double ey, double eth, double Ld, double cruise, double dt,
  const GridMap& truth, const GridMap& cost){
  GuideOut g=guide_step(path,st,ex,ey,eth,pid,Ld,cruise,dt,cost);
  if(g.reached) return {0,0,0,true,false};
  BikeState cand=bike.step(bs_true,{g.tv,g.steer},dt);
  if(!truth.isFree(cand.x,cand.y)) return {g.err,g.steer,g.tv,false,true};
  bs_true=cand;
  return {g.err,g.steer,g.tv,false,false};
}
}
