#include "dwa.hpp"
#include <cmath>
#include <limits>
namespace sim {
DwaResult dwa_compute(Vec2 pos,double theta,double cur_v,double cur_w,Vec2 goal,
  const GridMap& map,DwaWeights wt,double v_max,double w_max,double accel,double predict_T,double dt){
  return dwa_compute_dyn(pos,theta,cur_v,cur_w,goal,map,wt,{},v_max,w_max,accel,predict_T,dt);
}
DwaResult dwa_compute_dyn(Vec2 pos,double theta,double cur_v,double cur_w,Vec2 goal,
  const GridMap& map,DwaWeights wt,const std::vector<DynState>& dyn,
  double v_max,double w_max,double accel,double predict_T,double dt){
  auto dyn_hit=[&](double x,double y,double t)->bool{
    for(auto &s:dyn){
      double px=s.p.x+s.v.x*t, py=s.p.y+s.v.y*t;
      if(std::hypot(x-px,y-py)<s.radius+kRobotRadius) return true;
    }
    return false;
  };
  DwaResult out; out.best.valid=false; out.best.score=-1e18;
  for(double v=0; v<=v_max+1e-9; v+=0.25){
    if(std::abs(v-cur_v)>accel*predict_T) continue; // dynamic window
    for(double w=-w_max; w<=w_max+1e-9; w+=0.3){
      double x=pos.x,y=pos.y,th=theta,clear=predict_T,min_d=1e9;
      bool hit=false;
      for(double t=0;t<predict_T;t+=dt){
        x+=v*std::cos(th)*dt; y+=v*std::sin(th)*dt; th+=w*dt;
        if(!map.isFree(x,y)||dyn_hit(x,y,t)){ hit=true; clear=t; break; }
        double dg=std::hypot(goal.x-x,goal.y-y);
        min_d=std::min(min_d,dg);
      }
      double goal_score = -std::hypot(goal.x-x,goal.y-y);
      double clear_score = hit ? clear : predict_T+1.0;
      double score = wt.w_goal*goal_score + wt.w_clear*clear_score + wt.w_speed*v;
      out.rollouts.push_back({{x,y},score});
      if(!hit && score>out.best.score){ out.best={v,w,true,score}; }
    }
  }
  return out;
}
}
