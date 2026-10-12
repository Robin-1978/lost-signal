#include "sensor.hpp"
#include <cmath>
namespace sim {
Scan lidar_scan(const GridMap& m, Vec2 pos, double theta,
  int n_rays, double max_range, double noise_std, Rng& rng){
  Scan s; s.n_rays=n_rays; s.max_range=max_range; s.ranges.reserve(n_rays); s.hit.reserve(n_rays);
  double step = m.res()*0.5;
  for(int i=0;i<n_rays;i++){
    double a = theta + 2*M_PI*i/n_rays;
    double ca=std::cos(a), sa=std::sin(a);
    double r=max_range; bool h=false;
    for(double t=step;t<max_range;t+=step){
      double x=pos.x+ca*t, y=pos.y+sa*t;
      auto [gx,gy]=m.worldToGrid(x,y);
      if(!m.inside(gx,gy)){ r=max_range; break; } // 射出地图=虚空无回波, 不是墙(否则fog会画满幻影墙)
      if(!m.isFreeCell(gx,gy)){ r=t; h=true; break; }
    }
    r += rng.gauss(noise_std); // 噪声只污染测量值, 不改变"有无命中"
    if(r<0) r=0; if(r>max_range) r=max_range;
    s.ranges.push_back(r); s.hit.push_back(h?1:0);
  }
  return s;
}
void DeadReckon::update(double v,double w,double dt,Rng& rng,double slip_scale,double yaw_bias){
  double dist=v*dt, dth=w*dt;
  double sig_d = 0.02*std::abs(dist)+0.0005;
  double sig_t = 0.05*std::abs(dth)+0.0008;
  double dn = dist + rng.gauss(sig_d)*slip_scale;
  double dt2 = dth + rng.gauss(sig_t)*slip_scale + yaw_bias*dist;
  th += dt2;
  p.x += dn*std::cos(th);
  p.y += dn*std::sin(th);
}
double DeadReckon::driftErr(Vec2 t) const { return std::hypot(t.x-p.x,t.y-p.y); }
}
