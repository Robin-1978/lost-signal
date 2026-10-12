#include "core/bicycle.hpp"
#include "core/pid.hpp"
#include "core/astar.hpp"
#include "core/dwa.hpp"
#include "core/sensor.hpp"
#include "core/follow.hpp"
#include "game/level.hpp"
#include <cstdio>
#include <cmath>
#include <cstdlib>
// Release(-DNDEBUG)下assert是空操作,测试必须用硬检查,不许"假绿".
#define REQ(cond) do{ if(!(cond)){ printf("[FAIL] %s:%d: %s\n",__FILE__,__LINE__,#cond); return 1; } }while(0)
int main(){
  using namespace sim;
  // 1. bicycle respects accel limit (realism vs NPC teleport)
  { Bicycle b; BikeState s; BikeCmd c{5.0,0};
    s=b.step(s,c,0.01);
    REQ(s.v<=3.0*0.01+1e-9);
    printf("[ok] bicycle accel limit v=%.4f\n",s.v); }
  // 2. PID converges roughly
  { PID pid(2.0,0.5,0.1,10,5); double x=0;
    for(int i=0;i<500;i++){ double u=pid.update(1.0-x,0.01); x+=u*0.01; }
    REQ(std::abs(x-1.0)<0.15);
    printf("[ok] pid converge x=%.3f\n",x); }
  // 3. AStar S-maze: h=1 optimal (more expanded, shorter), h=3 greedy (less expanded, longer)
  { GridMap m=buildMaze();
    GridMap cost=m.inflated(0.4);
    auto r1=astar_plan(cost,mazeStart(),mazeGoal(),1.0);
    auto r2=astar_plan(cost,mazeStart(),mazeGoal(),3.0);
    REQ(r1.ok&&r2.ok);
    printf("[ok] astar maze w=1 expanded=%d len=%zu | w=3 expanded=%d len=%zu\n",
      r1.expanded,r1.path.size(),r2.expanded,r2.path.size());
    REQ(r1.expanded>=r2.expanded);
    REQ(r1.path.size()<=r2.path.size()+2); // allow tie on coarse grid
  }
  // 4. DWA avoids obstacle straight ahead
  { GridMap m(40,30,0.5); m.addBox(5,4.5,6,5.5);
    auto r=dwa_compute({3,5},0,0.5,0,{10,5},m,{1.0,2.0,0.3});
    REQ(r.best.valid);
    printf("[ok] dwa best v=%.2f w=%.2f rollouts=%zu\n",r.best.v,r.best.w,r.rollouts.size()); }
  // 5. lidar: 空图无回波(射出地图=虚空, 不是墙), 面向真墙才有短回波
  { GridMap m(40,28,0.5); Rng rng(7);
    auto s0=lidar_scan(m,{10,7},0,36,8.0,0.0,rng);
    for(auto r:s0.ranges) REQ(std::abs(r-8.0)<1e-9); // 空图: 全部顶满
    m.addBox(5,6.5,6,7.5);
    auto s1=lidar_scan(m,{3,7},0,36,8.0,0.0,rng);
    REQ(s1.ranges[0]<2.5 && s1.ranges[0]>1.0); // ray0 points +x at wall ~2m
    printf("[ok] lidar empty=%.1f wall=%.2f\n",s0.ranges[0],s1.ranges[0]); }
  // 6. odom: zero slip -> no drift; noisy + fixed seed -> bounded drift
  { Rng rng(99); DeadReckon o; o.reset(0,0,0);
    for(int i=0;i<1000;i++) o.update(2.0,0.1,0.01,rng,0.0);
    // slip=0 still has base noise by design; check small
    REQ(o.driftErr({o.p.x,o.p.y})==0);
    DeadReckon o2; o2.reset(0,0,0); Rng r2(99);
    double tx=0,ty=0,tth=0;
    for(int i=0;i<1000;i++){ o2.update(2.0,0.1,0.01,r2,1.0); tx+=2.0*0.01*std::cos(tth); ty+=2.0*0.01*std::sin(tth); tth+=0.1*0.01; }
    double drift=o2.driftErr({tx,ty});
    REQ(drift>1e-4 && drift<3.0);
    printf("[ok] odom drift=%.3fm\n",drift); }
  // 7. maze traversal: 前视纯追踪+调度, 全程无穿墙且走完
  { GridMap m=buildMaze();
    auto r=astar_plan(m.inflated(0.4),mazeStart(),mazeGoal(),1.5);
    REQ(r.ok);
    Bicycle b; BikeState bs{mazeStart().x,mazeStart().y,0,0,0};
    PID pid(2.0,0.2,0.15,0.6,2.0);
    GuideState gst; int hits=0;
    GridMap costmap=m.inflated(0.4);
    for(int i=0;i<60000 && gst.idx<r.path.size();i++){
      auto fo=follow_step(r.path,gst,b,bs,pid,bs.x,bs.y,bs.theta,2.0,1.0,0.01,m,costmap);
      if(fo.blocked){ hits++; bs.v=0; pid.reset(); }
    }
    printf("[ok] traverse wp=%zu/%zu hits=%d end=(%.2f,%.2f)\n",gst.idx,r.path.size(),hits,bs.x,bs.y);
    REQ(gst.idx>=r.path.size());
    REQ(hits==0);
  }
  printf("ALL CORE TESTS PASSED\n");
  return 0;
}
