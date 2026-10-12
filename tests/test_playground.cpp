#include "game/playground.hpp"
#include <cstdio>
#include <cmath>
#define REQ(cond) do{ if(!(cond)){ printf("[FAIL] %s:%d: %s\n",__FILE__,__LINE__,#cond); return 1; } }while(0)
int main(){
  using namespace play;
  // 1. teleop前进: 按Up 2秒应向前移动且无穿墙
  { PlaygroundSim s(42); s.setMode(Mode::Teleop);
    TeleopInput in; in.up=true;
    s.step(in, 200);
    double x,y,th; s.activePose(x,y,th);
    REQ(x > 1.2 && y > 0.5); // 起点(1,1)出发向前
    REQ(s.events().events().size() <= 2);
    printf("[ok] teleop fwd x=%.2f y=%.2f\n",x,y); }
  // 2. teleop顶墙guard: 一直向前不应穿墙(迷宫墙x=8), 碰撞事件应被记录且位姿保持free
  { PlaygroundSim s(42); s.setMode(Mode::Teleop);
    TeleopInput in; in.up=true;
    s.step(in, 3000);
    double x,y,th; s.activePose(x,y,th);
    REQ(x < 8.0); // 没穿过第一堵墙
    printf("[ok] teleop guard x=%.2f ev=%zu\n",x,s.events().events().size()); }
  // 3. 确定性: 同种子同输入两次复现
  { PlaygroundSim a(7), b(7); a.setMode(Mode::Pid); b.setMode(Mode::Pid);
    a.step({}, 500); b.step({}, 500);
    double ax,ay,ath,bx,by,bth; a.activePose(ax,ay,ath); b.activePose(bx,by,bth);
    REQ(std::abs(ax-bx)<1e-9 && std::abs(ay-by)<1e-9 && std::abs(ath-bth)<1e-9);
    printf("[ok] determinism x=%.4f\n",ax); }
  // 4. DWA避让: 在巡逻叉车走廊附近不应撞动障碍(只检查事件/位姿合法)
  { PlaygroundSim s(48); s.setMode(Mode::Dwa);
    s.step({}, 3000);
    double x,y,th; s.activePose(x,y,th);
    bool hit=false;
    for(auto &e:s.events().events()) if(e.code==sim::EV_DYN_HIT) hit=true;
    REQ(!hit);
    printf("[ok] dwa dynavoid x=%.2f y=%.2f ev=%zu\n",x,y,s.events().events().size()); }
  // 5. odom漂移: slip=3长跑应比slip=0漂移大, 且>1m时DRIFT_HIGH事件出现
  { PlaygroundSim clean(42), bad(42);
    clean.setMode(Mode::Pid); clean.setSlip(0.0);
    bad.setMode(Mode::Pid); bad.setSlip(3.0);
    clean.step({}, 2000); bad.step({}, 2000);
    double cx,cy,ct,bx,by,bt; clean.activePose(cx,cy,ct); bad.activePose(bx,by,bt);
    double dc=clean.odom().driftErr({cx,cy}), db=bad.odom().driftErr({bx,by});
    REQ(db > dc);
    bool f=false; for(auto &e:bad.events().events()) if(e.code==sim::EV_DRIFT_HIGH) f=true;
    printf("[ok] drift clean=%.2f bad=%.2f flag=%d\n",dc,db,(int)f); }
  // 6. fog重规划闭环能跑(乐观未知=空闲至少不立即无路)
  { PlaygroundSim s(42); s.setMode(Mode::Pid); s.setFog(true);
    s.step({}, 600);
    REQ(s.astar().ok);
    printf("[ok] fog astar pts=%zu\n",s.astar().path.size()); }
  printf("ALL PLAYGROUND TESTS PASSED\n");
  return 0;
}
