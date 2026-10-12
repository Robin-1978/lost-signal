#include "game/task.hpp"
#include "core/diff_drive.hpp"
#include <cstdio>
#define REQ(cond) do{ if(!(cond)){ printf("[FAIL] %s:%d: %s\n",__FILE__,__LINE__,#cond); return 1; } }while(0)
static bool sameMetrics(const task::Metrics& a, const task::Metrics& b){
  return a.success==b.success && !__builtin_strcmp(a.fail_reason,b.fail_reason)
    && a.time_used==b.time_used && a.path_len==b.path_len && a.energy==b.energy
    && a.min_clear==b.min_clear && a.drift_end==b.drift_end && a.min_dyn_clear==b.min_dyn_clear
    && a.min_via_gap==b.min_via_gap
    && a.collisions==b.collisions && a.astar_expanded==b.astar_expanded && a.path_pts==b.path_pts
    && a.dwa_steps==b.dwa_steps && a.dwa_stops==b.dwa_stops && a.replans==b.replans
    && a.event_mask==b.event_mask && a.n_events==b.n_events;
}
static void show(const char* tag, const task::Metrics& m){
  printf("[%s] ok=%d why=%s t=%.1f len=%.1f E=%.1f clr=%.2f drift=%.2f hit=%d exp=%d pts=%zu dynclr=%.2f dwastep=%d dwastop=%d viagap=%.2f repl=%d evmask=0x%x nev=%zu\n",
    tag,(int)m.success,m.fail_reason,m.time_used,m.path_len,m.energy,
    m.min_clear,m.drift_end,m.collisions,m.astar_expanded,m.path_pts,
    m.min_dyn_clear,m.dwa_steps,m.dwa_stops,m.min_via_gap,m.replans,m.event_mask,m.n_events);
}
int main(){
  using namespace task;
  auto ts=allTasks();
  REQ(ts.size()==7);
  Loadout good; // 默认好配置
  // T1: 空房直线,好胎真值 -> 过
  { auto m=run_task(ts[0],good); show("t1-reach",m); REQ(m.success); }
  // T2: 有lidar停住 vs 瞎子撞墙(含COLLISION事件)
  { auto m=run_task(ts[1],good); show("t2-eye",m); REQ(m.success);
    Loadout blind=good; blind.has_lidar=false;
    sim::EventLog ev;
    auto b=run_task(ts[1],blind,nullptr,&ev); show("t2-blind",b); REQ(!b.success && b.collisions>0);
    REQ(b.event_mask&(1u<<sim::EV_COLLISION));
    printf("[ok] t2-blind events=%zu\n",ev.events().size()); }
  // T3: 好胎odom跟随过 vs 烂胎漂移丢(含DRIFT_HIGH事件)
  { Loadout odo=good; odo.use_odom=true; odo.slip_scale=0.2;
    auto m=run_task(ts[2],odo); show("t3-odo-good",m); REQ(m.success);
    Loadout bad=odo; bad.slip_scale=3.0;
    sim::EventLog ev;
    auto b=run_task(ts[2],bad,nullptr,&ev); show("t3-odo-bad",b); REQ(!b.success);
    REQ(b.event_mask&(1u<<sim::EV_DRIFT_HIGH)); }
  // T4: 膨胀三态: 0.05险过(净空小) / 0.4稳过 / 1.5封死无路
  { Loadout ok=good; ok.inflation=0.4;
    auto m=run_task(ts[3],ok); show("t4-infl-ok",m); REQ(m.success);
    Loadout fat=good; fat.inflation=1.5;
    auto b=run_task(ts[3],fat); show("t4-infl-fat",b); REQ(!b.success);
    Loadout thin=good; thin.inflation=0.05;
    auto t=run_task(ts[3],thin); show("t4-infl-thin",t); REQ(t.success); REQ(t.min_clear<=m.min_clear); }
  // T5: h_w=1与3都过, expanded取舍可见
  { Loadout h1=good; h1.h_weight=1.0;
    Loadout h3=good; h3.h_weight=3.0;
    auto m1=run_task(ts[4],h1); show("t5-hw1",m1);
    auto m3=run_task(ts[4],h3); show("t5-hw3",m3);
    REQ(m1.success && m3.success);
    REQ(m1.astar_expanded>=m3.astar_expanded); }
  // T6: 有DWA避开叉车过 / 无DWA致盲撞上 + recorder重放 + 确定性复跑
  { Loadout dw=good; dw.use_dwa=true;
    sim::Recorder rec; sim::EventLog ev;
    auto m=run_task(ts[5],dw,&rec,&ev); show("t6-dwa",m); REQ(m.success);
    sim::DiffDrive dd;
    REQ(rec.size()>0 && rec.replay(dd,0.01));
    printf("[ok] t6-dwa replay %zu steps identical\n",rec.size());
    REQ(m.event_mask&(1u<<sim::EV_DYN_NEAR));
    REQ(m.event_mask&(1u<<sim::EV_VIA));
    REQ(m.event_mask&(1u<<sim::EV_REACHED));
    Loadout blind=good; blind.use_dwa=false;
    sim::EventLog ev2;
    auto b=run_task(ts[5],blind,nullptr,&ev2); show("t6-nodwa",b); REQ(!b.success && b.collisions>0);
    REQ(b.event_mask&(1u<<sim::EV_DYN_HIT));
    sim::Recorder rec2; sim::EventLog ev3;
    auto m2=run_task(ts[5],dw,&rec2,&ev3); REQ(sameMetrics(m,m2));
    REQ(rec2.replay(dd,0.01));
    printf("[ok] deterministic rerun identical\n"); }
  // T7: 乐观+重规划闭环过 / 乐观无重规划撞墙 / 保守无路超时
  { Loadout cl=good; cl.fog=true; cl.unknown_free=true; cl.replan=true;
    auto m=run_task(ts[6],cl); show("t7-fog-replan",m); REQ(m.success); REQ(m.replans>0);
    Loadout open=cl; open.replan=false;
    auto b=run_task(ts[6],open); show("t7-fog-open",b); REQ(!b.success && b.collisions>0);
    Loadout cons=cl; cons.unknown_free=false;
    auto c=run_task(ts[6],cons); show("t7-fog-cons",c); REQ(!c.success); }
  printf("ALL TASK TESTS PASSED\n");
  return 0;
}
