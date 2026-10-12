#include "task.hpp"
#include "../core/astar.hpp"
#include "../core/diff_drive.hpp"
#include "../core/follow.hpp"
#include "../core/pid.hpp"
#include "../core/sensor.hpp"
#include "../core/fogmap.hpp"
#include "level.hpp"
#include <cmath>
namespace task {
using namespace sim;
static constexpr double DT=0.01;

GridMap buildTaskMap(MapKind k){
  if(k==MapKind::MAZE){
    GridMap m(40,28,0.5);
    m.addBox(8,0,9,11);
    m.addBox(12,3,13,14);
    return m;
  }
  GridMap m(20,16,0.5); // 10m x 8m空房,边界即墙
  if(k==MapKind::ROOM_BOX) m.addBox(5,3.5,6,4.5);
  return m;
}

std::vector<TaskConfig> allTasks(){
  std::vector<TaskConfig> t(7);
  t[0]={1,"醒来","手动/直线开出工作间",MapKind::ROOM,GoalKind::REACH,{1,1},{8,6},{0,0},false,0,0.5,60,2.0,0,42};
  t[1]={2,"第一只眼","看到障碍物并停下",MapKind::ROOM_BOX,GoalKind::OBSERVE,{1,4},{8,4},{0,0},false,0,0.5,60,2.0,0,43};
  t[2]={3,"撞墙之谜","8米直线,估计位姿跟随",MapKind::ROOM,GoalKind::REACH,{1,4},{9,4},{0,0},false,0,0.5,60,2.0,0,44};
  t[3]={4,"地图","膨胀层保护过S弯",MapKind::MAZE,GoalKind::REACH,{1,1},{17,10},{0,0},false,0,0.6,150,2.0,0,45};
  t[4]={5,"绕行","h_w取舍,都要能到",MapKind::MAZE,GoalKind::REACH,{1,1},{17,10},{0,0},false,0,0.6,150,2.0,0,46};
  t[5]={6,"运输","经停补给点再到能源室, 避开巡逻叉车",MapKind::MAZE,GoalKind::REACH,{1,1},{17,10},{10.5,12.5},true,0,0.6,200,2.0,0,47};
  t[5].patrols=demoPatrols();
  t[6]={7,"迷雾","未知仓库: 揭示+重规划闭环",MapKind::MAZE,GoalKind::REACH,{1,1},{17,10},{0,0},false,0,0.6,200,2.0,0,48};
  return t;
}

// 起点若落在膨胀致死格(贴墙时常见), 螺旋搜最近free格做恢复. 真实系统同理"DWA前先清车体 footprint".
static Vec2 snapFree(const GridMap& cost, Vec2 p){
  if(cost.isFree(p.x,p.y)) return p;
  for(double r=cost.res();r<1.5;r+=cost.res()*0.5)
    for(int k=0;k<16;k++){
      double a=2*M_PI*k/16, x=p.x+std::cos(a)*r, y=p.y+std::sin(a)*r;
      if(cost.isFree(x,y)) return {x,y};
    }
  return p;
}
// 无噪声 coarse 净空探针(8方向,不耗rng流)
static double probe_clear(const GridMap& m, double x, double y){
  double mn=8.0;
  for(int i=0;i<8;i++){
    double a=2*M_PI*i/8, ca=std::cos(a), sa=std::sin(a);
    for(double t=0.25;t<8.0;t+=0.25){
      if(!m.isFree(x+ca*t,y+sa*t)){ mn=std::min(mn,t); break; }
    }
  }
  return mn;
}

Metrics run_task(const TaskConfig& cfg, const Loadout& lo, Recorder* rec, EventLog* ev){
  Metrics m;
  GridMap map=buildTaskMap(cfg.map);
  DiffParams dp; // 默认v_max2/w_max2
  DiffDrive dd(dp);
  DiffState bs{cfg.start.x,cfg.start.y,cfg.theta0,0,0};
  PID pid(lo.kp,lo.ki,lo.kd,dp.w_max,2.0);
  DeadReckon odom; odom.reset(bs.x,bs.y,bs.theta);
  Rng rng(cfg.seed);
  int max_steps=(int)(cfg.time_limit/DT);
  if(rec) rec->begin(bs);
  if(ev) ev->clear();
  auto finish=[&](){ if(ev){ m.event_mask=ev->mask(); m.n_events=ev->events().size(); } };

  if(cfg.goal==GoalKind::OBSERVE){
    // 沿初始朝向直行,lidar看到observe_range内障碍就停; 瞎子(无lidar)会撞墙
    int stopped=0;
    for(int i=0;i<max_steps;i++){
      if(lo.has_lidar && i%10==0){
        Scan s=lidar_scan(map,{bs.x,bs.y},bs.theta,36,lo.lidar_range,lo.lidar_noise,rng);
        double cone=lo.lidar_range;
        for(int k=0;k<36;k++){
          double da=std::abs(wrap_angle(2*M_PI*k/36)); // ray k相对朝向偏角
          if(da<0.35) cone=std::min(cone,s.ranges[k]);
        }
        m.min_clear=std::min(m.min_clear,cone);
        if(cone<cfg.observe_range){ stopped++; bs.v=0; bs.w=0; if(stopped>=3) break; }
        else stopped=0;
      }
      double tv=(stopped>0)?0.0:0.8;
      DiffState cand=dd.step(bs,{tv,0},DT);
      if(!map.isFree(cand.x,cand.y)){ m.collisions++; m.fail_reason="collision-blind"; if(ev)ev->log(m.time_used,EV_COLLISION); break; }
      m.path_len+=std::hypot(cand.x-bs.x,cand.y-bs.y);
      m.energy+=(std::abs(cand.v)+0.5*std::abs(cand.w))*DT;
      bs=cand;
      if(rec) rec->push(tv,0,bs,false);
      odom.update(bs.v,bs.w,DT,rng,lo.slip_scale,0.02*lo.slip_scale);
      m.time_used+=DT;
    }
    m.drift_end=odom.driftErr({bs.x,bs.y});
    // 瞎子撞墙已记collision-blind, 这里只补默认原因, 不覆盖具体原因
    if(m.collisions>cfg.max_collisions){ m.success=false; if(!m.fail_reason[0]) m.fail_reason="collision"; }
    else if(stopped>=3 && m.min_clear>0.2){ m.success=true; }
    else { m.success=false; m.fail_reason="timeout-no-detect"; }
    finish();
    return m;
  }

  // REACH: 分段A* + guide跟随(真值或odom估计). fog开则只用自己扫过的格子+周期重规划.
  FogMap fog(map.w(),map.h(),map.res());
  if(lo.fog) fog.reset(cfg.start.x,cfg.start.y);
  GridMap cost = lo.fog ? fog.planningGrid(lo.unknown_free).inflated(lo.inflation)
                        : map.inflated(lo.inflation);
  std::vector<Vec2> full;
  auto do_plan=[&](Vec2 from, std::vector<Vec2>& out)->bool{
    from=snapFree(cost,from); // 致死起点恢复
    Vec2 mid=cfg.has_via?cfg.via_pt:cfg.goal_pt;
    auto r1=astar_plan(cost,from,mid,lo.h_weight);
    if(!r1.ok) return false;
    m.astar_expanded+=r1.expanded;
    out=r1.path;
    if(cfg.has_via){
      auto r2=astar_plan(cost,cfg.via_pt,cfg.goal_pt,lo.h_weight);
      if(!r2.ok) return false;
      m.astar_expanded+=r2.expanded;
      out.insert(out.end(),r2.path.begin()+1,r2.path.end());
    }
    return true;
  };
  if(!do_plan(cfg.start,full)){ m.fail_reason="no-path"; if(ev)ev->log(0,EV_NO_PATH); finish(); return m; }
  m.path_pts=full.size();
  GuideState gst;
  bool via_logged=false;
  auto dyn_now=[&](){ // 当前时刻动障碍状态(位置+解析速度)
    std::vector<DynState> d;
    for(auto &p:cfg.patrols) d.push_back(patrol_state(p,m.time_used));
    return d;
  };
  auto dyn_gap=[&](double x,double y,const std::vector<DynState>& d){ // 最近动障碍净距离
    double mn=99;
    for(auto &s:d) mn=std::min(mn,std::hypot(x-s.p.x,y-s.p.y)-s.radius-kRobotRadius);
    return mn;
  };
  for(int i=0;i<max_steps;i++){
    double ex=lo.use_odom?odom.p.x:bs.x, ey=lo.use_odom?odom.p.y:bs.y, eth=lo.use_odom?odom.th:bs.theta;
    if(lo.fog && lo.has_lidar && i%10==0){ // 10Hz揭示: 传感器物理真值播撒, 规划永不偷看
      Scan s=lidar_scan(map,{bs.x,bs.y},bs.theta,36,lo.lidar_range,lo.lidar_noise,rng);
      fog.reveal({bs.x,bs.y},bs.theta,s);
    }
    if(lo.fog && lo.replan && i>0 && i%lo.replan_interval==0){ // 感知-规划闭环: 从估计位姿重规划
      cost=fog.planningGrid(lo.unknown_free).inflated(lo.inflation);
      std::vector<Vec2> np;
      if(do_plan({ex,ey},np)){ full.swap(np); gst.idx=0; gst.reversing=false; m.replans++; m.path_pts=full.size(); }
      else if(ev) ev->log(m.time_used,EV_NO_PATH);
    }
    std::vector<DynState> dyn=dyn_now();
    double tv, tw;
    if(lo.use_dwa){
      // 全局guide只给前视局部目标, 速度指令由DWA(含动障碍外推)定
      advance_wp(full,gst.idx,ex,ey,eth,0.6,lo.Ld);
      if(gst.idx>=full.size()) break;
      Vec2 la=lookahead_pt(full,gst.idx,ex,ey,lo.Ld,cost);
      // costmap检查(保净空, 与全局一致) + v按朝向误差调度(与guide同律, 弯道自动慢)
      double la_err=heading_err(la,bs.x,bs.y,bs.theta);
      double v_cap=std::max(0.25,sched_v(la_err,lo.cruise));
      auto r=dwa_compute_dyn({bs.x,bs.y},bs.theta,bs.v,bs.w,la,cost,lo.dwa,dyn,v_cap,dp.w_max,dp.accel_max,2.5,0.1);
      m.dwa_steps++;
      if(r.best.valid){ tv=r.best.v; tw=r.best.w; }
      else { tv=0; tw=0; m.dwa_stops++; if(ev)ev->log(m.time_used,EV_DWA_STOP); } // 无可行指令: 原地停车等(不是穿过去)
    } else {
      GuideOut g=guide_step(full,gst,ex,ey,eth,pid,lo.Ld,lo.cruise,DT,cost);
      if(g.reached) break;
      tv=g.tv; tw=g.steer;
    }
    DiffState cand=dd.step(bs,{tv,tw},DT);
    if(!map.isFree(cand.x,cand.y)){
      m.collisions++; m.fail_reason="collision";
      bs.v=0; bs.w=0; // 先清零再记, 与replay语义一致
      if(rec) rec->push(tv,tw,bs,true);
      if(ev) ev->log(m.time_used,EV_COLLISION);
      if(m.collisions>cfg.max_collisions) break;
      pid.reset(); continue;
    }
    if(dyn_gap(cand.x,cand.y,dyn)<0){
      m.collisions++; m.fail_reason="collision-dyn";
      bs.v=0; bs.w=0;
      if(rec) rec->push(tv,tw,bs,true);
      if(ev) ev->log(m.time_used,EV_DYN_HIT);
      if(m.collisions>cfg.max_collisions) break;
      pid.reset(); continue;
    }
    m.path_len+=std::hypot(cand.x-bs.x,cand.y-bs.y);
    m.energy+=(std::abs(cand.v)+0.5*std::abs(cand.w))*DT;
    bs=cand;
    if(rec) rec->push(tv,tw,bs,false);
    odom.update(bs.v,bs.w,DT,rng,lo.slip_scale,0.02*lo.slip_scale);
    m.time_used+=DT;
    if(cfg.has_via){
      m.min_via_gap=std::min(m.min_via_gap,std::hypot(bs.x-cfg.via_pt.x,bs.y-cfg.via_pt.y));
      if(ev && !via_logged && m.min_via_gap<=1.0){ ev->log(m.time_used,EV_VIA); via_logged=true; }
    }
    { double gap=dyn_gap(bs.x,bs.y,dyn_now());
      m.min_dyn_clear=std::min(m.min_dyn_clear,gap);
      if(ev && gap<0.8 && gap>=0) ev->log(m.time_used,EV_DYN_NEAR,(float)gap); }
    if(ev){ double dr=odom.driftErr({bs.x,bs.y}); if(dr>1.0) ev->log(m.time_used,EV_DRIFT_HIGH,(float)dr,5.0); }
    if(i%50==0) m.min_clear=std::min(m.min_clear,probe_clear(map,bs.x,bs.y));
  }
  m.drift_end=odom.driftErr({bs.x,bs.y});
  double dg=std::hypot(cfg.goal_pt.x-bs.x,cfg.goal_pt.y-bs.y);
  if(m.collisions>cfg.max_collisions){ m.success=false; }
  else if(m.time_used>=cfg.time_limit-1e-9){ m.success=false; m.fail_reason="timeout"; if(ev)ev->log(m.time_used,EV_LOST); }
  else if(dg<=cfg.goal_tol){
    if(cfg.has_via && m.min_via_gap>1.0){ m.success=false; m.fail_reason="via-missed"; if(ev)ev->log(m.time_used,EV_LOST); }
    else { m.success=true; if(ev)ev->log(m.time_used,EV_REACHED); }
  }
  else { m.success=false; m.fail_reason=(gst.idx>=full.size())?"lost-drift":"timeout"; if(ev)ev->log(m.time_used,EV_LOST); }
  finish();
  return m;
}
}
