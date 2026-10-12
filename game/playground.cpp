#include "playground.hpp"
#include "../core/astar.hpp"
#include "../game/level.hpp"
#include <algorithm>
#include <cmath>

namespace play {
using namespace sim;

PlaygroundSim::PlaygroundSim(uint32_t seed)
    : map_(buildMaze()),
      cost_(map_.inflated(0.4)),
      start_(mazeStart()),
      goal_(mazeGoal()),
      rng_(seed),
      fog_(map_.w(), map_.h(), map_.res()),
      patrols_(demoPatrols()) {
  bs_ = {start_.x, start_.y, 0, 0, 0};
  ts_ = {start_.x, start_.y, 0, 0, 0};
  pid_ = PID(2.0, 0.2, 0.15, 0.6, 2.0);
  odom_.reset(bs_.x, bs_.y, bs_.theta);
  fog_.reset(start_.x, start_.y);
  astar_ = astar_plan(cost_, start_, goal_, h_w_);
  scan_.n_rays = 72;
  scan_.max_range = 8.0;
  scan_.ranges.assign(72, 8.0);
  scan_.hit.assign(72, 0);
  ev_.clear();
}

void PlaygroundSim::rebuildCostAndPlan() {
  cost_ = map_.inflated(0.4);
  astar_ = astar_plan(cost_, start_, goal_, h_w_);
}

void PlaygroundSim::reset() {
  bs_ = {start_.x, start_.y, 0, 0, 0};
  gst_ = GuideState();
  pid_.reset();
  dwa_pos_ = {1, 3};
  dwa_th_ = 0;
  dwa_v_ = dwa_w_ = 0;
  dst_ = GuideState();
  ts_ = {start_.x, start_.y, 0, 0, 0};
  if (mode_ == Mode::Teleop)
    odom_.reset(ts_.x, ts_.y, ts_.theta);
  else if (mode_ == Mode::Dwa)
    odom_.reset(dwa_pos_.x, dwa_pos_.y, dwa_th_);
  else
    odom_.reset(bs_.x, bs_.y, bs_.theta);
  ev_.clear();
  sim_t_ = 0;
  fog_tick_ = 0;
  step_count_ = 0;
  last_dwa_ = DwaResult();
  last_err_ = last_p_ = last_i_ = last_d_ = 0;
}

void PlaygroundSim::resetAll() {
  fog_ = FogMap(map_.w(), map_.h(), map_.res());
  fog_.reset(start_.x, start_.y);
  rebuildCostAndPlan();
  reset();
}

void PlaygroundSim::setMode(Mode m) {
  if (m == mode_) return;
  mode_ = m;
  if (mode_ == Mode::Teleop)
    odom_.reset(ts_.x, ts_.y, ts_.theta);
  else if (mode_ == Mode::Dwa)
    odom_.reset(dwa_pos_.x, dwa_pos_.y, dwa_th_);
  else
    odom_.reset(bs_.x, bs_.y, bs_.theta);
}

void PlaygroundSim::setHw(double h) {
  h = std::min(5.0, std::max(1.0, h));
  if (h == h_w_) return;
  h_w_ = h;
  astar_ = astar_plan(cost_, start_, goal_, h_w_);
  gst_.idx = 0;
  gst_.reversing = false;
  dst_.idx = 0;
  dst_.reversing = false;
}

void PlaygroundSim::setPid(double kp, double ki, double kd) { pid_.setGains(kp, ki, kd); }

void PlaygroundSim::tuneDwa(double d_goal, double d_clear, double d_speed) {
  dwt_.w_goal = std::max(0.0, dwt_.w_goal + d_goal);
  dwt_.w_clear = std::max(0.0, dwt_.w_clear + d_clear);
  dwt_.w_speed = std::max(0.0, dwt_.w_speed + d_speed);
}

void PlaygroundSim::setFog(bool on) { fog_on_ = on; }
void PlaygroundSim::setSlip(double s) { slip_scale_ = s; }
void PlaygroundSim::setLidarNoise(double n) { lidar_noise_ = n; }

void PlaygroundSim::kidnap(double dx, double dy) {
  bs_.x += dx;
  bs_.y += dy;
  ts_.x += dx;
  ts_.y += dy;
  dwa_pos_.x += dx;
  dwa_pos_.y += dy;
}

void PlaygroundSim::activePose(double& x, double& y, double& th) const {  if (mode_ == Mode::Dwa) {
    x = dwa_pos_.x;
    y = dwa_pos_.y;
    th = dwa_th_;
  } else if (mode_ == Mode::Teleop) {
    x = ts_.x;
    y = ts_.y;
    th = ts_.theta;
  } else {
    x = bs_.x;
    y = bs_.y;
    th = bs_.theta;
  }
}

size_t PlaygroundSim::wpIdx() const {
  if (mode_ == Mode::Dwa) return dst_.idx;
  return gst_.idx;
}

void PlaygroundSim::followActive(double ex, double ey, double eth) {
  auto fo = follow_step(astar_.path, gst_, bike_, bs_, pid_, ex, ey, eth, 2.0, 1.0, DT, map_, cost_);
  if (fo.reached || fo.blocked) {
    if (fo.blocked) {
      bs_.v = 0;
      ev_.log(sim_t_, EV_COLLISION);
    }
  } else {
    double w = (std::abs(bs_.v) > 1e-6) ? bs_.v / 2.0 * std::tan(bs_.delta) : 0;
    odom_.update(bs_.v, w, DT, rng_, slip_scale_, 0.02 * slip_scale_);
    last_err_ = fo.err;
    last_p_ = pid_.p();
    last_i_ = pid_.i();
    last_d_ = pid_.d();
  }
}

void PlaygroundSim::step(TeleopInput in, int n) {
  for (int k = 0; k < n; k++) {
    if (mode_ == Mode::Teleop) {
      double tv = 0, tw = 0;
      if (in.up) tv = 1.0;
      if (in.down) tv = -0.5;
      if (in.left) tw = 1.2;
      if (in.right) tw = -1.2;
      DiffState cand = ddrv_.step(ts_, {tv, tw}, DT);
      if (map_.isFree(cand.x, cand.y))
        ts_ = cand;
      else {
        ts_.v = 0;
        ts_.w = 0;
        ev_.log(sim_t_, EV_COLLISION);
      }
      odom_.update(ts_.v, ts_.w, DT, rng_, slip_scale_, 0.02 * slip_scale_);
    } else if ((mode_ == Mode::Pid || mode_ == Mode::Astar) && gst_.idx < astar_.path.size()) {
      followActive(bs_.x, bs_.y, bs_.theta);
    } else if (mode_ == Mode::Odom && gst_.idx < astar_.path.size()) {
      followActive(odom_.p.x, odom_.p.y, odom_.th);
    } else if (mode_ == Mode::Pid || mode_ == Mode::Astar || mode_ == Mode::Odom) {
      double w = (std::abs(bs_.v) > 1e-6) ? bs_.v / 2.0 * std::tan(bs_.delta) : 0;
      odom_.update(bs_.v, w, DT, rng_, slip_scale_, 0.02 * slip_scale_);
    }
    if (mode_ == Mode::Dwa) {
      std::vector<DynState> dyn;
      for (auto& p : patrols_) dyn.push_back(patrol_state(p, sim_t_));
      advance_wp(astar_.path, dst_.idx, dwa_pos_.x, dwa_pos_.y, dwa_th_, 0.6, 2.0);
      if (dst_.idx < astar_.path.size()) {
        Vec2 la = lookahead_pt(astar_.path, dst_.idx, dwa_pos_.x, dwa_pos_.y, 2.0, cost_);
        double la_err = heading_err(la, dwa_pos_.x, dwa_pos_.y, dwa_th_);
        double v_cap = std::max(0.25, sched_v(la_err, 1.0));
        auto r = dwa_compute_dyn({dwa_pos_.x, dwa_pos_.y}, dwa_th_, dwa_v_, dwa_w_, la, cost_, dwt_,
                                 dyn, v_cap, 2.0, 3.0, 2.5, 0.1);
        last_dwa_ = r;
        if (r.best.valid) {
          dwa_v_ = r.best.v;
          dwa_w_ = r.best.w;
        } else {
          dwa_v_ = 0;
          dwa_w_ = 0;
          ev_.log(sim_t_, EV_DWA_STOP);
        }
      }
      double nx = dwa_pos_.x + dwa_v_ * std::cos(dwa_th_) * DT;
      double ny = dwa_pos_.y + dwa_v_ * std::sin(dwa_th_) * DT;
      bool dynhit = false;
      for (auto& s : dyn)
        if (std::hypot(nx - s.p.x, ny - s.p.y) < s.radius + kRobotRadius) dynhit = true;
      if (map_.isFree(nx, ny) && !dynhit) {
        dwa_pos_.x = nx;
        dwa_pos_.y = ny;
        dwa_th_ += dwa_w_ * DT;
      } else {
        dwa_v_ = 0;
        dwa_w_ = 0;
        ev_.log(sim_t_, map_.isFree(nx, ny) ? EV_DYN_HIT : EV_COLLISION);
      }
      odom_.update(dwa_v_, dwa_w_, DT, rng_, slip_scale_, 0.02 * slip_scale_);
    }
    sim_t_ += DT;
    step_count_++;
    // lidar固定节拍(确定性RNG流) + fog揭示
    if (step_count_ % SCAN_EVERY == 0) {
      double rx, ry, rth;
      activePose(rx, ry, rth);
      scan_ = lidar_scan(map_, {rx, ry}, rth, 72, 8.0, lidar_noise_, rng_);
      if (fog_on_) fog_.reveal({rx, ry}, rth, scan_);
    }
    if (fog_on_ && (fog_tick_++ % REPLAN_EVERY == 0) && fog_tick_ > 1) {
      double rx, ry, rth;
      activePose(rx, ry, rth);
      GridMap pg = fog_.planningGrid(true).inflated(0.4);
      auto np = astar_plan(pg, {rx, ry}, goal_, h_w_);
      if (np.ok) {
        cost_ = pg;
        astar_ = np;
        gst_.idx = 0;
        gst_.reversing = false;
        dst_.idx = 0;
        dst_.reversing = false;
      }
    }
    // 状态类事件(同码节流, 每步检查便宜)
    {
      double rx, ry, rth;
      activePose(rx, ry, rth);
      double dr = odom_.driftErr({rx, ry});
      if (dr > 1.0) ev_.log(sim_t_, EV_DRIFT_HIGH, (float)dr, 5.0);
    }
    if (mode_ == Mode::Dwa) {
      double rx, ry, rth;
      activePose(rx, ry, rth);
      double gap = 99;
      for (auto& p : patrols_) {
        DynState s = patrol_state(p, sim_t_);
        gap = std::min(gap, std::hypot(rx - s.p.x, ry - s.p.y) - s.radius - kRobotRadius);
      }
      if (gap < 0.8 && gap >= 0) ev_.log(sim_t_, EV_DYN_NEAR, (float)gap);
    }
  }
}
}  // namespace play
