#pragma once
#include "../core/grid_map.hpp"
#include "../core/bicycle.hpp"
#include "../core/diff_drive.hpp"
#include "../core/pid.hpp"
#include "../core/astar.hpp"
#include "../core/dwa.hpp"
#include "../core/sensor.hpp"
#include "../core/fogmap.hpp"
#include "../core/follow.hpp"
#include "../core/eventlog.hpp"
#include <cstdint>
#include <vector>
// Playground仿真(Core,无图形/SFML): 把app/main.cpp的固定步长行为收进来.
// 目标: 输入抽象(TeleopInput)+固定DT+固定scan/replan节拍, 确定性可测.
// 与旧app行为的刻意修正(修bug,不保逐位姿一致):
//  1) lidar/fog揭示从wall-clock节流(20Hz)改为固定5步(20Hz@DT=0.01), RNG流确定.
//  2) DWA仿真步用v_cap(与task runner同律), 不再是固定2.0; view不再另算一套.
//  3) fog重规划从当前活动机器人位姿出发, 不再恒用自行车位姿.
namespace play {
enum class Mode { Teleop = 0, Pid = 1, Astar = 2, Dwa = 3, Odom = 4 };
struct TeleopInput { bool up{false}, down{false}, left{false}, right{false}; };

class PlaygroundSim {
 public:
  static constexpr double DT = 0.01;
  static constexpr int SCAN_EVERY = 5;    // 20Hz
  static constexpr int REPLAN_EVERY = 200;  // 2s

  explicit PlaygroundSim(uint32_t seed = 42);

  void reset();  // 位姿/引导/PID/事件/时间复位, RNG种子不变(连续流, 与app一致)
  void resetAll();  // reset + fog/cost/astar重建(R键语义)
  void setMode(Mode m);
  void setHw(double h);  // 钳制[1,5], 重规划+indices复位
  void setPid(double kp, double ki, double kd);
  void tuneDwa(double d_goal, double d_clear, double d_speed);
  void setFog(bool on);
  void setSlip(double s);
  void setLidarNoise(double n);
  void kidnap(double dx, double dy);  // 真值跳变, odom不知

  void step(TeleopInput in = {}, int n = 1);

  // ---- view/测试只读 ----
  Mode mode() const { return mode_; }
  double simTime() const { return sim_t_; }
  double hw() const { return h_w_; }
  const sim::AStarResult& astar() const { return astar_; }
  const sim::BikeState& bike() const { return bs_; }
  const sim::DiffState& teleop() const { return ts_; }
  sim::Vec2 dwaPos() const { return dwa_pos_; }
  double dwaTheta() const { return dwa_th_; }
  double dwaV() const { return dwa_v_; }
  double dwaW() const { return dwa_w_; }
  const sim::DeadReckon& odom() const { return odom_; }
  const sim::Scan& scan() const { return scan_; }
  const sim::FogMap& fog() const { return fog_; }
  const sim::EventLog& events() const { return ev_; }
  const sim::DwaResult& lastDwa() const { return last_dwa_; }
  const sim::DwaWeights& dwaWeights() const { return dwt_; }
  const sim::PID& pid() const { return pid_; }
  double lidarNoise() const { return lidar_noise_; }
  double slip() const { return slip_scale_; }
  bool fogOn() const { return fog_on_; }
  double lastErr() const { return last_err_; }
  double lastP() const { return last_p_; }
  double lastI() const { return last_i_; }
  double lastD() const { return last_d_; }
  // 当前活动机器人位姿(与view的rx/ry/rth同义)
  void activePose(double& x, double& y, double& th) const;
  const sim::GridMap& truthMap() const { return map_; }
  const sim::Vec2& goalPt() const { return goal_; }
  size_t wpIdx() const;
  size_t wpTotal() const { return astar_.path.size(); }

 private:
  void rebuildCostAndPlan();
  void followActive(double ex, double ey, double eth);

  sim::GridMap map_, cost_;
  sim::Vec2 start_, goal_;
  double h_w_{1.5};
  sim::AStarResult astar_;
  sim::Bicycle bike_;
  sim::BikeState bs_;
  sim::DiffDrive ddrv_;
  sim::DiffState ts_;
  sim::PID pid_;
  sim::DwaWeights dwt_{1.0, 2.0, 0.3};
  sim::Rng rng_;
  sim::DeadReckon odom_;
  double lidar_noise_{0.05}, slip_scale_{1.0};
  bool fog_on_{false};
  sim::FogMap fog_;
  int fog_tick_{0}, step_count_{0};
  sim::GuideState gst_, dst_;
  sim::Vec2 dwa_pos_{1, 3};
  double dwa_th_{0}, dwa_v_{0}, dwa_w_{0};
  double sim_t_{0};
  std::vector<sim::Patrol> patrols_;
  sim::DwaResult last_dwa_;
  sim::Scan scan_;
  sim::EventLog ev_;
  Mode mode_{Mode::Pid};
  double last_err_{0}, last_p_{0}, last_i_{0}, last_d_{0};
};
}  // namespace play
