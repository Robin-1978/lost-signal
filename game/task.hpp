#pragma once
#include "../core/grid_map.hpp"
#include "../core/math.hpp"
#include "../core/dynobs.hpp"
#include "../core/dwa.hpp"
#include "../core/eventlog.hpp"
#include "../core/recorder.hpp"
#include <cstdint>
#include <vector>
// 统一任务接口(M3): 每个任务=起点/目标/障碍/约束/种子 + 评价指标.
// Loadout=机器人能力配置("改造"向量): 同一任务换Loadout结果不同,这就是诊断玩法的地基.
namespace task {
enum class MapKind { ROOM, ROOM_BOX, MAZE };
enum class GoalKind { REACH, OBSERVE };
struct TaskConfig {
  int id{0}; const char* name{""}; const char* brief{""};
  MapKind map{MapKind::ROOM}; GoalKind goal{GoalKind::REACH};
  sim::Vec2 start{1,1}, goal_pt{8,6}, via_pt{0,0};
  bool has_via{false};
  double theta0{0}, goal_tol{0.5}, time_limit{120}, observe_range{2.0};
  int max_collisions{0};
  uint32_t seed{42};
  std::vector<sim::Patrol> patrols; // 动障碍(t6), 空=静态关
};
// 能力配置: 感知/底盘/定位/规划. 改一项,行为真会变(不是数值皮肤).
struct Loadout {
  bool has_lidar{true}; double lidar_range{8.0}, lidar_noise{0.05};
  double slip_scale{1.0};
  bool use_odom{false};
  double inflation{0.4}, h_weight{1.5}, cruise{1.0}, Ld{2.0};
  double kp{2.0}, ki{0.2}, kd{0.15};
  // 局部避障: 关=只跟全局路径(对动障碍致盲)
  bool use_dwa{false}; sim::DwaWeights dwa{1.0,2.0,0.3};
  // 雾战: 开=只用自己扫过的格子规划, 需配合replan形成感知-规划闭环
  bool fog{false}, unknown_free{true}, replan{true};
  int replan_interval{100}; // 步数(100步=1s)
};
struct Metrics {
  bool success{false}; const char* fail_reason{""};
  double time_used{0}, path_len{0}, energy{0}, min_clear{99}, drift_end{0};
  double min_dyn_clear{99}; // 与动障碍最小距离(无动障碍=99)
  double min_via_gap{99}; // 与经停点最小距离(无经停=99)
  int collisions{0}, astar_expanded{0};
  int dwa_steps{0}, dwa_stops{0}; // DWA接管步数 / 无可行指令停车次数
  int replans{0}; // fog重规划次数
  size_t path_pts{0};
  uint32_t event_mask{0}; // EventLog位图(诊断用)
  size_t n_events{0};
};
sim::GridMap buildTaskMap(MapKind k);
std::vector<TaskConfig> allTasks(); // 7关MVP链
// rec/ev可选: 非空则记录逐步指令与事件(View/测试用, 不影响仿真行为)
Metrics run_task(const TaskConfig& cfg, const Loadout& lo, sim::Recorder* rec=nullptr, sim::EventLog* ev=nullptr);
}
