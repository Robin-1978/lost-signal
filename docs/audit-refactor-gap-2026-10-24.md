## 修复进展（2026-10-24）

以下问题已经在当前分支修复并带回归测试（本文件下方的旧分段保留为
审计证据，若与此处冲突以本节和源码为准）：

- P0-1 配置几何：`make_chassis` 全部读取 `RobotDescription::plant`，非法几何 fail closed。
- P0-2 扫描角度：新增 `iface::scan_beam_angle_rad()`，感知、建图、校验、安全统一使用；非零起始角与局部扇区已覆盖。
- P0-3 Validator：改为连续轨迹段的点-线段距离检查，补了采样间隙障碍物用例。
- P0-4 SLAM：submap 只保留与静态参考一致的返回，补了动态 actor 不落图测试。
- P1-3 仿真时钟：决策时刻使用 plant 时钟，观测保留采集时间，delivery latency 可触发 freshness。
- P1-4 DWA：速度窗口支持负速度，补了后方目标倒车用例。
- P1-6 目标生命周期：hold 清除 route；arrival 先完成目标并清 route，再规划，到点即停。
- P1-2/P1-3 多速率、plant-clock tick、延迟/dropout freshness：已接入
  `ScenarioSession::set_sensor_cadence()`、独立 scheduler taps 和真实
  `sim_stamp_ns`，见 `tests/test_realism.cpp`。
- P1-5 Ackermann/Holonomic：Hybrid-A*、gear 贯通、曲率门控和
  `DecisionCyclePipeline` 已接入；`tests/test_ackermann_planner.cpp` 与
  `tests/test_multichassis_mission.cpp` 覆盖四种底盘端到端。
- P2 地图语义：导航 world 仍只发布公开几何/高程先验；转换器额外输出
  `.semantics.json` SIM-only sidecar，保留 conflict zones、非 box collision
  和 dormant actor，不把游戏 authority 穿透给 ROBOT。
- 当前验证基线：`ctest --test-dir build --output-on-failure` 49/49，
  `tests/check_boundary.py` 通过。

仍待处理：独立 Ackermann path smoothing、将 semantics sidecar 接入更高层
SIM scoring schema、完整 mission score 的 per-objective 明细，以及旧段落中
尚未逐项重写的审计文字。

# 重构分支算法与仿真能力适配审计

审计对象：`refactor/clean-rebuild`
对照基线：本地 `main` 分支（仓库没有 `master` 分支）
审计日期：2026-10-24
工作区状态：审计期间未修改源码

## 1. 总体判断

新框架在工程边界上完成度较高：`WorldTruth` 与 ROBOT 类型隔离、`RobotApplication` 组合根、生命周期与 fail-closed 命令通道、MuJoCo 单引擎、地图版本、确定性回放、provider 注册等基础都成立，`tests/check_boundary.py` 也通过。

但“算法能力和仿真能力已经完整适配新框架”这个判断不成立。当前存在四类 P0 缺陷，分别破坏了几何契约、扫描角度契约、碰撞校验语义和 SLAM 静态地图假设；此外 headless 任务执行、传感器时序、倒车与多底盘规划、目标生命周期等主路径存在明显能力缺口。

`47/47` CTest 通过只说明已有测试覆盖的路径稳定。现有测试没有覆盖下面列出的关键契约，因此不能作为完整适配的背书。

## 2. 已验证的通过项

- `ctest --test-dir build --output-on-failure -j4`：47/47 通过。
- `tests/check_boundary.py`：通过，输出 `robot boundary ok: no sim includes, no truth tokens, no sim links`。
- `git status --short`：审计开始时为空。
- 真值隔离设计清楚：仿真真值只进入 `WorldTruth`、碰撞判定、评分与记录，不进入 ROBOT 输入。
- MuJoCo 单引擎、1 ms 子步、地形与材质、tracked 底盘、16 线 LiDAR、composite scan、terrain mission 均有实质实现。
- 地图 revision、确定性 replay、episode recorder、provider registry 具备继续扩展的基础。

## 3. P0：必须优先修复

### P0-1 配置几何与 ROBOT 控制器断链

`description_from_config()` 把 JSON 中的 `wheel_radius_m`、`track_width_m`、`wheel_base_m` 写入 `desc.plant`：

- `src/robot/platform/robot_config.cpp:486-513`

但 `make_chassis()` 的内置工厂仍硬编码几何：

- `src/robot/platform/chassis.cpp:326-373`
- differential：半径 `0.1`、轮距 `0.5`
- ackermann：半径 `0.1`、轴距 `0.62`、轮距 `0.58`
- tracked：半径 `0.1`、轮距 `0.6`
- swerve/mecanum：使用控制器默认几何

已复现：配置 `wheel_radius_m=0.23`、`track_width_m=0.91` 后

```text
desc=0.23,0.91 controller=0.1,0.5
```

也就是说 SIM 物理侧读取新几何，ROBOT 运动学与控制器继续使用旧几何，直接违反 `docs/algorithms.md:248-260` 中“同一硬件契约驱动两侧”的声明。

现有 `test_robot_config_native` 只验证 JSON 解析，没有验证 `description -> make_chassis` 的几何一致性。该项会让里程计、限速、转弯半径、轮速换算和控制器输出全部出现系统性偏差。

### P0-2 扫描角度字段未被所有算法遵守

`ObservationFrame` 定义了：

- `scan_angle_min_rad`
- `scan_angle_increment_rad`

但以下模块仍硬编码 `[-π, π)` 均匀布局：

- `src/robot/autonomy/perception/lidar_perception.cpp:78`
- `src/robot/autonomy/mapping/mapper.hpp:241`
- `src/robot/autonomy/validation/validator.hpp:253-254, 324`
- `src/robot/autonomy/safety/safety.cpp:42-44`

扫描匹配、定位和 SLAM 的部分路径使用了传入字段，形成同一帧数据在不同模块中被解释成不同几何的危险状态。

已复现：72 束、`scan_angle_min_rad=0`、`scan_angle_increment_rad=2π/72`，前 5 束放置 3 m 正前方目标：

```text
tracks=1 xy=-2.93197,-0.516986
```

目标本应在 `+x` 附近，却被解释到 `-x`。SIM 默认 `-π` 起始、360° 扫描掩盖了这个问题；任何局部扇区、非零起始角或非 360° LiDAR 都会触发。

### P0-3 Validator 声称 swept footprint，实际只检查离散中心点

`src/robot/autonomy/validation/validator.hpp:315-451` 的实现是：

- 把原始扫描转成 endpoint 点；
- 每个 `rollout_dt_s`（默认 0.1 s）计算一个机器人中心位置；
- 只检查中心到 endpoint 的欧氏距离是否小于 `footprint_radius_m`；
- 不检查机器人真实 footprint 形状；
- 不检查相邻采样位置之间的连续扫掠段。

因此高速运动时，细障碍物可能落在两个离散采样点之间而完全漏检。代码注释与 `docs/algorithms.md`、`docs/sim-plan.md` 中“swept footprint”的描述不一致。动态轨迹用线性预测，DWA 用 `predict_frames()`，两套时间与预测逻辑也没有统一。

这是安全相关缺陷，应按连续几何或保守包络实现，并补充高速细障碍物回归用例。

### P0-4 SLAM 子图污染动态 actor

`SlamSession::ingest()` 同时保存了：

```cpp
kf.ranges_m = obs.ranges_m;
kf.static_ranges_m = obs.static_ranges_m;
```

见 `src/robot/autonomy/mapping/slam.cpp:41-57`。

但 `submap_for()`（`src/robot/autonomy/mapping/slam.cpp:112-198`）完全使用 `kf.ranges_m` 做 free/occupied 栅格化，忽略 `static_ranges_m`，也没有屏蔽 foreground/dynamic returns。

结果是移动物体会被写入 keyframe submap，scan-to-submap tracking 可能跟踪 actor，loop closure 也可能把 actor 轨迹当成静态几何闭合。现有 `test_slam.cpp`、`test_loop_closure.cpp` 没有动态 actor 场景。

## 4. P1：主路径能力缺口与语义错误

### P1-1 Headless 入口没有接入任务与 actor

`src/sim/scenario/sim_main.cpp:12-76` 当前行为：

- 读取 robot/sim/world 文件；
- 只加载 world JSON；
- 不加载 `<map>.actors.json`；
- 有地图时只自动请求第一个 place；
- 固定运行 5 步；
- 最后只打印 `sim ran 5 steps`；
- 没有 duration/step CLI、objective list、score 输出、JSON artifact/replay 导出、mission/campaign 完成状态。

实际运行：

```text
$ ./build/lost_signal_sim --map configs/worlds/signal_town.json
robot config: configs/robot/differential.json
sim config: configs/sim/default.json
sim ran 5 steps
```

`configs/worlds/signal_town.actors.json` 中有 2 个 actor，但 headless 不读取。actor sidecar 解析仅存在于 `src/sim/viewer/viewer_main.cpp:76-147`。

旧 `main` 已有完整的 headless runner、run artifacts、mission simulation、性能评分和 JSON 输出。这是相对旧实现的能力明显退化，且 `docs/sim-plan.md:120-135` 已把 S5 headless multi-goal 标为 landed，文档与事实不符。

### P1-2 传感器时序与 provenance 被过度简化

`src/sim/scenario/scenario_session.cpp:85-141`：

- 世界固定 0.02 s 步长；
- 只有 `scheduler_.scan_due()` 时才轮询 suite；
- suite 一次性组合 motion/IMU/LiDAR 到单个 `ObservationFrame`；
- IMU、编码器实际只在 scan tick 被采样。

`src/sim/providers/sensor_suite.cpp:41-87` 同步采样 motion、IMU、2D/3D LiDAR，envelope 以 motion endpoint 的 `source/sequence/stamp` 作为单一 transport identity。

`docs/interfaces.md:13-15` 已承认只有一个 transport identity，section-level provenance 是 future work。`configs/sim/default.json` 和 `src/sim/eval/sim_config.hpp:22` 中的 `decision_s` 被解析但完全未使用，决策周期不可独立配置。

### P1-3 传感器延迟在 freshness 检查中被消解

`ScenarioSession::step_once()` 使用：

```cpp
robot::iface::MotionCommand cmd = app_.tick(obs.stamp_ns);
```

而不是当前仿真时间 `sim_stamp_ns`。delivery latency 保留了旧 `obs.stamp_ns`，同时又把旧时间作为 `now_ns` 传入：

```cpp
SafetyMonitor::evaluate(..., now_ns)
```

其中的

```cpp
if (now_ns - obs.stamp_ns > stale_timeout_ns)
```

看到的延迟恒为 0。真实传送延迟无法被识别为 stale。

`src/sim/robot/sim_robot.cpp:47-61` 更直接：`tick(now_ns)` 完全忽略入参，内部继续使用 `obs.stamp_ns`。这会让延迟和 dropout 注入在安全层失去意义。

### P1-4 DWA 速度窗口（已修复）

`DwaPlanner` 现在按测量速度生成对称的正/负动态窗口；RoutePoint 的显式
Ackermann gear 会继续约束局部采样方向，未标注 gear 的旧路线仍保留原有
前后向可达性。

### P1-5 Ackermann/Holonomic 规划适配不足（Ackermann 已补齐主路径）

Ackermann 现在由 planning factory 按声明的曲率包络接入
`AckermannPlanner`：全局采用带航向离散、前进/倒车 gear 和自行车模型
primitive 的 Hybrid-A*，局部 DWA 继续施加 `v·kappa` 曲率约束；Coordinator
只负责把模块串起来。无效运动学配置会 fail closed，不会退回不满足车辆
约束的通用 A* 路径；vector-only map 只产生待验证的直线 seed。

仍未完成的 Ackermann 增强项是独立 path smoothing（当前 primitive 路径由
局部 DWA 追踪）以及完整 RobotApplication -> provider -> MuJoCo mission
覆盖。Holonomic 方面：

旧 `main` 中存在：

- `src/robot/autonomy/planning/global/hybrid_astar.*`
- `src/robot/autonomy/planning/local/ackermann_local.*`
- `path_smoothing.*`

Holonomic 方面，`configs/robot/mecanum.json` 与 `swerve.json` 都是 `max_lateral_mps: 0.0`，因此默认不会采样横移。即使手工配置非零，DWA 也只有 `{-max_lateral, 0, +max_lateral}` 三个离散值，没有横向速度状态，窗口每个周期全开，缺少平滑与可达性约束。

`docs/algorithms.md:177-183` 宣称支持 lateral，但默认配置和端到端接入都不成立。测试只有控制器/工厂单元测试和手工 limits 的 DWA lateral 测试，没有 `RobotApplication -> provider -> MuJoCo -> mission` 的多底盘端到端覆盖。

### P1-6 目标生命周期存在 hold 与到达后继续运动缺陷

`RobotExecutive` 是单目标槽位（`src/robot/executive/executive.hpp:11-18`、`src/robot/executive/executive.cpp:34-85`），缺少任务排序执行、电量/返充/资源策略、多目标调度、失败恢复/重试和任务依赖。`TaskOrderingRegistry` 只存在于 `src/robot/autonomy/planning/task/task_ordering.*`，没有被 executive/coordinator 消费。

已复现 hold 语义错误：空 `place_id` 的请求直接进入 executive，executive 只写一个 Done hold，不清除 coordinator 当前 route：

```text
before=0.2,goal=active
hold_ack=done,next_cmd=0.2,old_goal=active
```

关键源码：

- `src/robot/executive/executive.cpp:39-45`
- `src/robot/app/robot_application.cpp:70-101`
- `src/robot/autonomy/coordinator.cpp:166-177`

结果是发送 hold 后机器人仍沿旧 route 行驶。

到达语义同样有缺陷。目标在 1 m 处完成时：

```text
done_at=62, cmd=0.497961, goal=done
next_cmd=0.497961, goal=done
```

`RobotApplication::tick()` 调用 `executive_.note_arrival(coordinator_.route_complete())`，但 `note_arrival()` 只把 executive 状态置 Done，不调用 `coordinator_.clear_goal()`。coordinator 继续持有 `has_goal_` 和 `active_route_`，因此 goal 已 Done 仍可能继续输出运动。

## 5. P2：能力削弱、语义丢失与文档问题

### P2-1 行为策略过度简化

`src/robot/autonomy/behavior/behavior.cpp:10-23` 的 `WaitOnIntrusionBehavior`：

- 任意 track 距机器人小于 2 m 就整体停车；
- 不按前进方向或冲突区判断；
- 不做通行权、等待/恢复、时间占用；
- 不使用预测速度，只看当前 track 位置。

`behavior.hpp:30-32` 明确承认要等 predictor。

旧 `main` 有 crossing behavior、entry governor、conflict zone semantics、high-level policy、dynamic occupancy/prediction alignment；当前实现明显退化。

### P2-2 感知对象几何过度简化

`src/robot/autonomy/perception/lidar_perception.cpp`：

- 聚类只输出质心；
- `DynamicTrack::radius_m` 永远使用默认 `0.45`；
- 不根据点簇尺寸估计对象半径；
- 速度是首次观测到当前的平均速度，急转/加减速响应滞后。

见 `src/robot/autonomy/perception/perception.hpp:19-40`、`src/robot/autonomy/perception/lidar_perception.cpp:159-213`。

### P2-3 地图转换有损

`tools/convert_master_map.py`：

- 普通 obstacles 无高度时统一默认 `1.2 m`；
- 旋转 world object 转保守 AABB；
- 非 box collision 丢弃；
- conflict zones 丢弃；
- `dormant_until_summoned` actor 跳过；
- actor 写入 sidecar，headless 不读取。

见脚本注释与 `tools/convert_master_map.py:13-29, 183-205, 255-284`。

实际统计：

```text
signal_town:
 master_obstacles=4
 master_world_objects=23
 master_collisions=23
 master_conflict_zones=2
 master_actors=2
 new_boxes=27
 new_actors=2
```

box 数虽保留，但 conflict zones 等语义丢失，sidecar 仅 viewer 使用。另外 `ScenarioSession::fit_map_window_to_world()`（`src/sim/scenario/scenario_session.cpp:143-168`）没有把 support slab 范围纳入，可能裁掉桥面或支撑面地图窗口。

### P2-4 评分与运行 artifact 不完整

`src/sim/eval/scoring.cpp` 只统计距离、safe stop、观测接受/拒绝、碰撞步数、估计误差。`reward_step()` 只有 truth x 方向位移减停步/丢观测惩罚。

缺少任务完成、时间、能耗、路径质量、最小 clearance、动态避让质量等完整 mission score。`EpisodeRecorder` 只在内存保存，`ScenarioSession` 没有 headless artifact export CLI。旧 `main` 有 run artifacts、mission performance score 和 JSON 输出。

### P2-5 文档与事实不一致

- `docs/sim-plan.md:3` 仍写 `Robot side is done (clean rebuild, 24/24 green, boundary gate green).`，实际为 47 个 CTest。
- `docs/sim-plan.md:120-135` 称 S5 headless multi-goal 已落地，但 `sim_main` 固定 5 步且不加载 actor sidecar。
- `docs/sim-plan.md:145-151` 称 latency/dropout/IMU realism 已完成，实现存在，但 freshness 时钟逻辑错误。
- `docs/algorithms.md:248-260` 宣称同一 `DriveProfile` 几何驱动 `make_chassis` 和 SIM，已被复现推翻。
- `docs/algorithms.md:177-183` 宣称 holonomic lateral 与通用规划已适配，但默认 lateral=0，且没有专用端到端路径。
- `docs/algorithms.md:217-239` 描述 SLAM submap / loop closure 能力，没有说明动态 actor 过滤缺失。
- `README.md` 仅 46 行，缺少运行方法、CLI 能力、配置边界、能力矩阵、已知限制和 headless 输出说明。旧 `main` README 的 build/headless/mission 文档更完整。

## 6. 能力矩阵

| 能力 | 当前实现 | 端到端接入 | 相对 main | 优先级 |
|---|---|---|---|---|
| 差速底盘 | 控制器、MuJoCo、HIL 均有 | 是，默认主路径 | 几何契约断链，部分保持 | P0 |
| Ackermann | Hybrid-A*、曲率约束 DWA、低层控制与 plant | 主路径已接入，缺 mission E2E | path smoothing/任务覆盖待补 | P1 |
| Swerve/Mecanum | 有控制器与 plant | 无 mission E2E，默认 lateral=0 | 部分适配 | P1 |
| Tracked | plant 与注册存在 | 主要在 terrain 测试 | 基础能力保留 | P2 |
| 动态避障 | track + DWA 预测 | 有最小链路 | 行为策略退化 | P1/P2 |
| SLAM | pose graph、scan match | 静态场景测试 | 动态 actor 污染 | P0 |
| Headless mission | scenario/objective 类存在 | `sim_main` 未接入 | 大幅退化 | P1 |
| 多速率传感器 | scheduler/provider 类存在 | decision 配置 inert，sensor bundled | 未完成 | P1 |
| 地图转换 | world/sidecar 可生成 | headless actor 缺失 | 语义丢失 | P2 |

## 7. 建议整改顺序

### 第一阶段：契约与安全

1. 统一几何来源：`description_from_config -> make_chassis` 全链路读取 `desc.plant`，删除硬编码；补各底盘几何一致性测试。
2. 统一扫描角度：perception、mapper、validator、safety 全部使用 `scan_angle_min_rad` 与 `scan_angle_increment_rad`；补局部扇区、非零起始角、非 360° 用例。
3. Validator 实现真正的连续 swept footprint：footprint 形状、相邻位姿之间的扫掠段、统一预测时间基准。
4. SLAM submap 使用 `static_ranges_m`，屏蔽 foreground/dynamic returns；补动态 actor SLAM 与 loop closure 测试。

### 第二阶段：主路径恢复

5. 修正仿真时钟：`tick(sim_stamp_ns)` 作为决策时刻，观测 stamp 保持采集时间，使 freshness 能识别 delivery latency。
6. 恢复 headless runner：duration/step、objective list、actor sidecar、score、JSON artifact/replay 导出、mission 完成状态。
7. 修正 hold 与 arrival 生命周期：hold 清除 route；到达后清除 goal 并输出零速；补状态机测试。
8. 分离传感器采样周期：IMU、encoder、LiDAR 独立 provider 时序；落实 `decision_s` 或删除该配置。

### 第三阶段：规划能力

9. DWA 负速度窗口与未标注路线的倒车能力已落地；Ackermann 显式 gear
 已接入，后续补狭窄区、靠墙脱困和任务级回归。
10. Ackermann Hybrid A*、曲率约束局部采样、前进/倒车 gear 已落地；后续补独立
    path smoothing 与多底盘 mission E2E。
11. Holonomic 增加横向速度状态与平滑采样，修正默认配置，补 mecanum/swerve 的 mission E2E。

### 第四阶段：行为、感知与交付质量

12. 行为层补冲突区、通行权、等待/恢复和速度预测。
13. 感知层按点簇尺寸估计对象半径，使用近期速度而非全程平均。
14. 地图转换保留 conflict zones、旋转几何、非 box collision 与 actor 语义；修正 support slab 窗口。
15. 扩展 mission score 与 artifact；更新 README、`sim-plan.md`、`algorithms.md`。

## 8. 必补回归测试

- 配置几何：每种 chassis 的 `description -> make_chassis` 半径/轮距/轴距一致。
- 扫描角度：非零起始角、局部扇区、非 360°、不同 beam 数。
- Validator：高速细障碍物、footprint 形状、离散采样间隙。
- SLAM：动态 actor 存在时的 submap、tracking、loop closure。
- 延迟链路：delivery latency 触发 stale 与 safe stop。
- 目标生命周期：hold、cancel、arrival、preempt 后命令归零。
- Headless：actor 加载、objective 执行、score 与 artifact 输出。
- 多底盘：`RobotApplication -> provider -> MuJoCo -> mission` 的 differential/ackermann/mecanum/swerve/tracked 端到端。
