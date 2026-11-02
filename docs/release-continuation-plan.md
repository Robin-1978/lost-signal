# 发布版大目标（精简入口）

最后更新：2026-11-03
分支：`refactor/clean-rebuild`
历史恢复日志：[release-continuation-archive-2026-11-02.md](release-continuation-archive-2026-11-02.md)

这份文件只保留当前决策、发布证据和未完成事项。历史失败实验、逐次诊断和旧阶段记录放在归档文件中，不再作为当前待办读取。

## 产品定位

Robot Simu 是一个用真实机器人软件传播机器人知识的产品：同一套 `src/robot` 代码同时服务于玩家体验、机器人教学和工程验证。SIM 负责物理、场景和离线真值；ROBOT 只接收带时间戳的传感器、地图、任务意图和执行反馈。三种模式不能偷偷使用不同的导航算法。

| 模式 | 主要用户 | 核心价值 |
|---|---|---|
| Player | 初学者、公众 | 通过任务理解“传感器 → 估计 → 规划 → 安全” |
| Learning | 学生、教师、机器人爱好者 | 对照 ROBOT 认知和 SIM 真值，做可复现 A/B 实验 |
| Engineering | 算法、硬件和教学实验室 | 跑固定步长 benchmark、回放、性能和安全回归 |

当前产品能力是 **2D LiDAR + 轮速/IMU + surveyed-map 定位与导航**。它不是完整 3D LiDAR-Inertial SLAM：没有完整 IMU bias/preintegration、3D 点云紧耦合估计和逐点去畸变。

## 当前发布结论

软件侧已经达到 Release Candidate：Debug 和 Release 的 97 个 CTest 全部通过，工业五目标、Ackermann、signal_town、四底盘、动态场景、terrain、SLAM/回环、Safety、Viewer、truth-isolation、VirtualMcu/native HIL 软件契约均有最终 HEAD 证据。

First Contact 的软件交付切片、双语课件、截图、A/B 报告格式和可分发 Release 包已经完成；目标用户试用记录表见 `docs/first-contact/user-trial-form.md`。真实板卡仍是 1.0 实机门禁，不能用仿真结果替代：具体 MCU HIL、急停与制动距离、满载/低抓地、通信延迟、实际 LiDAR 量程、标定 artifact 和现场长时间回放均保持“待验”。

## 发布证据

| 范围 | 结果 | 证据 |
|---|---:|---|
| Release 非 long | 94/94 | `ctest --test-dir build-release -LE long`，最新重跑 285.59 s |
| Debug 非 long | 94/94 | `ctest --test-dir build -LE long`，642.55 s |
| Release long | 3/3 | 工业五目标 849.01 s；direct 208.41 s；`signal_town` 27.15 s |
| Debug long | 3/3 | 工业五目标 1148.11 s；direct 273.92 s；`signal_town` 47.18 s |
| 工业五目标 | 5/5，0 碰撞 | Release/Debug 长测，覆盖 4→5 恢复段 |
| Ackermann 路线净空 | A* 0.177 m；Hybrid-A* 0.235 m | `clearance_cost_per_m=6.0`，硬包络不变 |
| Manual authority | 过期立即安全停车 | `goal_auth` 使用 1 s decision period 回归 |

本轮产品冲刺基线：`c507508`。主要功能提交：`b57b3d0`（Ackermann 净空）、`19d2da3`（Manual watchdog）；完整历史见归档文件。

## 软件架构和安全契约

- `RobotExecutive` 拥有 Goal 生命周期；Hold 会取消旧 Goal，Manual 接管后 watchdog 只能停车，不能自动恢复旧任务。
- `ObservationPipeline` 拥有传感器同步、估计、感知、地图和定位更新；过期估计不能继续当作有效定位。
- `DecisionCyclePipeline`/`MotionDecisionPipeline` 编排全局路线、局部候选、Behavior、Validator 和 Controller。
- `DWA`、`TEB`、Ackermann Hybrid-A* 共享候选/Validator/Safety 契约，底盘差异通过 `ChassisLimits` 和 factory 投影；不复制三套碰撞算法，也不把差速原地转向泄漏给 Ackermann。
- `SafetyMonitor` 使用新鲜 raw scan，独立于 costmap 和 planner；Validator 负责候选轨迹，Safety 是最终降级门。
- footprint 按底盘声明的半径解释；硬 clearance、软净空代价和 Safety 包络分别管理，不能用直径重复膨胀。
- SIM truth、actor identity、未来脚本、碰撞真值和 score 不进入 ROBOT 决策路径。

行业对照资料：

- [Nav2 Collision Monitor](https://github.com/ros-navigation/navigation2/blob/main/nav2_collision_monitor/README.md)
- [Nav2 Smac Hybrid-A*/State Lattice](https://github.com/ros-navigation/navigation2/blob/main/nav2_smac_planner/README.md)
- [Nav2 Regulated Pure Pursuit](https://github.com/ros-navigation/navigation2/blob/main/nav2_regulated_pure_pursuit_controller/README.md)
- [TEB Local Planner](https://github.com/rst-tu-dortmund/teb_local_planner/blob/master/README.md)

## 产品判断

这个产品方向是合理的，而且有清晰差异：它不是把机器人做成“会自己跑的游戏”，而是把真实的机器人估计、规划、控制和安全决策变成可观察、可回放、可比较的学习对象。Player、Learning、Engineering 三层共用同一个 ROBOT 接口，能同时服务传播知识、课程实验和工程验证，这是可信的产品基础。

需要守住三个产品边界：

1. **先讲因果，再堆功能。** 第一体验应让用户看懂一次“测量 → 估计 → 规划 → Validator → Safety”的完整链路；不要一开始展示过多参数和真值层。
2. **明确能力范围。** 对外称为 2D 移动机器人导航与仿真教学/验证平台，不宣传成完整 3D LIO 或已通过真实机器人安全认证。
3. **教学和工程结果分层。** 教学解释可以展示 SIM 真值，但工程结论必须只来自 ROBOT 证据、固定回放和独立安全门。

因此，当前适合发布为软件仿真 Release Candidate、课程试用版和受控工程验证版；完成真实板卡门禁后，才适合称为完整实机 1.0。

产品 MVP 应优先服务“想理解机器人如何做决定的人”：一条 10 分钟 First Contact 课程、一个可复现的变量实验、一次带证据链的复盘和一张知识卡，先把因果讲清楚，再扩展工业、SLAM、底盘和硬件课程。Player、Learning、Engineering 可以共存，但对外传播时应以 Learning 为主入口；工程 benchmark 是可信度支撑，不应成为第一次体验的界面。

当前吸引力判断：对机器人学生、教师和工程师已经较强；对普通公众还需要一个一眼可懂、愿意完成并分享的主体验。现有 First Contact 已具备双语台词、Hero intro、3D hotspot、运行前预测、固定变量的 LiDAR A/B、关键帧因果复盘和知识卡，因此下一阶段优先把这条纵向链路做成产品，不再先扩场景数量。

本轮已把首屏钩子落到可分发构建：第一帧立即显示真实 MuJoCo 画面，首镜头直接提出 manifest 中的预测问题，并用“测量 → 估计 → 停车”标出体验路径；First Contact 显式跳过可选部件检查后直接进入预测，Release 包同时携带首屏证据图和可复现截图命令。代码提交为 `0cdc090`，入口提交为 `b287a8e`，交付资产提交为 `3e1e36b`，包装门禁提交为 `a400fb4`。

## 下一冲刺：First Contact 可传播闭环（P0）

冲刺目标：第一次接触产品的用户无需阅读开发者文档，在 10 分钟内完成一次任务、做出一次预测、更换一个变量、复跑对比，并说清“机器人为什么这样停下或绕行”。所有内容继续使用同一个 ROBOT 运行时；教学层只能增加解释，不能改变判断。

| 工作包 | 交付结果 | 完成标准 |
|---|---|---|
| 故事与入口 | 默认进入双语 First Contact；开场只提出一个问题和一条任务 | 冷启动后一次操作进入体验；可跳过 intro；暂停、极速、返回和键盘操作均有明确提示 |
| 因果可观察性 | Player/Learning/Engineering 三层逐步呈现观测、估计、规划候选、Validator、Safety 和最终命令 | 运行层只显示 ROBOT 证据；SIM 真值仅在明确复盘层出现；证据缺失时显示“未记录”，不得补造 |
| 3D 呈现 | 稳定跟随镜头、有效 hotspot、RO-1 Hero Asset、可读的 LiDAR/轨迹/状态标记 | 跟随镜头无明显抖动；关键部件可点击；1080p 下无需离屏即可识别层级；截图能表达前向与安全状态 |
| 受控实验 | 保留 First Contact 的 LiDAR 周期 A/B，并明确锁定世界、机器人、种子和其他参数 | 基线/变体使用同一 runtime factory；仅声明变量不同；失败、超时和证据不足均如实展示 |
| 复盘与学习 | 关键帧时间线、基线差异、预测判定、知识卡和简短学习结果 | 复盘引用真实 episode step；ROBOT 结论与 SIM 验收分开；安全非等效结果不限制差异方向；用户完成后能回答一个因果问题 |
| 可交付安装包 | 首个章节课件、运行命令、截图、证据报告格式和中英文内容清单 | 新机器可复现启动；内容与代码版本可追溯；报告包含 invariant digest、代码版本、关键指标和 verdict |

首个纵向切片的验收不只看“能运行”，还看用户是否完成理解闭环。课程试用先以 10–20 名目标用户记录完成时长、预测选择、变体完成率、复盘完成率和前后知识题；记录表见 [First Contact user-trial form](first-contact/user-trial-form.md)。这些数据用于验证教学效果，不用星级或游玩时长代替。

## 后续优先级

1. **P1 证据包与分发：** 将 First Contact 的实验报告、关键帧截图、知识卡和 benchmark 命令导出为可分享包；随后处理安装、版本升级和离线运行。
2. **P2 内容扩展：** P0 达到体验和证据标准后，再以“过期地图：实时安全”作为第二课，逐步扩展到工业、Ackermann、SLAM 和动态障碍。
3. **P3 实机 1.0：** 完成硬件门禁和现场长回放；对外发布 1.0 前不得用仿真结果替代。

## 仍然有效的待办

1. 执行上述 First Contact 目标用户试用，按记录表收集 10–20 名参与者的完成时长、预测选择、变体完成率、复盘完成率和前后知识题；未达标的项先修复，不新增 Campaign 场景。
2. 在目标硬件上完成 MCU/Native HIL、急停、制动、负载、通信延迟、LiDAR profile 和 calibration artifact 验收，并保存可追溯记录；记录格式和 fail-closed 校验见 [hardware release gate](hardware-release-gate.md)。
3. First Contact 可交付安装包已经生成；目标用户试用通过后，把首个章节、回放报告和 benchmark 命令归档为 P1 证据包。
4. 实板证据完成后重新运行同一套 97 项软件门禁和现场回放，不修改 Safety、footprint 或 GoalArrival 来换取通过。

## 常用验证命令

```sh
cmake --build build -j6
cmake --build build-release -j6
ctest --test-dir build -LE long --output-on-failure
ctest --test-dir build -L long --output-on-failure
ctest --test-dir build-release -LE long --output-on-failure
ctest --test-dir build-release -L long --output-on-failure
```

Viewer/资产工具路径：`/disk/blender-5.2.2-linux-x64/blender`。

## 3D Hero 资产更新

RO-1 的 chassis 与标准 LiDAR 已按产品传播场景重设计，增加前向识别、状态灯带、服务面板和分层传感器外壳，服务 Player/Learning 的 3D 跟随视角。资产仍使用米制 `+X` 前向、`+Y` 左、`+Z` 上坐标；MuJoCo 继续作为 link、joint、碰撞、footprint 和姿态的唯一权威，视觉重设计不进入导航或物理参数。

`tools/modeling/inspect_assets.py` 对全量 53 个模型通过；RO-1 chassis 为 564 triangles、标准 LiDAR 为 188 triangles，manifest bounds、link binding、材质/三角预算和 content hash 均通过。Release 定向回归 `mujoco_backend|sim_actor|viewer_frame|asset_manifest|scene_manifest|experience_manifest|experience_runtime|experience_catalog` 为 8/8。

工业地图：

```sh
./build-release/sim_viewer --sandbox --3d \
  --map configs/worlds/industrial_200k.json --quality fast
```
