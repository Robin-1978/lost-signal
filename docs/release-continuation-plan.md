# 发布版持续推进清单

本文件是 `refactor/clean-rebuild` 分支的跨会话恢复点。上下文压缩或会话恢复后，先读取本文件、`git status` 和最近提交，再从第一个未完成阶段继续。不得因为恢复上下文而重新设计已经确定的方向。

## 不变约束

- 保留当前新框架；Coordinator 只负责串联，算法逻辑下沉到对应模块。
- SIM 真值、actor identity、未来脚本、碰撞真值和 score 不得进入 ROBOT。ROBOT 只使用传感器、关节反馈、任务意图和自身估计。
- 地形与材质真值只属于 SIM；ROBOT 只有在部署地图明确提供相应先验时才能读取地图层。
- 渲染姿态来自真实 MuJoCo body/joint/contact 状态。轮、caster、叉车、行人和未来机械臂不得使用墙钟动画或按车速重新积分姿态。
- 算法不可为关卡特化；不通过放宽到达条件、移动目标、读取真值或叠加独立魔法阈值掩盖问题。
- 安全优先，其次是连续和平滑。模块测试负责算法能力，场景测试负责系统验收。
- 每个完整阶段独立提交；提交前运行该阶段相关测试，最终再跑全量测试。

## 执行阶段

1. **无图 SLAM 回环可信度**
   - 完成跨历史候选统一比较，不能按距离遇到第一个可用候选就接受。
   - 时序确认必须锁定同一历史子图 ID。
   - 恢复并验收 2.5 m 方形路线；首次回环必须发生在完成 east、north、west 三个目标后的真实重访阶段。
   - SIM 真值只用于测试失败诊断，不能进入 ROBOT 回环判断。
   - 不新增独立魔法阈值；优先使用已有 matcher/gate 的有单位配置与相对证据。

2. **SLAM 教学体验与文档**
   - 更新 SLAM Toolbox、REP-105 和 map/odom 权限说明。
   - 修正文档中“无图 tracking 不降低绝对 covariance”的过期描述：每个 causal submap anchor 最多融合一次，并使用 scan matcher 的方向性 information；不可观方向不压低 covariance。
   - 提交 2.5 m `slam_loop` 教学体验及其中英文内容。

3. **算法与系统回归**
   - 运行定位、回环、`signal_town`、四底盘、Experience 及全量测试。
   - 清理重复代码和临时诊断，保留可复现失败所需的结构化证据。

4. **Release 性能与一致性**
   - 构建 Release。
   - 实测 1x、8x、64x、Turbo 和 headless。
   - 记录 RTF、steps/s、ROBOT tick p99/最大值，并验证 observation、command、collision、终态和 score 一致。

5. **SIM 游戏体验**
   - 缩小首屏说明，使第一次主动决定在十秒内发生。
   - 完成 `EpisodePair`、独立 ROBOT/SIM safety/Mission/Experiment verdict。
   - 完成玩家、教学、工程三层视图和基于不可变 Episode 证据的复盘。

6. **物理证据可视化与效果**
   - 完成 LiDAR 时序扫描、Safety polygon、local costmap/inflation、planner swept volume 和定位修正 ghost。
   - 粒子与声音只由真实 contact、slip、normal load、service brake 和 impact 驱动，并随 sim time 确定性复现。

7. **模型与场景升级**
   - 机器人、叉车、行人和机械臂全部由 MuJoCo body/joint pose 驱动。
   - 增加 joint axis、collision geometry、COM、contact overlay。
   - 按资产 manifest 的单位、轴、pivot、bounds 和 link binding 扩展 Blender/GLB 环境资产。

## 当前恢复点

- 已完成基线：`a66d644 fix: reject ambiguous mapless loop closures`。
- `signal_town` 最近基线：5 个目标、0 碰撞、最大定位误差 0.278 m、ROBOT tick p99 6.624 ms、最大 tick 14.306 ms、普通构建 RTF 6.13x。
- 当前阶段：第 4 项。
- 第 1 项已经完成：候选预算先覆盖不同历史模式，入选候选全部完成 coarse/refine/reverse 验证后统一比较双向 score 与 overlap；相同 correction mode 内只有同一个历史锚点 ID 才能完成跨时序确认。
- `test_loop_closure` 首次回环发生在完成 east、north、west 三个目标后；SIM 真值距起点 0.305 m 只写入失败诊断。`slam` 与 `loop_closure` 均连续通过 3 次，`truth_isolation` 与 `robot_boundary` 通过。
- 第 2 项已经完成：2.5 m 教学路线完成 4/4 目标且零碰撞；终端捕获按底盘运动能力修复，没有放宽到达阈值或延长任务时间。`slam_experience_mission` 连续重复三次通过，相关 14 项定位、SLAM、Experience、truth-isolation 和 boundary 回归通过。

### 工作区检查点（2026-10-26）

- 第 2 阶段提交：`4711d91 feat: add mapless SLAM teaching mission`。
- `slam_loop` 中英文 Experience、无先验地图 world/scene 和端到端 mission test 已进入提交；Experience manifest/runtime 显式传递 `slam_only`，避免由关卡外部偷偷切换 ROBOT 模式。
- 回环使用 matcher 输出的保守位置/航向不确定度，不再固定为 `0.05 m / 0.02 rad`。mapless tracking 每个 causal submap anchor 最多融合一次 3x3 information；surveyed-map tracking 保持相对证据权限。
- 最终目标附近的差速终端捕获已经按“可原地转向、不可横移”的底盘能力修复；Mecanum/Swerve 不会被错误强制先转向。到达判定仍由统一 `GoalArrivalPolicy` 的置信包络和稳定停车条件负责。
- 物理审查发现的 post-step velocity projection 方案已判定会破坏 MuJoCo 法向接触求解；该实验不在当前 diff 中。后续轮胎/roller 真实性只能用 solver 内广义力或显式 roller bodies 实现。

### 工作区检查点（2026-10-27）

- 当前 HEAD 是 `06b68aa docs: advance release checklist to regression`；分支相对远端领先 3 个提交。
- 第 3 阶段首次全量回归为 73/75：`terrain_mission` 在冰面坡道终端永久停车/绕圈，`industrial_200k_mission` 在第三目标附近因一次 `global-plan-failed` 永久停车；其余定位、回环、`signal_town`、四底盘、DWA/TEB 和 Experience 测试通过。
- 工作区有 9 个属于第 3 阶段的未提交修改：`ackermann_planner.cpp`、`astar_planner.cpp`、`dwa_planner.cpp/.hpp`、`global_planner.hpp`、`route_pipeline.cpp`、`test_astar.cpp`、`test_dwa.cpp`、`test_terrain_mission.cpp`。恢复时不得丢弃这些修改。
- 坡面终端修复使用现有位置/航向置信包络和底盘运动能力：DWA 进入位置包络后锁存终端航向阶段，位置漂出时重新取得位置而不在目标方位和最终航向之间逐 tick 切换；横向残差占主导时先对准再平移。`terrain_mission` 使用 `SeedAndTrack + surveyed static map + operator prior`，没有读取真值；原始 2500 步预算已通过。
- 路线恢复改为原子替换：同目标局部重规划失败时保留上一条已验证路线，所有命令仍逐次经过局部规划、Validator 和 Safety；新目标、地图变化或离开路线管道会立即失效旧路线。失败请求只在 ROBOT 估计跨栅格、目标或地图变化时重试，避免规划风暴。
- A*/Hybrid-A* 的 `GlobalPlanProgress` 已增加结构化 `diagnostic`；新增单元测试覆盖旧路线保留、失败锁存和跨栅格重试。相关 `astar`、`industrial_200k`、`dwa`、`terrain_mission` 模块测试已通过。
- 完整 20 万平方米五目标复测结果：62169 步、5/5、0 碰撞、464.99 m、最大定位误差 2.583 m；五个目标 truth miss 依次为 0.083/0.090/0.101/0.091/0.132 m，最终状态 `goal_hold/arrived/stop`。这些真值仅用于 SIM 验收和失败诊断。

### 阶段 3 最新检查点（2026-10-27）

- 阶段 3 第一次全量测试更新为 74/75；20 万平方米、`signal_town`、四底盘、terrain、DWA/TEB 等均通过，唯一失败为 `slam_experience_mission`。
- 第二个教学目标的失败已证明是 ROBOT 局部 scan tracking 错误，而非到达阈值：失败时 ROBOT 估计距目标 0.124 m、估计标准差 0.029 m，SIM 真值距目标 0.261 m；第 12 次局部跟踪单帧接受了 0.141 m 横向修正，而修正前后的实际估计误差约为 0.038 m 和 0.162 m。SIM 真值只用于测试诊断。
- 无图局部 tracking 现要求大于既有 matcher 栅格步长的 SE(2) 修正由同一 causal submap anchor、独立运动视角和第二次一致刚体修正共同确认。实现复用 `track_step_m`、`track_heading_step_rad` 与既有 keyframe 运动门限，没有增加关卡阈值；`test_slam` 已覆盖单视角大修正保持 pending。
- 局部 tracking 修复后前三个教学目标真实通过，同时暴露旧回环验证漏洞：旧逻辑在真实重访时接受约 0.045 m / 0.164 rad 的错误修正；回环前原始 odom 航向误差约 0.01 rad，回环后实际误差约 0.18 rad。endpoint occupancy 在厚墙带上无法区分该偏航。
- 历史候选现在使用完整 centered historical submap，reverse verification 使用与历史窗口分离的 recent causal submap；`ScanMatcher` 已增加 expected-range 评分，增量 loop lattice 可在 endpoint shortlist 后按距离一致性重排，dense refine 还要求相对 odom prior 有足够 likelihood improvement。
- coarse 与 refine 同时使用 expected-range 时，`slam_experience_mission` 已真实完成 4/4、零碰撞，但验证过于保守，没有产生 closure，导致 `loop_closure` 的 `max_loops == 0` 失败。
- 最后一个尚未验证的修改把 loop coarse 恢复为 endpoint occupancy，只在 dense refine 启用 expected-range：coarse 负责进入候选 basin，dense refine 负责精确距离一致性。不得降低 `minimum_score_margin`，也不得允许 ambiguous scan 直接形成闭环约束。
- 当前工作区预计有 16 个修改文件：阶段 3 的规划/测试修改、本文档，以及 `slam.cpp/.hpp`、`scan_matcher.cpp/.hpp`、`test_slam.cpp`、`test_slam_experience_mission.cpp`。这些都属于尚未提交的阶段 3 工作，不得丢弃。

### 阶段 3 完成检查点（2026-10-27）

- coarse endpoint retrieval + dense expected-range refinement 已验证：局部 tracking、`localization`、真实重访回环和 2.5 m `slam_experience_mission` 四项全部通过。完整历史子图与不相交 recent causal submap 做双向距离一致性验证，跨历史候选统一比较；无图大修正仍要求同一 causal anchor 的独立视角确认。
- `terrain_mission` 使用 surveyed static map + operator prior 隔离地形/牵引能力；终端航向锁存按统一置信包络恢复位置，不为坡道增加阈值。
- 同目标全局重规划改为原子替换：失败时保留上一条已验证路线，新目标、地图修订或离开路线管道时立即失效；所有输出继续通过局部规划、Validator 和 Safety。
- 完整构建通过；定向回归 14/14；全量 CTest 75/75。20 万平方米、`signal_town`、四底盘、terrain、A*/Hybrid-A*、DWA、TEB、truth isolation、ROBOT boundary、SLAM 与 Experience 均通过。
- `git diff --check` 通过；ROBOT 改动未引用 SIM 类型或真值。新增决策尺度复用 matcher 栅格、关键帧间距、已有 score margin、置信包络与底盘能力，没有增加关卡专用常量。

恢复后按以下顺序继续：

1. 读取本文件，执行 `git status --short --branch` 和 `git log -5 --oneline`，确认阶段 3 已提交且工作区没有遗留的阶段 3 修改。
2. 配置并完整构建 `build-release`，保留固定物理步长；倍率只改变墙钟调度，Turbo/headless 以最大吞吐运行。
3. 为同一个可复现实验实测 1x、8x、64x、Turbo 和 headless，记录 RTF、steps/s、ROBOT tick p99/最大值及墙钟耗时。
4. 比较 observation、command、collision、终态、objective verdict 和 score；任何倍率差异先定位时间源、调度或未使用 sim time 的模块，不通过放宽比较掩盖。
5. 阶段 4 验收后更新本表并单独提交；随后不等待提醒，依次完成第 5 至第 7 阶段。

## 阶段完成记录

每完成一项，在这里记录提交、测试、性能数据和剩余风险，再进入下一项。

| 阶段 | 状态 | 提交 | 验收证据 |
|---|---|---|---|
| 1. 无图 SLAM 回环可信度 | 已完成 | `a66d644` | 首次回环前已完成 3 个目标；SLAM/回环重复回归与 truth isolation 通过 |
| 2. SLAM 教学体验与文档 | 已完成 | `4711d91` | 4/4 目标、零碰撞；任务连续三次通过；14 项相关回归通过；文档明确 SLAM Toolbox/REP-105 能力边界 |
| 3. 算法与系统回归 | 已完成，待提交 | - | 完整构建；定向 14/14；全量 75/75；20 万平方米、signal_town、四底盘、SLAM/回环、truth isolation 均通过 |
| 4. Release 性能与一致性 | 待开始 | - | 待完成 |
| 5. SIM 游戏体验 | 待开始 | - | 待完成 |
| 6. 物理证据可视化与效果 | 待开始 | - | 待完成 |
| 7. 模型与场景升级 | 待开始 | - | 待完成 |
