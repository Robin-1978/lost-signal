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
- 当前阶段：第 3 项。
- 第 1 项已经完成：候选预算先覆盖不同历史模式，入选候选全部完成 coarse/refine/reverse 验证后统一比较双向 score 与 overlap；相同 correction mode 内只有同一个历史锚点 ID 才能完成跨时序确认。
- `test_loop_closure` 首次回环发生在完成 east、north、west 三个目标后；SIM 真值距起点 0.305 m 只写入失败诊断。`slam` 与 `loop_closure` 均连续通过 3 次，`truth_isolation` 与 `robot_boundary` 通过。
- 第 2 项已经完成：2.5 m 教学路线完成 4/4 目标且零碰撞；终端捕获按底盘运动能力修复，没有放宽到达阈值或延长任务时间。`slam_experience_mission` 连续重复三次通过，相关 14 项定位、SLAM、Experience、truth-isolation 和 boundary 回归通过。

### 工作区检查点（2026-10-26）

- 第 2 阶段提交：`4711d91 feat: add mapless SLAM teaching mission`。
- `slam_loop` 中英文 Experience、无先验地图 world/scene 和端到端 mission test 已进入提交；Experience manifest/runtime 显式传递 `slam_only`，避免由关卡外部偷偷切换 ROBOT 模式。
- 回环使用 matcher 输出的保守位置/航向不确定度，不再固定为 `0.05 m / 0.02 rad`。mapless tracking 每个 causal submap anchor 最多融合一次 3x3 information；surveyed-map tracking 保持相对证据权限。
- 最终目标附近的差速终端捕获已经按“可原地转向、不可横移”的底盘能力修复；Mecanum/Swerve 不会被错误强制先转向。到达判定仍由统一 `GoalArrivalPolicy` 的置信包络和稳定停车条件负责。
- 物理审查发现的 post-step velocity projection 方案已判定会破坏 MuJoCo 法向接触求解；该实验不在当前 diff 中。后续轮胎/roller 真实性只能用 solver 内广义力或显式 roller bodies 实现。

恢复后按以下顺序继续：

1. 读取本文件，执行 `git status --short --branch` 和 `git log -5 --oneline`，确认 HEAD 与上述检查点一致。
2. 从干净工作区开始第 3 阶段，先构建当前 Debug 基线并列出全部 CTest；按定位/回环、`signal_town`、四底盘、Experience、truth boundary 的顺序运行，最后跑全量测试。
3. 对任何失败先判断是算法、物理、场景还是测试契约问题；测试场景真实失败就保留失败，不得读取 SIM 真值、放宽验收或加入关卡特化。
4. 清理重复代码和临时诊断，只保留可复现失败所需的结构化证据；阶段通过后单独提交并记录完整测试数量、耗时和剩余风险。
5. 立即进入第 4 阶段的 Release 倍率与一致性实测，不等待新的会话提醒。

## 阶段完成记录

每完成一项，在这里记录提交、测试、性能数据和剩余风险，再进入下一项。

| 阶段 | 状态 | 提交 | 验收证据 |
|---|---|---|---|
| 1. 无图 SLAM 回环可信度 | 已完成 | `a66d644` | 首次回环前已完成 3 个目标；SLAM/回环重复回归与 truth isolation 通过 |
| 2. SLAM 教学体验与文档 | 已完成 | `4711d91` | 4/4 目标、零碰撞；任务连续三次通过；14 项相关回归通过；文档明确 SLAM Toolbox/REP-105 能力边界 |
| 3. 算法与系统回归 | 进行中 | - | 待完成 |
| 4. Release 性能与一致性 | 待开始 | - | 待完成 |
| 5. SIM 游戏体验 | 待开始 | - | 待完成 |
| 6. 物理证据可视化与效果 | 待开始 | - | 待完成 |
| 7. 模型与场景升级 | 待开始 | - | 待完成 |
