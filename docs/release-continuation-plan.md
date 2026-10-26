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
 - 完成 LiDAR 时序扫描、Safety swept envelope、local costmap/inflation、planner swept volume 和定位修正 ghost；显示形状必须忠实于 Safety Monitor 实际使用的 footprint 合同。
   - 粒子与声音只由真实 contact、slip、normal load、service brake 和 impact 驱动，并随 sim time 确定性复现。

7. **模型与场景升级**
   - 机器人、叉车、行人和机械臂全部由 MuJoCo body/joint pose 驱动。
   - 增加 joint axis、collision geometry、COM、contact overlay。
   - 按资产 manifest 的单位、轴、pivot、bounds 和 link binding 扩展 Blender/GLB 环境资产。

## 当前恢复点

- 已完成基线：`f22acc9 feat: make simulation experiments evidence driven`。
- `signal_town` Release 基线：5 个目标、0 碰撞、RTF 6.34x、ROBOT tick p99 6.432 ms、最大 tick 14.146 ms。
- 当前阶段：第 6 项；第 1–5 项均已独立提交，不得重新实现或回退已经验收的边界。
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

### 阶段 4 完成检查点（2026-10-27）

- 阶段 3 已提交为 `2c13308 fix: harden SLAM and route recovery`。
- Release 已用 `LOST_SIGNAL_WITH_VIEWER=ON` 完整配置；倍率一致性门禁逐步比较 1×/8×/64×/Turbo/headless 的 measurement stamps、observation acceptance、完整 command、碰撞、最终 truth、任务终态和完整 score，并覆盖已稳定完成的 ROBOT/SIM 双重 verdict。
- 20 万平方米 Viewer 软件渲染实测：1× 为 1.00×/50 steps/s，8× 为 5.47×/273 steps/s，64× 为 5.38×/269 steps/s，Turbo 为 6.35×/317 steps/s。超过本机能力时保留固定步并显示实际 RTF/LIMIT。
- 同场景 headless 1200 步为 6.40×/320.2 steps/s，ROBOT tick p99/max 9.249/18.929 ms。发布配置 `signal_town` 在 5901 步完成 5/5、0 碰撞，RTF 6.34×，ROBOT tick p99/max 6.432/14.146 ms。
- 详细方法、环境与复现命令见 `docs/release-performance-2026-10-27.md`。
- 阶段 4 已提交为 `fba2d87 test: verify accelerated simulation consistency`；当前 HEAD 为 `fba2d87`，分支相对远端领先 5 个提交。

### 阶段 5 完成检查点（2026-10-27）

- typed `ExperimentSpec` 已绑定 experience/experiment ID、变量/单位、baseline/variant 值、两局预算、证据容差、输入 digest 和代码 revision。`EpisodePair::experiment_valid()` 会逐项核对两份 artifact 的 condition、变量值、预算、map revision、digest 和 revision。
- `EpisodeArtifact` 已分开记录 ROBOT 与 Mission verdict：ROBOT 只读取 goal Done/abandoned；Mission 只读取 SIM 的 truth arrival 合同；SIM safety 只读取碰撞结果；Episode 完整性保留为内部 evidence gate。
- Viewer 与 v2 bundle 对外固定输出 `ROBOT / SIM SAFETY / MISSION / EXPERIMENT`。bundle 同时保存完整 spec、每局 replay、score、keyframe、输入 digest 和 revision；不匹配的 digest、预算或 condition 会使 Experiment verdict 失败。
- 预测证据已搬入 `episode_evidence()`，只读取 captured artifact/pair，不再读取 live `ScenarioSession`。无效但完整的 A/B 证据返回 `InsufficientEvidence`，不会误判预测或永久显示 pending。
- 玩家首屏从约 700 px 缩为 430 px，First Contact 的部件卡改为可选教学；6 秒 hero 后可立即进入预测，完整知识仍可在教学层查看。实机 Xvfb/llvmpipe 1100×750 截图已验收无遮挡。
- manifest 到 runtime 的字段复制已收敛到 `experience_runtime_config()`；初始章节与 campaign 跳转共用同一路径，`slam_loop` 的 `slam_only` 不会再丢失。重复 `snprintf` 已清理。
- 定向构建通过；Experience/asset/scene/keyframe/A-B/runtime 共 10 项回归 10/10 通过，包含 v2 bundle 和两份 v4 replay round-trip、三层视图策略、首个决定预算及 startup mode。

### 阶段 6 当前检查点（2026-10-27）

- 阶段 6 功能实现已完成，尚未提交：
 - `PhysicalAudioEvidence` 按 sim step 消费真实 wheel speed、逐轮 normal load/contact motion/slip、service brake、impact impulse/relative speed。
 - Viewer 播放 motor、rolling、slip、brake 四层循环声及由物理碰撞证据控制增益的 impact；暂停、Turbo、静音、重复帧和回放回退不会补播历史事件。
 - 2D/3D Validator 与 Safety sweep 使用连续 capsule/tube；Viewer 直接消费 ROBOT costmap、Validator 和 Safety Monitor 证据，不在绘制时重建算法结果。
 - `--lidar`、`--robot-evidence`、`--shot` 支持可复现视觉验收；HUD 明确区分 `ROBOT OBS/AI`、`ROBOT ESTIMATE` 与 `SIM PHYSICS/AUDIO`。
 - 新增无音频设备依赖的 `test_physical_audio`、`test_viewer_audio`，以及 `motor_drive.wav`、`wheel_roll.wav`、`tire_slip.wav`、`service_brake.wav`。
- Release/Debug 构建和阶段 6 的 17/17 定向测试已通过；`stage6-*.png` 是临时验收截图，提交前必须删除。
- Debug 全量回归当前为 75/77，暴露两个算法鲁棒性问题，不能用 Viewer 改动、关卡特化或改变场景掩盖：
 - `terrain_mission` 在冰面坡道完成第一个目标后，于第二目标附近因差速底盘终端重捕获选择正向 gear 而绕圈。工作区的通用修复根据前进/倒退所需的最小车体转角选择短距离重捕获，复用已有终端容差、置信包络和底盘能力，没有新增阈值；尚需完成输出 gear、单元测试和 Debug/Release 验证。
 - `industrial_200k_mission` 的 Debug 轨迹在前往 `south_yard` 时与 `east_shuttle` 相遇并锁死，100000 步只完成 4/5 且发生 691 个碰撞 step；Release 同任务 62169 步完成 5/5、零碰撞。需要建立短小、确定性的动态横穿相位测试，再修复 predictor/behavior/DWA/Validator/Safety 的通用链路。SIM actor truth 只允许用于失败诊断。
- `tests/test_terrain_mission.cpp` 当前含临时诊断：第一段预算临时为 3000，2200 步后每 25 步输出状态。提交前必须恢复为 `drive_until_done(session, 2500)` 并删除 trace。

### 阶段 7 预审检查点（2026-10-27）

- 现有机器人 drive wheel 与 caster 已读取真实 MuJoCo link pose；后续必须保留这条 body/joint 到 GLB link 的数据链。
- `load_visual_assets()` 当前只按 `robot_id` 选择模型，而 Ackermann、Mecanum、Swerve 配置共用 `ro1`，会错误加载 RO-1 外观。模型选择必须绑定底盘/资产 manifest，不能靠绘制端猜测。
- manifest 已声明但尚未完整绑定 RO-2 front knuckle、RO-4 swerve module、RO-5 左右 Mecanum roller、RO-6 turret/upper_arm/forearm/tool；后端还未导出 steer body pose。
- forklift 与 pedestrian 仍是 SIM kinematic actor，不是 MuJoCo body/joint；forklift/worker GLB 虽含命名部件或骨架，目前仍整体绘制。机械臂也必须逐关节读取物理状态。
- 阶段 7 依次完成：按底盘能力加载模型和左右部件；建立统一 `link_binding`；加入显式 SIM debug 的 joint axis/collision geom/COM/contact force；把 forklift、pedestrian、机械臂根和关节迁入 MuJoCo；最后扩展 Blender 环境资产并保持碰撞代理诚实。
- Blender 路径为 `/disk/blender-5.2.2-linux-x64/blender`。旧代码分支实际为 `main`，仓库没有 `master` 分支。

### 阶段 6 算法回归最新检查点（2026-10-27）

- terrain 的通用修复已经完成。差速底盘在终端位置漂到车后、或位置进入包络但终端航向尚未稳定且目标落到车后时，会根据现有几何误差选择倒车重捕获；`requested_gear` 已进入 DWA 输出、采样和评分。实现只使用误差投影、现有置信包络和底盘能力，没有新增阈值。`test_terrain_mission` 已恢复正式 2500 步预算并删除临时 trace；Debug/Release 的 `test_dwa` 和 `test_terrain_mission` 均通过。
- 不确定到达的永久停车已经修复。`GoalArrivalDecision::observation_motion_required` 在名义估计进入原始目标容差、但置信区域尚不能放进目标包络时请求低速观测运动；运动仍由 `MotionDecisionPipeline` 产生，并经过 Validator 和 Safety。严格到达仍要求置信包络、姿态和稳定停车。策略位于 goal-arrival/decision 模块，Coordinator 只合并并传递请求。Debug/Release `test_autonomy` 通过。
- 高定位协方差下的动态物体丢失已经修复。LiDAR 中处于 map-relative foreground 强证据门与协方差扩张门之间的回波保留为 temporal-only 候选，只有经过现有运动更新次数和 1 秒速度拟合窗口后才发布匿名动态 track；静态墙在高协方差下仍不能成为动态物体。复用现有 `foreground_threshold_m`，没有新增阈值。Debug/Release `test_perception` 通过。
- 新增 `tests/test_dynamic_crossing_mission.cpp`：Ackermann 底盘分别验证 actor 在机器人之前、同时和之后到达横穿点；链路使用真实 LiDAR、匿名 track、prediction、behavior、DWA、Validator 和 Safety，要求零碰撞、危险相位产生预测响应并在风险消失后恢复前进。SIM 真值只用于离线断言。Debug 已通过，Release 尚待运行。
- Debug `industrial_200k_mission` 正在以本轮修复后的 binary 做完整五目标复测。当前统一 exec session 为 `53069`，最后记录约为 step 42500；central worker 横穿已形成匿名动态 track 并进入 `dynamic_clearance` 安全停车。关键 `east_shuttle` 横穿预计在 step 52000–54000，必须继续运行至任务终态，不能以中途状态判定通过。
- 当前尚未提交的算法回归文件包括 `dwa_planner.cpp`、`goal_arrival.hpp/.cpp`、`coordinator.cpp`、`lidar_perception.cpp` 及对应 DWA、terrain、autonomy、perception、dynamic-crossing 测试。不得把这些修改与 Viewer/音频提交混为同一阶段提交，也不得丢弃。

最新恢复顺序：

1. 若 session `53069` 仍存在，先轮询到 `industrial_200k_mission` 完成；若进程已不存在，用现有 Debug binary 重新运行同一测试并保留 step 51500–53500 的匿名 track、预测 clearance、Safety 和 actor 离线诊断。
2. 若 20 万平方米任务失败，只修复 ROBOT 的通用感知、预测、行为、局部规划、Validator 或 Safety 链路；不得移动 `east_shuttle`、修改关卡或放宽碰撞/到达条件。
3. 运行 Release 的 `test_dynamic_crossing_mission`、`test_perception`、`test_autonomy`、`test_dwa`、`test_terrain_mission` 和 `test_industrial_200k_mission`。
4. 全部通过后，只暂存算法和相关测试，单独提交“算法鲁棒性回归修复”。
5. 删除 `stage6-*.png` 临时截图；跑 Debug 全量（新增测试后预计 78 项）、truth isolation、ROBOT boundary 和 `git diff --check`。
6. 更新本文件的阶段 6 验收记录，再单独提交 Viewer、物理音频、WAV、文档及其测试；随后立即进入阶段 7。

恢复后按以下顺序继续：

1. 读取本文件，执行 `git status --short --branch` 和 `git log -5 --oneline`，确认 HEAD 至少包含阶段 5 提交 `f22acc9`。
2. 直接从“阶段 6 算法回归最新检查点”继续；不得回退 immutable Episode、四类 verdict、ROBOT/SIM 边界、物理音频或已有物理证据可视化。
3. 所有新增表现只读取已记录的 observation、decision、MuJoCo contact/joint state、actuator feedback 和 sim time；不得把视觉/音频层状态反向输入 ROBOT 或 physics。
4. DWA 终端倒车重捕获、terrain 正式预算、动态横穿相位测试和高协方差 temporal-only track 已完成；先完成正在运行的 20 万平方米任务和 Release 定向回归。
5. 回归修复单独提交；随后跑 Debug 全量（新增测试后预计 78/78），更新阶段 6 验收记录，删除临时截图，通过 `git diff --check`、truth isolation 和 ROBOT boundary 后单独提交阶段 6。
6. 提交阶段 6 后立即进入第 7 阶段：先验证 MuJoCo body/joint 到 GLB link binding，再增加 joint axis、collision geometry、COM/contact overlay，最后扩展 Blender 环境资产；禁止 draw-time 落地偏移、猜轴或墙钟动画。

## 阶段完成记录

每完成一项，在这里记录提交、测试、性能数据和剩余风险，再进入下一项。

| 阶段 | 状态 | 提交 | 验收证据 |
|---|---|---|---|
| 1. 无图 SLAM 回环可信度 | 已完成 | `a66d644` | 首次回环前已完成 3 个目标；SLAM/回环重复回归与 truth isolation 通过 |
| 2. SLAM 教学体验与文档 | 已完成 | `4711d91` | 4/4 目标、零碰撞；任务连续三次通过；14 项相关回归通过；文档明确 SLAM Toolbox/REP-105 能力边界 |
| 3. 算法与系统回归 | 已完成 | `2c13308` | 完整构建；定向 14/14；全量 75/75；20 万平方米、signal_town、四底盘、SLAM/回环、truth isolation 均通过 |
| 4. Release 性能与一致性 | 已完成 | `fba2d87` | Release 75/75；1×/8×/64×/Turbo/headless 实测；逐步确定性门禁；发布 signal_town 5/5、0 碰撞 |
| 5. SIM 游戏体验 | 已完成 | `f22acc9` | typed ExperimentSpec；immutable EpisodePair；四类 verdict；10 秒内首次预测；Experience 回归 10/10；Viewer 截图通过 |
| 6. 物理证据可视化与效果 | 进行中 | - | 功能与 17/17 定向测试完成；Debug 全量 75/77，先修复 terrain 终端重捕获和 20 万平方米动态横穿鲁棒性 |
| 7. 模型与场景升级 | 待开始 | - | 待完成 |
