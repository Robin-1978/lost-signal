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
- 当前阶段：第 7 项；第 1–6 项均已独立提交，不得重新实现或回退已经验收的边界。
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

### 阶段 6 完成检查点（2026-10-27）

- 阶段 6 的算法/物理稳定性与 Viewer/音频效果已完成，提交为 `cdea986 feat: drive viewer effects from physical evidence`：
 - `PhysicalAudioEvidence` 按 sim step 消费真实 wheel speed、逐轮 normal load/contact motion/slip、service brake、impact impulse/relative speed。
 - Viewer 播放 motor、rolling、slip、brake 四层循环声及由物理碰撞证据控制增益的 impact；暂停、Turbo、静音、重复帧和回放回退不会补播历史事件。
 - 2D/3D Validator 与 Safety sweep 使用连续 capsule/tube；Viewer 直接消费 ROBOT costmap、Validator 和 Safety Monitor 证据，不在绘制时重建算法结果。
 - `--lidar`、`--robot-evidence`、`--shot` 支持可复现视觉验收；HUD 明确区分 `ROBOT OBS/AI`、`ROBOT ESTIMATE` 与 `SIM PHYSICS/AUDIO`。
 - 新增无音频设备依赖的 `test_physical_audio`、`test_viewer_audio`，以及 `motor_drive.wav`、`wheel_roll.wav`、`tire_slip.wav`、`service_brake.wav`。
- Release/Debug 定向回归已通过；Debug 全量 CTest 78/78 通过，Release 定向 9/9 通过；临时 `stage6-*.png` 已删除。
- 终端差速重捕获按位置误差投影选择前进/倒退 gear；低抓地斜坡的终端转向只在 ROBOT 有效姿态证据下允许由剩余位置误差推导的 crawl，默认平面/未知姿态仍保持纯旋转。
- MuJoCo 接触求解按真实 SIM 接触几何选择：普通平面保持默认 PGS；存在非平面、粗糙或有高程的 terrain patch 时使用 elliptic Newton、`impratio=10`、50 次主迭代和显式线搜索容差。该条件不读取关卡 ID、actor 真值或 ROBOT 状态。
- Release/Debug 的 autonomy、perception、DWA、dynamic escape/crossing、terrain、scenario、safety experience、composite、signal_town 和 20 万平方米定向回归均通过；terrain 与 industrial 均为 5/5、零碰撞。
- `tests/test_terrain_mission.cpp` 已恢复正式 2500 步预算并删除临时诊断。

### 阶段 7 预审检查点（2026-10-27）

- 现有机器人 drive wheel、steer knuckle 与 caster 已读取真实 MuJoCo link pose；wheel/knuckle GLB 绑定消费这些 pose，渲染端不再重建 steering/spin 姿态。Mecanum 左右 roller 由物理模块的 `y_m` 选择，不能由绘制序号猜测。
- `asset_selection_for_robot()` 现在以 `chassis_kind` 为权威来源，覆盖 differential、Ackermann、tracked、swerve、mecanum；多个底盘共用 `robot_id = ro1` 时仍会加载正确的资产族。
- 阶段 7 的首个子阶段已完成：backend 导出 drive/steer/idler/caster 的真实 joint axis、anchor、angle、rate、全部活动 contact 点/法向/normal load 和 base-link 相对 COM；这些字段通过 SIM-only `PresentationSnapshot` 进入 `FrameBody`，ROBOT observation/decision/actuation 没有新增真值入口。工程视图可显示 COM、joint axis 与 contact evidence。
- forklift 与 pedestrian 已迁入 MuJoCo mocap body：碰撞几何参与引擎接触，pose/heading/ground height 由 engine read-back 驱动历史与实时 GLB；LiDAR 仍通过匿名 actor 观测路径消费，不把 actor identity 或真值穿透给 ROBOT。新增配置驱动的四关节 ArmParams、MuJoCo link/joint chain 和 SIM-only `ArmLinkViz`，RO-6 turret/upper_arm/forearm/tool 按 link pose 绑定；下一步继续补机械臂控制/接触验收和 Blender 场景资产。
- 阶段 7 依次完成：按底盘能力加载模型和左右部件；建立统一 `link_binding`；加入显式 SIM debug 的 joint axis/collision geom/COM/contact force；把 forklift、pedestrian、机械臂根和关节迁入 MuJoCo；最后扩展 Blender 环境资产并保持碰撞代理诚实。
- Blender 路径为 `/disk/blender-5.2.2-linux-x64/blender`。旧代码分支实际为 `main`，仓库没有 `master` 分支。

### 阶段 6 算法回归最新检查点（2026-10-27）

- terrain 的通用修复已经完成。差速底盘在终端位置漂到车后、或位置进入包络但终端航向尚未稳定且目标落到车后时，会根据现有几何误差选择倒车重捕获；`requested_gear` 已进入 DWA 输出、采样和评分。实现只使用误差投影、现有置信包络和底盘能力，没有新增阈值。`test_terrain_mission` 已恢复正式 2500 步预算并删除临时 trace；Debug/Release 的 `test_dwa` 和 `test_terrain_mission` 均通过。
- 不确定到达的永久停车已经修复。`GoalArrivalDecision::observation_motion_required` 在名义估计进入原始目标容差、但置信区域尚不能放进目标包络时请求低速观测运动；运动仍由 `MotionDecisionPipeline` 产生，并经过 Validator 和 Safety。严格到达仍要求置信包络、姿态和稳定停车。策略位于 goal-arrival/decision 模块，Coordinator 只合并并传递请求。Debug/Release `test_autonomy` 通过。
- 高定位协方差下的动态物体丢失已经修复。LiDAR 中处于 map-relative foreground 强证据门与协方差扩张门之间的回波保留为 temporal-only 候选，只有经过现有运动更新次数和 1 秒速度拟合窗口后才发布匿名动态 track；静态墙在高协方差下仍不能成为动态物体。复用现有 `foreground_threshold_m`，没有新增阈值。Debug/Release `test_perception` 通过。
- 新增 `tests/test_dynamic_crossing_mission.cpp`：Ackermann 底盘分别验证 actor 在机器人之前、同时和之后到达横穿点；链路使用真实 LiDAR、匿名 track、prediction、behavior、DWA、Validator 和 Safety，要求零碰撞、危险相位产生预测响应并在风险消失后恢复前进。Debug/Release 均通过，SIM 真值只用于离线断言。
- Debug/Release `industrial_200k_mission` 完整五目标均通过；Release 长测为 70593 步、5/5、0 碰撞、最大定位误差 1.073 m。`signal_town` Release 为 5826 步、5/5、0 碰撞。
- 当前尚未提交的算法回归文件包括 `dwa_planner.cpp`、`goal_arrival.hpp/.cpp`、`coordinator.cpp`、`lidar_perception.cpp` 及对应 DWA、terrain、autonomy、perception、dynamic-crossing 测试。不得把这些修改与 Viewer/音频提交混为同一阶段提交，也不得丢弃。

### 阶段 7 配置与控制检查点（2026-10-27）

- `525116c feat: load articulated arm geometry from robot config` 已把机械臂从只能程序化设置改为正式机器人配置契约。`arm.enabled`、`base_xyz_m`、四段连杆的长度/质量/半径、关节轴、初始角度、扭矩上限和阻尼均严格解析；数量、有限性、正几何、非零轴和 actuator 参数错误会 fail-closed。示例配置为 `configs/robot/ro6_arm.json`。
- `53bd9cf feat: add articulated arm actuator feedback contract` 已把四关节 arm effort `[−1,1]` 通道加入 `ActuatorCommand`，把 encoder 角度/角速度加入 `MotionFeedback`/`ObservationFrame`，并贯穿 envelope、同步器、回放 v4 和 native boundary。MuJoCo 通过配置的每关节扭矩上限和阻尼施加真实 effort，反馈来自引擎 qpos/qvel；没有 draw-time 姿态或 SIM 真值入口。`mujoco_backend`、`replay`、`robot_config`、`robot_config_native`、`robot_cycle`、`truth_isolation`、`viewer_frame`、`actuation` 定向回归通过。
- 去掉上述 arm 改动后，`industrial_200k_mission --steps=1000` 与当前短轨迹一致；该任务完整 71223 步在 fac57d0 基线也稳定复现第 5 个目标 SIM truth miss 2.820 m（4/5、零碰撞），归因尚未完成，不能以放宽到达条件掩盖。
- 2026-10-27 的 Release 基线复测（当前 HEAD、无额外实验改动）为 71640 步、5/5、0 碰撞、最大定位误差 3.755 m；五个目标 truth miss 为 0.060/0.081/0.181/0.168/0.199 m，最终 `goal_hold/arrived/stop`。此前 71223 步的 4/5 结果没有在这次复测中重现，不能把一次通过当作算法证明；后续仍要保留重复长测和结构化定位证据。
- 下一恢复点：先在当前干净工作区重复工业长测并保留定位候选/时序证据，再继续 worker 动画、forklift link 拆分、Blender 环境资产和碰撞验收；每个修复独立提交。

### 持续恢复锚点（2026-10-27，`d9cebd6`）

- 当前分支为 `refactor/clean-rebuild`，工作区在本锚点应保持干净；恢复时先执行
  `git status --short --branch` 与 `git log -5 --oneline`，确认 HEAD 至少包含
 `d9cebd6 feat: drive worker animation from sim motion`。
- 已完成的机械臂配置、MuJoCo link/joint pose、effort/encoder feedback、回放 v4、动态 actor mocap、GLB link binding、COM/contact/joint-axis overlay 和物理音频均视为已验收边界。恢复工作不得回退这些契约，也不得把 SIM-only 字段加入 `src/robot/`。
- 第 7 阶段第一个未完成子任务是“模型与场景升级”：先让 worker 行人和 forklift 的分部件视觉绑定消费真实 MuJoCo body/joint pose，再补 Blender/GLB 场景资产 manifest；每个子任务都必须有对应的 backend/frame/viewer 单测或截图验收，并单独提交。
- 工业长测的 `4/5` 结果曾在干净工作区复测为 `5/5`，因此只能作为需重复的稳定性证据；后续长测必须保留定位候选、协方差、恢复动作和时序记录，不能用放宽到达条件、读取真值或关卡专用常量掩盖波动。

### 压缩后恢复协议（必须遵守）

1. 读取本文件的“持续恢复锚点”和“最新恢复顺序”，再检查 `git status`/`git log`；不要从对话记忆猜测已完成内容。
2. 从表格中第一个“进行中”阶段的第一个未完成子任务继续。当前入口是第 7 阶段的 worker/forklift link binding；完成后更新本文件、记录测试证据并单独提交。
3. 任何失败先记录可复现命令、输入 revision、SIM/ROBOT 边界和结构化证据，再修改代码。已撤销的动态回波过滤、无条件 prior-improvement、post-step velocity projection 等实验不得重新引入。
4. 每次阶段性提交后立即更新本文件中的提交号、测试结果和剩余风险，确保下一次上下文压缩仍能从同一锚点恢复。

### 第 7 阶段 worker 子阶段（2026-10-27）

- worker GLB 的 `WorkerWalk` clip 已接入 Viewer；动画相位由记录帧的 SIM 时间、MuJoCo mocap 根位姿对应的实际速度和 manifest `animation_stride_m` 推导。暂停、回放、Turbo 和多 worker 不共享墙钟相位，停下时回到站立帧。
- `AssetDefinition`/manifest 现在显式声明动画步长；Blender `inspect_assets.py` 对带 clip 的资产强制检查有限的正步长。资产仍保持 `actor_root` 地面 pivot，根位姿直接来自 MuJoCo read-back。
- 验收：`test_asset_manifest`、`test_viewer_frame`、`test_mujoco_backend`、`test_sim_actor` 通过；Blender 5.2.2 `inspect_assets.py` 报 `ASSET_MANIFEST_OK assets=44`；Xvfb 下 `sim_viewer --3d --map configs/worlds/industrial_200k.json --mute` 成功加载 `WorkerWalk (50 frames, 0.833333s)`。
- 本子阶段不改变 ROBOT observation/decision/actuation，也未把 SIM actor identity 或真值加入 `src/robot/`。下一子阶段：forklift 的车体、驱动轮、转向轮、门架、carriage 和货叉 link 绑定真实 MuJoCo body/joint pose。

### 阶段 1 回环修订检查点（2026-10-27）

- 回环异步搜索的 pending proposal 不再按墙钟超时清理；proposal 绑定同一历史 keyframe anchor，只有该 anchor 被有界 keyframe ring 丢弃时才失效。这样不会在 coarse/refine/reverse 分片执行期间丢失第一次真实重访证据。
- coarse retrieval 与 dense refinement 只负责候选关联；reverse association 使用 endpoint occupancy，最终接受仍由不相交的 historical/recent causal submap 的 expected-range 双向证据、pooled overlap、score margin、信息矩阵和同锚点时序确认共同决定。SIM 真值仍只出现在测试诊断。
- `test_loop_closure` 在目标完成后保留固定观测尾窗，验证异步 matcher 的最终状态；首次闭环在完成前三个目标后的真实重访阶段发生。当前 Debug 重复回归：`truth_isolation`、`slam`、`loop_closure`、`slam_experience_mission`、`robot_boundary` 共 5 项各重复 3 次通过。
- 该修订提交后，继续从第 6 项 Viewer/音频独立提交，再进入第 7 项模型与场景升级；不要把工作区中未提交的 Viewer/音频文件混入本阶段提交。

最新恢复顺序：

1. 删除 `stage6-*.png` 临时截图；跑 Debug/Release 全量 CTest、truth isolation、ROBOT boundary 和 `git diff --check`。
2. 将算法、物理求解选择和相关测试单独提交；不得把这些修改与 Viewer/音频提交混为同一阶段提交。
3. 再单独提交 Viewer、物理音频、WAV、文档及其测试；随后立即进入阶段 7。

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
| 6. 物理证据可视化与效果 | 已完成 | `cdea986` | 算法/物理与 Viewer/音频完成；Debug 全量 78/78；Release 定向 9/9；物理音频单测覆盖暂停、回退、Turbo、接触、打滑、制动和冲击 |
| 7. 模型与场景升级 | 进行中 | `525116c`, `53bd9cf`, `d9cebd6` | 机械臂配置、MuJoCo effort/feedback、worker `WorkerWalk` 和回放契约已完成；叉车 link 拆分已完成于工作区，Blender 环境资产和最终视觉/Release 验收待完成 |

### 阶段 7 叉车 link 子阶段（工作区验收，待提交）

- Blender 5.2.2 重新导出了完整叉车和六个可复用分部件 GLB：base、drive
  wheel、steer wheel、mast、carriage、fork。每个文件的单位、轴、pivot、
  grounding point、bounds、材质/三角形预算和 FNV-1a 内容哈希均进入 manifest；
  `inspect_assets.py` 在 50 个资产上通过。
- MuJoCo 将 `vehicle`（兼容既有 sidecar）和显式 `forklift` actor 的可视
  部件作为 root 的 SIM-only child body，`World::actor_links()` 读取真实 body
  pose。root collision proxy 不变，actor link 表没有进入 ROBOT observation、
  decision 或 actuation。
- Frame/episode 保存 link pose，Viewer 加载分部件并按各自 manifest 的
  `asset_to_link` 变换和 MuJoCo link quaternion 绘制；只有缺少完整分部件时才
  回退到完整 forklift GLB。component 不再用最低顶点自动落地，避免 chassis/mast
  clearance 被错误压低；整体 actor 仍由真实 root pose 和 terrain anchor 放置。
- 定向验收：`test_mujoco_backend`、`test_viewer_frame`、`test_asset_manifest`
  和 Blender 资产检查通过；`sim_viewer --3d --map
  configs/worlds/signal_town.json --mute --shot 120 ...` 成功加载全部分部件、
  WorkerWalk 和 3D 渲染。下一步是提交该子阶段，然后扩展环境资产和最终 Release
  视觉/碰撞验收。
