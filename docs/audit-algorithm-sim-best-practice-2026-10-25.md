# ROBOT 算法与 SIM 真实性发布审计

审计日期：2026-10-25  
审计分支：`refactor/clean-rebuild`  
对照分支：本地 `main`（仓库没有 `master`）  
范围：定位、建图、规划、控制、安全、执行器契约、物理、传感器、评分、原生运行时和旧版能力迁移。

## 1. 发布结论

当前分支已经建立正确的新框架边界：MuJoCo 是 SIM 物理真值源，ROBOT 只消费 provider 发布的传感器观测、公开地图和自身估计；碰撞、actor 脚本、材质、接触、真值位姿和评分不会进入 ROBOT 决策。`tests/check_boundary.py` 对 include、link 和 truth 类型执行静态门禁。20 万平方米差速底盘长测也证明现有 A*、定位和流式评分可以在大图上运行。

当前版本仍不满足真机发布要求。本轮最初确认四项 P0；异步传感器时间语义、Ackermann 大图规划和机械制动执行契约已经按本审计方案修复并进入回归。剩余 P0 是 native robot 仍为一次 tick 后退出的 bringup stub。另有 polygon footprint、低抓地制动、执行跟踪故障闭环、传感器默认真实性和动态 actor 动力学等 P1 缺口。

优先级定义：P0 阻止对应产品路径发布；P1 会在合理环境或故障条件下造成安全性、鲁棒性或真实性缺口；P2 是能力完整性和进一步逼真度提升。

## 2. 已验证的优势

- ROBOT/SIM 编译边界成立，SIM 真值只用于物理、显示、记录和离线评分。
- 地形物理使用连续 canonical height，survey 与引擎几何一致；连续坡面接缝不再暴露有限碰撞 slab 的侧壁；ROBOT 只有在部署地图明确发布 elevation/survey prior 时才能读取该先验。
- LiDAR 使用 3D raycast，并已支持安装位姿、rolling shutter、actor 命中、强度和噪声模型；IMU 已有安装位姿、bias、bias walk 和 white noise；encoder 已有量化；传输已支持 latency/dropout。
- 全局恢复搜索已跨 tick 分片；局部匹配拒绝不会直接触发全局搜索，只有估计有效性或协方差达到策略阈值才进入恢复。
- mapless SLAM、scan-to-submap、loop closure 和 robust pose graph 已进入新框架，决策过程中不读取真值。反向于轮速先验的 scan-to-submap 修正现在按模式处理：有先验地图时相对子图证据不能改变绝对权威，反向修正只允许由正式 scan-to-map 定位链提交；无先验地图时必须在同一 submap anchor 上由第二个独立运动视角确认。单帧反向修正仍拒绝。所有缺少独立绝对地图确认的回环，无论单次 correction 大小，都必须经过跨视角的刚体变换一致性确认，避免多个“小修正”累计拖走 `map`。
- SIM 的逐轮打滑标签读取 MuJoCo 已求解的接触法向载荷、切向力和当前 friction cone，并以该接触的 `solref` 响应时间过滤单个子步的边界触碰；split-μ、载荷转移和短暂离地不再被平均轴载荷掩盖。连续纵/横滑移、法向载荷和摩擦利用率只进入 SIM viewer/离线诊断。
- 全局和局部 costmap 已统一为占据硬膨胀加连续衰减 soft layer；DWA、TEB-style、Validator 和 Safety 形成分层检查。
- Collision Monitor 直接读取 ROBOT 原始扫描，独立于全局定位和 costmap，符合最终安全层的职责方向。
- 动态预测支持常速、静止、走廊反射、备选意图和增长的不确定性，并用连续时间最小净空避免离散时刻漏检。
- 20 万平方米差速底盘五目标 Release 长测完成 5/5、0 碰撞；ROBOT tick p50/p95/p99/max 为 1.165/3.192/4.359/13.560 ms，Release CTest 59/59。

## 3. P0 发布阻断项

### P0-1 异步多传感器在统一 envelope 中丢失时间与来源

**原始源码证据。** 修复前 `ObservationFrame` 只有一组 `sequence/stamp_ns/source/frame`，却同时装载 scan、odometry/joints 和 IMU；`SimSensorSuite::poll_envelope()` 按各自 cadence 决定 section，而 Coordinator 收到 envelope 后整体覆盖 `last_obs_`。该实现已由下述修复替换。

**影响。** 100 Hz IMU 或 50 Hz motion 会覆盖 10 Hz LiDAR 帧，使下一次 50 Hz 决策看到 `scan_valid=false`，Safety/Validator 因缺扫描证据停车。若将旧 scan 与新 motion 合并，又会把 scan 错误标成 envelope 的新时间，破坏 freshness、deskew、scan matching 和乱序检查。单一 source 也无法证明每个 section 来自哪个 endpoint。

**已复现。** 使用 scan/motion/IMU/decision 为 0.10/0.02/0.01/0.02 s、零延迟零丢包运行 `energy_hub`：1000 步仅移动 0.67 m，并反复报告 `no scan evidence for motion+safety:no-scan-evidence`。默认四路均为 0.02 s 时，同样 1000 步移动 20.87 m。默认同步频率掩盖了契约错误。

**修复。** 每种 typed measurement 必须保留独立 acquisition timestamp、sequence、frame 和 source；ROBOT 为各类测量保存独立缓存和 freshness，按采集时间有序送入估计器。决策使用仍在有效期内的最新扫描，而不是要求本决策周期恰好收到新扫描。Safety 单独检查 LiDAR 年龄并在超时后 fail-safe。不得由 Coordinator 推测或改写采集时间。

**验收。** 增加 10 Hz LiDAR、50 Hz motion、100 Hz IMU、50 Hz decision 的异步相位测试，并覆盖乱序、延迟、重复 sequence、future skew 和 LiDAR 超时。正常新鲜扫描下不得周期性 stop-go；过期扫描必须确定性停车；每个样本只积分一次。

**修复状态：已完成。** `ObservationFrame` 现在为 scan、motion/joints、IMU 保存独立 `MeasurementHeader`；应用入口逐 section 验证 endpoint ownership，headerless legacy frame 只允许一个 endpoint section。排序、去重、缓存、IMU 外参变换和一次性消费全部下沉到 `MeasurementSynchronizer`。其上层 `ObservationPipeline` 完整拥有异步测量处理：按采集时间分发 estimator event，仅在新 scan 到达时触发 perception/mapper，生成一次性 localization/adaptation view，并持有定位状态、感知结果和地形观测时钟。Coordinator 只调用该子系统并串联其输出，不再保存或判断异步测量状态。估计器只处理本次新到的 motion/IMU，迟到样本不会倒退积分时钟；感知、建图、定位只处理新 scan；Validator/Safety 使用缓存的最新 scan，Safety 按 scan 自身采集时间 fail-safe。执行器闭环只接受 `safety_reaction_time_s` 内的新鲜 joint feedback，超时、未来时间或缺失反馈都关闭 motor duty 并返回机械 `SafeStop`。SIM 不再在新 envelope 中重复携带旧 joint 值。JSONL 升级为保留 section header 的 v3，并兼容读取 v2。

`World::sim_time_ns()` 现在是 SIM 唯一采集时钟，20 ms public transition 会推进到内部各传感器 deadline；300 个 transition 明确产生 600 个 100 Hz IMU measurement。`signal_town` 异步复测在 984 步完成目标，移动 20.68 m、0 碰撞，ROBOT tick p99/max 为 12.725/13.876 ms，不再出现 `no-scan-evidence` 周期误停；修复前 1000 步只移动 0.67 m。新增模块回归覆盖独立来源与时间、旧 scan 缓存、scan 超时、迟到 measurement、future skew、endpoint 冒充、异步场景和 replay header round-trip。

### P0-2 Ackermann Hybrid-A* 在 20 万平方米地图上必然返回 invalid

**源码证据。** `AckermannPlanner` 默认 `max_states=4,000,000`、`heading_bins=72`，状态包含前后档；当前实现同步构建全图 costmap 并分配 `cells × bins × 2` 状态。即使降到最低 8 个 heading bins，2032×1632 地图仍需要 53,059,584 个状态，超过预算后在搜索前直接返回。

**已复现。** `ackermann.json + industrial_200k.json + west_hub` 运行 500 步，位移 0.00 m，任务未完成，tick 最大 49.544 ms；同场景差速底盘已移动 2.53 m。

**修复。** 规划器使用基于起终点和障碍走廊的 ROI，在粗分辨率上先求可达走廊，再在局部细化 Hybrid-A*；按 map revision 缓存 costmap/obstacle heuristic；搜索必须跨 tick 或受严格 wall-time/expansion budget 限制。禁止用无限增大 `max_states` 掩盖架构问题。

**验收。** Ackermann 在 20 万平方米地图所有发布目标上能规划和执行；内存不随全图 `cells × headings × gears` 分配；单 tick 不产生长阻塞；窄门、倒车、最小转弯半径和重新规划均有模块测试。

**修复状态：已完成。** 全图 A* 只生成二维代价感知 guide，Hybrid-A* 在走廊内按转弯半径推导的有限距离分段细化，稀疏状态跨 decision tick 增量扩展，不再分配全图 `cells × headings × gears`。地图膨胀层按 revision 缓存，路线长度通过连续局部段处理，没有固定总 expansion 上限。搜索同时验证自行车弧线和实际交付给局部规划器的折线弦，避免弧线绕过膨胀墙角、输出折线却切角后在最终复检失败。

20 万平方米 Ackermann 五目标任务完成 `5/5`、零碰撞；从启动障碍群直接重规划到 250 m 外 `central_gate` 的回归完成 250.20 m 行驶，truth miss 0.238 m。对应测试覆盖窄通道倒车、姿态目标、增量预算、失效姿态重规划、全任务和直达长任务。

### P0-3 安全制动模型与真实执行器契约断裂

**源码证据。** Robot JSON 配置 `brake_max_torque_Nm` 和 apply/release time；MuJoCo `ServiceBrakePlant` 与 `World::set_brake_request()` 能模拟机械制动。然而 `ActuatorCommand` 只有 drive/steer，`SimMotionEndpoint::consume_command()` 只下发 drive/steer，正常闭环从不驱动 brake request。SafeStop 实际依靠轮速 PID 的反向电机力矩，而 Safety、Validator 和 Behavior 以静态 `brake_deceleration_mps2` 计算停车距离。

**影响。** 安全层声称的制动能力、仿真实际执行和部署执行器没有共同契约；制动延迟、机械刹车失效、低抓地力和电驱限制无法被端到端验证。

**修复。** 在 actuator contract 增加有明确语义的 service/emergency brake command，SIM 与 native endpoint 都实现相同状态机。安全模型使用配置上限与 ROBOT 自身制动诊断估计下界中的保守值。SIM 物理接触和真值只能用于测试判定，不能反向更新 ROBOT 的在线制动能力。

**验收。** 用物理实测 stopping distance/velocity decay 校验安全包络；覆盖 apply/release lag、brake failure、低抓地、坡地和失去 motion feedback；命令过期必须进入确定性最小风险制动。

**修复状态：执行器契约已完成。** `ActuatorCommand` 已增加范围校验为
`[0,1]` 的 service-brake 通道；各底盘在正常停车时同时执行闭环电制动和
机械制动，feedback 缺失、命令非法及 watchdog 过期时关闭 motor duty 并
保留正确底盘模式的满额机械制动。`SimMotionEndpoint` 通过普通 actuator
seam 把请求送入 MuJoCo 的 lagged passive brake plant，发布 robot 配置显式
声明制动力矩与 apply/release 时间。端到端测试证明相同命令可停止运动中
plant、不会反向驱动车辆，并覆盖非法请求和过期降级。低抓地力下在线制动
能力估计仍列在 P1-1，必须只使用 ROBOT 自身 slip/执行反馈继续收敛。

### P0-4 native robot 尚不是可部署 runtime

`src/robot/app/main.cpp` 只创建 application、请求 hold、执行一次 `tick(0)` 后退出。目前只有 LiDAR replay/feed seam，缺少 encoder/motion、IMU、actuator 的 native endpoint，持续调度循环、通信桥接和 watchdog integration 也没有闭环。

发布验收必须至少提供持续运行的单调时钟循环、独立 provider channel、命令 expiry/watchdog、受控启动/关闭和故障状态；用 fake/native loopback provider 证明与 SIM 使用同一 ROBOT application contract。若本产品版本明确只发布仿真 SDK，则必须在发布范围中排除 native deployment，不能把 stub 声称为可部署机器人程序。

## 4. P1 安全与鲁棒性缺口

### P1-1 低抓地力没有降低制动能力

`GripAdapter::effective_limits()` 目前只降低前进速度、角速度和横向速度；Safety、Validator、Behavior、DWA/TEB 的减速度仍使用启动时静态值。同样速度和障碍距离下，低 grip 的 braking tail 因此与高 grip 相同。

修复应只使用 ROBOT 侧 slip/grip belief 或实测减速诊断，按保守下界降低有效制动减速度。测试必须证明 grip factor 越低，允许速度不增、制动尾迹不缩短。不得读取 SIM friction/contact truth。

### P1-2 全链路只支持圆形 footprint

`footprint_radius_m` 贯穿 A*、Hybrid-A*、DWA、TEB-style、Validator、Safety 和 SIM judge。对 Ackermann、叉车、带托盘或机械臂伸出的机器人，外接圆会过度保守；只按宽度配置圆又会漏掉转弯扫尾。

应在 robot description 定义 base polygon，并允许关节/载荷状态生成动态 footprint；规划、轨迹扫掠、最终安全层和评分使用同一几何约定。为性能可缓存 orientation bins 的栅格 mask，但安全检查不能退化为中心点加不正确半径。

### P1-3 轨迹执行超差只有诊断，没有故障闭环

`TrajectoryTrackingSystem` 比较最终发布命令与后续 ROBOT 估计，但报告明确不 veto motion。持续超差、actuator stuck、runaway 或 command-vs-feedback 不一致不会 latch fault、停车或触发重规划。

应将持续时间、超差趋势和可达包络判断下沉为独立执行监控模块；瞬时超差只诊断，连续不可达或反向运动进入故障状态，发布软停车或紧急停车，并要求明确恢复条件。Coordinator 只串联结果。

### P1-4 协方差模型仍过于简化；REP-105 状态分层已修复

`FusedOdometryEstimator` 仍只有标量 position sigma 与 heading sigma，没有完整状态 covariance、IMU bias、innovation covariance、NIS/NEES。

**REP-105 修复状态：已完成。** estimator 现在分别持有连续 odom 状态与允许绝对定位校正的 map 状态；wheel/IMU prediction 同时推进二者，`correct_pose`、relative SLAM correction 和 operator seed 只改变 map 状态。Coordinator 只发布 `odom→base_link` 以及由两状态计算的持久 `map→odom`，不再用单 tick 的“校正前/后”临时差值。RobotDescription 中的 `world→base_link` 只作为根标记，不进入静态 TF，避免与动态链形成冲突环。回归覆盖 map jump 前后 odom 连续、`map→odom` 跳变和组合后的 map pose 一致。

**相关观测修复状态：已完成。** SLAM-only 的 scan-to-submap tracking
来自当前 keyframe chain，与里程计 prior 相关；它现在只通过 relative
correction 改善局部一致性，不再把每帧 tracking 当作独立绝对地图观测而
重复压小全局协方差。只有经过 supervisor 接受的 surveyed-map/global match
或 pose-graph loop constraint 才能增加独立全局信息。模块回归实际触发一次
mapless tracking correction，并断言位姿可修正而绝对 position sigma 不下降。

执行跟踪也只观察连续 odom 状态，避免定位修正被误判为底盘失控。2026-10-25 验证结果：Debug 与 Release 均 59/59 通过，ROBOT/SIM 边界检查通过；异步 `signal_town` 在 984 步完成，行驶 20.68 m、0 碰撞、最大定位误差 0.285 m，robot tick p99/max 为 12.353/13.382 ms。

剩余修复是至少估计陀螺 bias 和二维相关 covariance，对每种 measurement 做 timestamp、frame、innovation/NIS gate。增加长走廊、重复纹理、稀疏特征、玻璃/缺测和异步传感器测试，并验证 covariance consistency，而不只看离线真值最大误差。

### P1-5 动态障碍在 raw scan 与预测层中重复表示

匿名动态轨迹来自连续 LiDAR 跟踪，但当前 scan endpoints 又作为静态障碍覆盖整个 rollout，动态对象当前位置可能与预测轨迹双重计费，造成空旷处瞬时无解或长时间等待。

应保留 raw scan 作为低延迟最终安全证据，同时在较长 horizon 的局部规划中将可信 foreground 从静态层剥离并由带不确定度的轨迹表示；跟踪不确定或刚出现的点继续按静态/未知保守处理。分层规则必须由 perception/planning 模块拥有。

**修复状态：已完成静态/动态权限切换。** 未知或与先验不一致的回波先由 raw scan 与滚动占据层保守处理，只有覆盖完整拟合时间窗且连续超过运动门限的匿名目标才进入动态预测；持续停止的目标按累计观测时间退出预测并回到静态层。`safety_unknown_obstacle` 回归证明未报告箱体能够被绕开并完成任务，零碰撞且最小净空 0.230 m。仍需在多目标遮挡、交叉换轨和动态目标贴近静态墙面的基准中继续量化关联质量。

### P1-6 TEB 是项目自研 TEB-style，不是完整 TEB

当前实现包含位置梯度平滑、简单时间调整、三条横向候选、动态预测代价和 warm start，但没有完整图优化后端、同伦类别搜索、系统的时间最优目标及完整非完整/加速度/jerk 约束。文档和界面应使用 `TEB-style`，并以成功率、最小净空、jerk、CPU 和动态交互指标与 DWA 对比；在结果优于 DWA 前不切默认。

## 5. SIM 真实性缺口

### P1-7 默认 profile 仍近似理想传感器

IMU noise/bias、传输 latency/dropout 默认是零；LiDAR 默认 72 beams、10 m。虽然设备真实性模型已有代码，`SimConfig` 没有完整暴露与选择经过标定的设备 profile。默认同步 20 ms 还掩盖了 P0-1。

应提供至少 `ideal`、`nominal`、`adverse` 三类显式 profile；发布测试使用 nominal，故障测试使用 adverse。配置应校验 `format`、未知字段和物理范围。传感器真值只在 provider 内生成 measurement，ROBOT 只能看到带来源、时间和噪声的观测。

### P1-8 叉车和行人是 kinematic actor，没有双向接触动力学

actor 目前按脚本直接更新位置，没有对应 MuJoCo dynamic body。它们适合确定性对抗障碍测试，却不能验证撞击后机器人/叉车/行人的反作用、质量和摩擦。渲染模型位置正确也不等于动力学真实。

应按场景目的区分 kinematic test actor 与 dynamic physical actor。dynamic actor 使用质量、惯量、接触几何、轮/足支撑及控制器，碰撞后由引擎决定状态；脚本只给目标或控制输入。评分读取引擎 truth，但 ROBOT 仍只能通过传感器看到 actor。

### P1-9 visual mesh 与 collision geometry 缺少可审查的一致性

GLB 是视觉资产，不会自动成为碰撞 mesh；环境碰撞主要来自 box、heightfield 和 cylinder primitive。应在 viewer 增加 collision geometry/COM/contact/axis overlay，并建立静置接地、轮心高度、关节轴、悬空部件、mesh-to-collision 偏差和 timestep sensitivity 测试。模型由配置装配，渲染从引擎 body/joint pose 读取，禁止用视觉补偿隐藏错误物理姿态。

### P1-10 Mecanum 与被动轮接触模型需明确能力边界

Mecanum 当前使用 body wrench 简化，没有显式滚子刚体与逐滚子接触，不能验证单滚子失效或局部打滑。冠状被动轮可以降低横向擦地，但需要用真实接触参数和自由关节验证，不应仅在 mesh 上做凸起。应增加静置漂移、直行偏航、原地旋转、横移、单轮低摩擦、接触收敛和能量增长测试。

**2026-10-26 修复进展。** 已删除按电机力直接注入车体的开环 wrench。四轮联合 box-QP 在每个固定 1 ms 量子内，根据当前 MuJoCo 法向接触、逐轮材质摩擦和接触点 Jacobian 求 45° 滚子冲量，并向轮轴与车体施加功率闭合的等大反作用；冲量总预算不超过实测载荷转移下的整车支撑重量。Mecanum 承重圆柱使用 `condim=1`，只参加法向接触，避免把零摩擦的退化切向维度送入求解器。四轮/四模块底盘不再继承差速底盘的默认 caster pair，修复了六点刚性支撑导致的周期弹跳。模块测试现覆盖静置无漂移、四轮接地、横移、原地旋转自由速度上界和完整多底盘任务。显式滚子刚体、单滚子失效与逐滚子接触仍属于后续高保真层，界面必须继续标明这项边界。

### P2 真实性扩展

相机/视觉、轮胎温度与磨损、电机热衰减、悬挂和柔性接触应按产品范围逐项加入。它们不阻止纯 LiDAR 移动底盘测试发布，但加入后必须进入真实 sensor/actuator contract，不能把 engine state 直接发布给 ROBOT。

## 6. `main` 能力迁移矩阵

| `main` 中有价值的能力 | 当前分支状态 | 新框架迁移要求 |
|---|---|---|
| 每类 measurement 自带 header | 已按新 envelope 接口恢复 | scan/motion/IMU 独立排序、缓存、freshness 和 replay，不恢复旧 composition root |
| 多目标 Executive、pause/resume/cancel/fail、dwell、arrival speed、abandon | 当前只有单 active goal，TaskOrderingRegistry 未接入 | 下沉目标生命周期与排序模块，Coordinator 只调度 |
| native channel/runtime/replay | 只有部分接口和 LiDAR seam | 复用新 RobotApplication 契约补齐 provider 与 watchdog |
| localization supervisor/error handling | 已有部分 innovation gate 和分片恢复 | 补完整 covariance、map/odom 分层与持续故障状态 |
| path smoothing | 已迁移 | 保持规划模块所有权和 collision revalidation |
| rich sensor realism profile | 底层能力部分存在，配置默认未启用 | 配置化 nominal/adverse profile 并纳入发布测试 |
| typed actuator channels | drive/steer/service-brake 命令已合并，执行反馈待补 | native 与 SIM 继续使用同一 actuator contract，补 applied/fault feedback |
| serial manipulator kinematics、4 关节 PD、重力补偿、MuJoCo arm | 未迁移 | 先建立通用 joint description/command/feedback，再迁移机械臂；所有 pose 来自引擎 joint state |
| 机械臂验收：闭环误差、FK/tool 对齐、失力下落、墙接触 | 未迁移 | 作为 manipulator 模块与物理 provider 的强制验收集 |
| 地图 package/runtime provider | 部分被新 MapSnapshot 取代 | 迁移部署加载、版本、frame 和来源，不迁移 SIM-only sidecar authority |

旧版 authored conflict zone、actor 身份、未来脚本、接触和材质属于 SIM authority，不得迁入 ROBOT。若真实部署通过地图、V2X 或感知获得同类信息，必须作为有来源和时间戳的 ROBOT measurement 独立接入。

## 7. 官方最佳实践对照

本轮所有外部页面均在 2026-10-25 通过命令级代理 `http://gfw.gs-robot.cn:6780` 访问，没有设置全局代理。

- [Nav2 Costmap 2D](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/costmap_2d/)：局部/全局 costmap 是分层环境表示；本项目已有统一 costmap，但需要解决动态前景重复表示。
- [Nav2 Footprint Setup](https://docs.nav2.org/rolling/configuration_and_development/first_time_robot_setup_guide/footprint/setup_footprint/)：非圆机器人局部规划应使用真实 polygon footprint，并可随机械臂或载荷外形更新；对应 P1-2。
- [Nav2 Inflation Layer](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/costmap_plugins/inflation/)：障碍周围使用指数衰减代价，inscribed radius 内为 lethal；当前两阶段 costmap 方向一致。
- [Nav2 Collision Monitor](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/collision_monitor/configuring_collision_monitor_node/)：独立安全层绕过 costmap/规划器直接处理新鲜传感器，支持 stop/slowdown/limit/approach 和随速度变化的 polygon；当前 raw-scan Safety 方向正确，但需独立 section freshness 与动态 footprint。
- [Nav2 Velocity Smoother](https://docs.nav2.org/rolling/configuration_and_development/configuring_velocity_smoother/)：速度、加速度、减速度、死区和高频插值应显式限制；闭环模式依赖高频低延迟 odometry。当前需把低 grip 与真实制动能力纳入限制。
- [Nav2 Smac Hybrid-A*](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/smac/smac_hybrid/configuring_smac_hybrid/)：支持 costmap downsampling、最大规划迭代、障碍启发式缓存、解析扩展、代价感知和平滑；同目标 heuristic 缓存可显著加速重规划。该现行 URL 已在 2026-10-25 通过官方迁移表定位并验证为 HTTP 200。对应 P0-2 的 ROI/coarse-to-fine/cache/time budget。
- [Autoware Localization Error Monitor](https://autowarefoundation.github.io/autoware_universe/main/sensing/autoware_localization_error_monitor/)：用 covariance confidence ellipse 监控定位误差，横向误差尤其关键；对应 P1-4。
- [Autoware Pose Instability Detector](https://autowarefoundation.github.io/autoware_universe/main/localization/autoware_pose_instability_detector/)：把估计位姿变化与车辆运动学可达范围比较；对应持续定位异常检测。
- [Autoware Planning Validator](https://autowarefoundation.github.io/autoware_universe/main/planning/planning_validator/autoware_planning_validator/)：控制前独立检查轨迹延迟、形状和碰撞，并通过连续异常计数、最后有效轨迹或软停车处理故障；对应 P1-3。
- [Autoware Obstacle Collision Checker](https://autowarefoundation.github.io/autoware_universe/main/planning/planning_validator/autoware_obstacle_collision_checker/)：结合预测轨迹、点云、延迟、footprint margin 和最大减速度检查碰撞；对应动态预测与制动契约。
- [REP-105](https://www.ros.org/reps/rep-0105.html)：`odom` 连续但允许漂移，`map` 长期稳定但允许离散跳变；对应 P1-4 的双层状态。
- [robot_localization](https://docs.ros.org/en/rolling/p/robot_localization/)：支持多源、逐传感器配置、15 维状态和连续预测；当前估计器明显更简化。
- [SLAM Toolbox ROS 文档](https://docs.ros.org/en/ros2_packages/jazzy/api/slam_toolbox/) 与[官方 README](https://github.com/SteveMacenski/slam_toolbox/blob/ros2/README.md)：明确区分 mapping/localization/lifelong 模式；scan matching 先修正 odometry，PosedScan 进入 pose graph，再由回环约束优化并发布 `map→odom`。Localization mode 加载已有 serialized pose graph，只保留 recent-scan rolling buffer，过期 scan 会移除且不修改底图。当前实现据此保持 `map/odom` 权限分层，并采用多视角确认；相关的 scan-to-current-submap tracking 只能改善局部位姿，不能按帧重复制造全局置信度。不会照搬参数，也不会把 tracking 搜索窗错误扩成“最高速度 × 周期”。搜索窗围绕 odometry prior 覆盖预期估计误差，运动包络由里程计传播、协方差和速度安全层分别负责。README 还报告其同步模式曾用于约 20 万平方英尺场地，这只作为规模参考，不替代本项目 20 万平方米发布测试。
- [MuJoCo Modeling](https://mujoco.readthedocs.io/en/stable/modeling.html) 与 [Computation](https://mujoco.readthedocs.io/en/stable/computation/index.html)：接触 `solref` time constant 应相对 timestep 足够大；打滑问题可评估 elliptic friction cone、较大 `impratio`、Newton solver、小 tolerance，必要时使用 NoSlip 后处理。当前 1 ms substep 与默认 20 ms contact time constant 满足基本比例，但仍缺参数敏感性矩阵。
- [Gazebo Sensors](https://gazebosim.org/docs/harmonic/sensors/) 与 [SDFormat Sensor Spec](https://sdformat.org/spec?ver=1.12&elem=sensor)：传感器 update rate、射线数量、角/距离分辨率、量程和噪声应是明确设备属性；对应 P0-1 和 P1-7。

这些资料用于约束接口和验收，不用于照搬 ROS/Gazebo 架构；实际实现继续遵循本仓库的新框架和真值隔离规则。

## 8. 分阶段完成路线

1. 已完成：独立 measurement header、ROBOT 分类型缓存和 freshness；真实异步 cadence 回归通过。
2. 增加 grip-aware braking，再把 service/emergency brake 纳入 actuator contract，并用 MuJoCo 实测停止距离校准。
3. 将持续 tracking/localization 异常接入独立故障状态与最小风险停车。
4. 重构 Ackermann 为 ROI/coarse-to-fine、可缓存、受预算约束的 Hybrid-A*，完成 20 万平方米 Ackermann 长测。
5. 引入 polygon/dynamic footprint，并统一规划、Validator、Safety 与评分的 swept geometry。
6. 启用 nominal/adverse 传感器 profile，增加 dynamic rigid actors、collision overlay 和接触敏感性验证。
7. 补齐 native runtime；随后按通用 joint contract 迁移机械臂和旧版物理验收。
8. 接入多目标 Executive 生命周期和排序，完成多底盘、多传感器故障、长时间性能与确定性发布矩阵。

每阶段必须包含模块级测试、相关场景测试、Release CTest 和 `tests/check_boundary.py`。场景失败若反映真实物理或算法能力不足，应保留失败并修正算法或明确产品边界，不能按关卡特化参数，也不能用 SIM 真值帮助 ROBOT 过关。
