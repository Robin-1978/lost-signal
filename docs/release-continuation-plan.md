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

## 当前执行队列（2026-10-30）

本节是恢复时唯一有效的执行顺序；下面的阶段记录保留历史证据，不再作为待办清单。

### 已完成基线

- 1–4：无图 SLAM 回环与 2.5 m 教学路线、定位/算法/场景回归、Release 性能与确定性。
- 5 的基础：EpisodePair、独立 ROBOT/SIM safety/Mission/Experiment verdict、三层视图、首屏预测、章节目录和手动 sandbox。
- 6 的基础：时序 LiDAR、ROBOT costmap 与 safety swept envelope、定位 ghost，以及由 contact/slip/load/brake/impact 证据驱动的粒子和声音。
- 7 的基础：GLB/Blender/MuJoCo body-joint/contact/COM 渲染、轮/叉车/行人姿态和连续地形入口。
- 8–9 的软件契约：VirtualMcu、native transport/HIL seam、watchdog、校准 artifact、CSV 导入、online rollback 和 nominal commissioning gate。

### 当前软件收尾顺序

1. **F4 传感器 realism profile**：已完成，见下方阶段记录。
2. **F3 物理/渲染自动验收**：已完成轮心接地、joint axis、COM/contact、collision geometry 和固定子步 timestep-sensitivity gate；资产 grounding point 现在必须落在声明 bounds 内。
3. **D2–D4 视觉发布债务**：D2 地面材质、D3 音频清理/登记与 D4 模型来源审计已完成；继续进入 E1/E2 教学内容。
4. **E1/E2/C4 教学内容**：E1 的轮胎、LiDAR、算力装配 registry 已完成并接入 assembly_lab；E2 本地 profile/星级/知识卡存储第一切已完成，继续接 Viewer 的章节进度和知识卡界面以及四底盘/算法差异课程。成对 replay/report 导出基础已存在，后续补教学存档界面；扩展 typed experiment registry 时不得把 SIM truth 暴露给 ROBOT。
5. **动态 actor/机械臂课程扩展**：动态 actor 横穿教学章及其匿名 track/预测/让行回归已完成；机械臂课程第一切已完成：`arm_lab` 使用同一 `ScenarioSession`/ROBOT seam，四个 link 的 GLB 以 MuJoCo joint frame 为原点、沿配置的局部 +X 装配，工程视图可检查关节轴、碰撞体、COM、接触和 encoder feedback。新增的 `arm_experience` 回归证明 operator effort 先经过 ROBOT 校验/watchdog 再进入物理关节，清除 effort 后由重力/阻尼自然收敛；SIM 真值没有进入 ROBOT。后续只扩展真实抓取/负载课程，不另造 draw-time 姿态或关卡特化算法。

### 实机外部验收队列

板卡、急停、驱动负载、通信时延、真实 calibration artifact 和温度/载荷范围必须使用同一 native/HIL 契约现场测量；没有实板数据时不以模拟数字宣称完成。完成后再将 profile、HIL 记录和现场报告纳入 Release gate。

### 每阶段完成条件

每一项都要：实现代码和单元测试 → 运行对应 Debug/Release 定向回归 → 更新本文件的阶段记录 → 单独提交。最终再运行 Debug 全量 CTest、Release smoke、truth-isolation/boundary、`signal_town`、20 万平方米、四底盘和 Viewer/headless 性能门禁。

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

8. **MCU 确定性控制域与真机适配**
 - MCU 是独立固定周期时钟域；SIM 默认在 ScenarioSession 的 sim-time deadline 上执行，不让墙钟决定控制结果。
 - 增加 typed motion ownership、capability 和 setpoint，明确 host motion loop、MCU joint loop、MCU body loop 中只有一层拥有外层控制环。
 - 硬件适配器通过 INativeAdapter 接入；MCU 固件不伪装成 IProvider，必要的单设备 wrapper 才实现 provider 端点。
 - 真机与 SIM 共用命令 expiry、session/sequence、watchdog、机械停车、测量时间戳和故障反馈。
 - 按契约、SIM 虚拟 MCU、native fake transport、HIL、实测硬件 profile 五个子阶段推进。详细设计见 docs/mcu-control-domain-design.md。

9. **自动标定与硬件 profile**
 - 离线标定：从真实传感器/编码器记录和可复现实验估计轮径、轮距、轴距、轮速比例、
 IMU bias/尺度、LiDAR 外参与时间偏移、电机 feedforward/PID 参数；输出带 robot
 description hash、输入 digest、版本、协方差和适用温度/载荷范围的 calibration artifact。
 - 在线标定：只允许对轮速比例、IMU bias、低速 slip factor 和执行延迟等有测量证据的
 参数做有界小步更新；每个更新需要残差改善、持久化窗口、健康门禁和可回滚 checkpoint。
 - 安全 footprint、碰撞膨胀、到达条件、planner 搜索规则和任务阈值不得在线自适应；
 calibration 不能读取 SIM truth、actor identity、score 或未来脚本。
 - SIM 只用于产生带噪声/延迟的测量和验证 artifact 回放；真机启动必须完成 profile、
 ownership、单位、版本和 calibration range handshake，超出适用范围保持 safe hold。

## 当前恢复点

### 工作区检查点（2026-10-28）

- 无图 SLAM 增加了独立的 scan-to-scan 运动证据：预测只积分两帧原始轮速/IMU twist，几何位移只来自 ROBOT 自身 LiDAR 的双向 scan matcher；不会读取 MuJoCo 接触、摩擦、真值位姿、actor 或 score。
- 运动证据使用 typed `SlamConfig::motion_evidence_sigma`、既有 matcher gate 和 `track_step_m` 栅格分辨率。匹配协方差至少覆盖一个搜索栅格，避免把占据栅格量化误判为滑移；方向不一致、重叠不足、分数边际不足或双向验证失败都会拒绝测量。
- 只有 mapless SLAM 使用该相对运动证据更新轮速 motion scale；已有先验地图的绝对 scan-to-map 链不再重复消费相关 scan-to-submap 结果。motion scale 通过有界标量 Kalman 融合，仍受 typed min/max 限制。
- L 图层显示 MuJoCo LiDAR 物理传感器起点和真实 raycast；E 图层显示 ROBOT 估计传感器起点与估计投影。两者不再混用，坡脚物理车不会因估计漂移而显示成已在坡上。
- Fast/Turbo/direct 3D 路径移除无 shadow-map 时的假地面圆盘阴影；高质量路径继续使用物理 shadow map。
- 回归：`test_slam`、`test_autonomy`、`test_localization`、`test_viewer_frame`、`test_experience_mission`、`test_slam_experience_mission` 已通过；无图路线首次回环在 east/north/west 三个目标完成后的真实重访阶段，2.5 m 路线 4/4、零碰撞。

- 执行跟踪故障闭环已下沉到 `autonomy/diagnostics/execution_monitor.*`：它只比较 ROBOT 已接受的 body command 与连续 odom 反馈，使用机器人 arrival policy 的速度观测带、已声明的 actuator response delay 和 `motion.execution_fault_confirmation_s`；持续无响应、反向运动、停车后失控运动或反馈过期会锁存 safe stop，单帧异常不会触发。恢复必须由外部停稳后显式调用 `acknowledge_execution_recovery()`，规划器和新目标不会自动清除故障。
- 该监控同时接入 SIM、replay 和 native runtime 的同一 `RobotApplication`，诊断输出为 `exec=...`；没有读取 MuJoCo contact、摩擦、碰撞、actor 或 score。新增 `test_execution_monitor` 覆盖响应、确认窗口、锁存、反向、失控运动、反馈丢失和 yaw 方向，`test_robot_config` 校验机器描述字段。

- 已完成基线：`f22acc9 feat: make simulation experiments evidence driven`。
- `signal_town` Release 基线：5 个目标、0 碰撞、RTF 6.34x、ROBOT tick p99 6.432 ms、最大 tick 14.146 ms。
- 当前阶段：F3 物理/渲染自动验收已完成，下一阶段为 D2–D4 视觉发布债务；第 1–9 项的主要软件交付已独立提交，不得重新实现或回退已经验收的边界。
  `signal_town_big` 的地图目标安全契约已修复并在完整五目标回归中验收：
  `bb87c91`，5286 步、5/5、0 碰撞。直接使用 `sim_viewer --map` 会运行该地图
  的全部 named places，不再只验证首个目标。
- 第 8.1 项已加入 chassis 专属 `ChassisControlOwner`、`ChassisCapabilities`、
 typed `ChassisSetpoint` 及边界校验（保留 Motion* 源兼容别名）；
 `test_motion_contract` 覆盖三种 ownership 与 duty/joint/body 单位隔离，
 `actuation` 和 `robot_boundary` 回归通过。8.2 已接入 SIM 虚拟 MCU 的
 sim-time deadline 调度、transport queue 与本地 watchdog；`virtual_mcu`、
 `signal_town_mission` 回归通过。8.3 已增加 `IChassisMcuTransport` 真机 seam
 和 `examples/stm32_chassis_mcu` 的固定周期 PID reference/CMake host test。
- 8.4 已增加 `examples/serial_sensor_adapter`：直接主机串口接入 LiDAR/IMU/编码器，
 具备增量拆包/粘包、CRC、boot/sequence/acquisition timestamp/finite-value 校验、
 POSIX 非阻塞串口边界和 `ObservationAdapter`；同时 `stm32_chassis_mcu` 增加了
 主机↔STM32 UART framing/CRC codec。两者都不把传感器强制挂到 STM32；真实板卡仍需
 补 UART DMA/中断、底盘 IO 和实测 profile，未经这些 profile 不允许退出 safe hold。
- 8.5 已把通用 native transport 下沉到 `src/robot/transport`：UDP、SocketCAN
（含有限 CAN 分片重组）、spidev 和 USB CDC/串口复用边界，并用 loopback/分片/乱序
 单测证明不会把半帧测量送入 ROBOT。板级驱动和设备树仍留在 adapter/profile 层。
- 8.6 已把 STM32 示例补成 `FirmwareLoop`：UART/DMA 接收与固定硬件 timer tick
 分离，编码器反馈进入底层控制器，MotorOutput 以完整 CRC frame 回传；host test
 与 `STM32_FIRMWARE=ON` 构建均通过。它仍需要具体 CubeMX 的 startup、IO、急停和
 实测 profile，不能冒充通用可烧录镜像。
- 第 9 项的第一切已落地到 `robot/platform/calibration.*`：版本化 artifact
 schema、机器人 description/profile pin、输入 digest、稳健 median wheel-scale
 拟合、静止 IMU bias 拟合、JSON round-trip，以及带确认窗口、步长/范围限制和
 checkpoint rollback 的在线更新。`test_calibration` 覆盖坏值、摘要保留和回滚。
  真实记录文件现在可通过部署无关的 CSV reader 导入，再由 calibration artifact 与
 `INativeAdapter::validate_startup_profile()` 接入；具体板卡仍必须提供实测 profile，
 不用运行时猜测值替代标定结果。
- 第 9 项的 profile 生命周期已补齐：artifact 现在可带温度/载荷适用范围和创建时间，
  `validate_calibration_profile()` 严格校验 robot description hash、profile revision、
  温度/载荷和显式最大年龄；缺少创建时间、超期或身份不匹配都会 fail closed。原子
  same-directory rename 持久化避免半写文件成为活动 profile；HIL online update 可写
  append-only JSONL journal，并在读取时校验 entry schema 与单调 sequence。新增
  `test_calibration` 覆盖 atomic save/load、profile mismatch/stale、temperature/load
  gate 和 journal round-trip。校验仍只使用实测输入，不读取 SIM truth、actor 或 score。

- 8.7 已把真实硬件 handshake 变成 `INativeAdapter::validate_startup_profile()`
  的显式 seam。`NativeRuntime` 在注册 endpoint、配置 ROBOT 或启动 watchdog
  之前调用它；UART、CAN、USB 或 SPI adapter 可以在这里校验 robot description
  hash、profile/revision、ownership、单位、关节布局和 calibration artifact。拒绝时
  不会注册 provider，也不会产生 actuator output。`test_native_runtime` 覆盖拒绝和
  正常路径，保持 SIM/ROBOT 边界门禁通过。
- 8.8 已把 `ProviderConformanceMonitor` 接到 `NativeRuntime`：启动时注册 endpoint
  期望，运行时检查结构、来源/frame、序列、采集时间、速率和静默，并把失败计数
  暴露到 native stats；适配器可声明每个 endpoint 的 nominal rate。出现 stream
  违规时 NativeRuntime 在下一次 ROBOT tick 前执行机械停车并 fail closed。监控只读取
  provider 测量，不改变 ROBOT 估计或引入 SIM 真值。
- 8.9 已把 typed chassis MCU 反馈闭环设为运动输出的前置条件：反馈必须来自已注册
  native encoder endpoint，frame、sequence、时间新鲜度和 drive/steer joint layout
  必须匹配 board capabilities；首次反馈缺失、profile liveness window 超时、重放/回退
  sequence、过期数据和 source/layout 不匹配都会在 ROBOT tick 前机械停车。反馈活性窗口
  由 typed `feedback_period_ns`、host decision period 和 `maximum_command_age_ns`
  推导，复用三周期 liveness 语义，不增加关卡阈值。MCU transport 与 adapter 的独立
 mechanical stop 都会在发送失败、反馈故障和 watchdog 触发时执行。`test_native_runtime`
 新增 host/joint/body 三种 ownership 的合法闭环，以及超时、重放、布局、source、过期和发送失败覆盖；`native_runtime`、
  `virtual_mcu`、`motion_contract`、`conformance` 定向回归通过。

 - Sandbox 手动控制已接入同一 RobotApplication/ScenarioSession seam：Viewer 的
 `H` + `WASD`/方向键只发布限幅的 body command，仍经过 ROBOT validator、raw-scan
 Safety、watchdog 与 MuJoCo/VirtualMcu；experience 运行期禁用。新增场景回归证明
 没有 map-frame prior 时也能安全驱动，清除输入后会减速停车。后续手动控制的输入
 录制仍必须保持为 observation/command 证据，不能写入 SIM 真值。
- 手动 body command 现在按“刷新代数 + 现有 command watchdog”失效：调用方停止刷新
 时，ROBOT 在 watchdog 窗口后自动清除 operator source；这避免了 UI/真机上层异常
 时持续保持最后一个速度，同时不增加独立的场景阈值。场景单测覆盖显式清除和自然
 过期两条路径。
- Viewer 能力目录已补齐：`--list-worlds` 列出全部世界的 places、箱体、连续地形、
 actor sidecar 和 scene 状态，`--list-experiences` 列出所有可加载教学/工程章节。
 新增 Signal Town 五目标、工业 20 万平方米和连续坡/低抓地三个 manifest；它们复用
 既有 Runtime，不增加关卡专用算法或真值入口，缺少装饰场景的地图明确使用物理几何
 fallback。manifest 回归现在会解析这三个入口并验证 map/scene 引用。
- 机械臂视觉资产与课程已落地：`assets/models/robots/ro6/{turret,upper_arm,forearm,tool}.glb`
  由 `tools/modeling/build_arm_assets.py` 可重复生成。每个 mesh 的 manifest bounds、pivot、
  link binding、材质角色和内容 hash 都经过 Blender 资产审计；上游 link 不再沿用竖直旧装饰
  mesh，而是从关节原点沿 MuJoCo 的局部 +X 延伸。`arm_lab` 的 `arm_experience` 测试覆盖
  四个 live link pose、joint axis、operator effort、clear/settle 和零碰撞终态。
- 场景清单回归随 `arm_lab.scene.json` 扩展到 15 个已配对 authored scene；测试按目录
  枚举并检查每个 world/scene，而不是允许新场景遗漏在固定旧数量之外。
- Viewer 首屏章节标签不再硬编码为 `FIRST CONTACT`；它现在从已加载的
  `ExperienceManifest.experience_id` 生成，直接打开 `arm_lab`、`signal_town` 或自定义
  experience 时不会把玩家误导到第一章。Xvfb 3D `arm_lab --shot 40` 已确认标题、四个
  arm GLB 加载和窗口正常退出。
- 代码清理阶段消除了路线流水线中两个“成功后仍写无效状态”的死赋值，简化 DWA
 非完整分支的恒真条件，并让 JSON 对象查找不产生未使用结构化绑定。`cppcheck`
 对 `src/robot` 的剩余提示均为可读性层面的 `std::find_if/count_if` 建议或 Eigen
 逗号初始化误报，没有发现未使用变量/逻辑错误；A*/DWA/TEB 对比与 boundary 回归
 通过。

### Release 运行门禁（2026-10-27）

- Release headless `signal_town` 两次完整运行产生字节一致的 artifact：5673
 steps、113.460 s sim time、5/5、0 collisions、distance 2.483 m、最大估计误差
 0.213 m；`sha256sum` 两次均为
 `72c0a8d76856fcb759e55b92af3b062896fd85716459b6e10e087350acb106af`。
- Release headless profiling：physics p99 2.163 ms，ROBOT tick p99 6.620 ms、
 max 14.113 ms，total p99 8.044 ms，RTF 5.10×。wall time 只用于报告，不进入
 仿真事件或结果。
- Xvfb Viewer 固定 120 帧（启用倍率工作预算上限后）：高质量 1×实际 1.00×、
 8×实际 4.39×、64×实际 3.93×；Turbo/FAST 实际 2.34×。倍率一致性已通过，
 8×/64×仍受 MuJoCo 物理与单线程 viewer 吞吐限制，不能把请求倍率当作实际
 RTF；后续优化目标是拆分渲染与 sim tick、降低 tick p99，并保留这组基线作为回归。
- Playback 工作预算已按请求倍率增长，并由独立的
 `maximum_rate_scaled_work_budget_ms` 上限控制；不会改变仿真步长、传感器 cadence
 或 ROBOT tick，仍会把超出上限的需求报告为 throttled。需重新采集上述 Viewer
 数值后才能关闭 8×/64×交互 RTF 门禁。
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

- Hybrid-A* 的生产分片现在只按 `expansions_per_cycle` 和显式的确定性
 `max_expansions` 计量工作；已移除按 `steady_clock` 的 segment compute-time
 fail-safe。这样同一 ROBOT 输入在仿真、重放和不同主机负载下不会因为 CPU 速度
 改变可达性或返回结果；真机 wall clock 仍只属于最外层 pacing/watchdog。
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

### D2 地面材质完成检查点（2026-10-30）

- 所有带 `.scene.json` 的世界现在声明 `floor.material_id`；`paved`、`gravel`、`mud`、`sand` 和水面材质映射到真实环境纹理，`ice`、`industrial_gravel`、`weathered_asphalt` 等物理材质使用明确的展示别名。没有对应贴图时仍回退到场景声明的颜色，不改变物理接触或 ROBOT 输入。
- 3D 地面贴图使用 shader 的世界坐标平铺（每两米一格纹理密度），不会随 20 万平方米地图的地面代理被拉伸成单张图；普通 GLB 继续使用其自身 UV。该 uniform 在每次地面绘制后清零，阴影 pass 与正常 pass 共用同一渲染路径。
- `test_scene_manifest` 覆盖空材质 ID 的 fail-closed 解析，`mujoco_backend`、`realism`、`viewer_frame`、`asset_manifest`、`scene_manifest` 定向回归通过；Xvfb Release viewer 生成 signal_town 3D 截图并确认 shader 编译成功。该阶段未改变 SIM/ROBOT 边界。
- D4 `NOASSERTION` 资产清理仍未宣称完成；在取得可核验来源或重新生成资产前，发布门禁必须继续拒绝未声明来源的模型。

### D3 音频完成检查点（2026-10-30）

- 删除了未被 Viewer 加载的五个旧事件音和孤立 `cues.json`；当前 WAV 库与 `ViewerAudio` 的九个加载端点一一对应，并由 `tests/test_audio_assets` 检查死文件不会回流。
- 增加 `assets/audio/manifest.json` 与 `LICENSES.md`。机械声仍只消费完成 SIM step 的接触、打滑、负载、制动和冲量证据；新增的 `environment_bed.wav` 是低增益、确定性、无第三方录音的中性环境循环，不会进入 ROBOT、score 或物理输入。
- 环境循环与四条机械循环共用暂停、回退、Turbo 和无音频设备的停止策略；`environment_loop_gain` 是 Viewer 配置项并受范围校验，不参与任务阈值。`physical_audio`、`viewer_audio` 和 `audio_assets` 定向回归通过后，单独提交本阶段。
- D4 模型来源审计已完成：39 个 legacy-main 条目均有仓库提交证据，根 `LICENSE`/`NOTICE` 已恢复，manifest 中 `NOASSERTION` 为 0；原始 `legacy-main:*` source artifact 保留，后续若发现外部来源必须替换为上游许可证。

### D4 模型来源完成检查点（2026-10-30）

- `assets/models/PROVENANCE.md` 记录审计范围、提交作者、保留的 legacy source artifact 和替换规则；`assets/models/LICENSES.md` 说明 Apache-2.0、CC0-1.0 与项目生成资产的适用关系。
- 根 `LICENSE`、`NOTICE` 和 `THIRD_PARTY_NOTICES.md` 恢复到当前分支，模型 manifest 的 39 个历史 `NOASSERTION` 已依据仓库来源证据登记为 `Apache-2.0`，没有把任何未知第三方 URL 或许可证凭空填入。
- `test_asset_manifest` 现在 fail closed 地拒绝 `NOASSERTION`，并继续验证 manifest 与 GLB/GLTF 库存一致；D2、D3、D4 形成独立视觉发布提交链。

### E1 装配 registry 完成检查点（2026-10-30）

- 新增 `sim/viewer/assembly_registry.*`，把 RO-1 的标准/防滑/节能轮、快/标准/远距 LiDAR 和轻量/性能计算单元固定为 typed registry。每个选择同时声明资产 ID、轮胎摩擦/滚阻、LiDAR 量程/束数/周期和计算决策周期；未知 ID fail closed。
- Experience manifest 可选声明 `assembly`，`assembly_lab` 已用防滑轮、快扫 LiDAR 和性能计算单元接入。Runtime 在首次 observation 前应用选择到 RobotConfig/SIM config，Episode invariant digest 包含三槽位 ID；Viewer 仅据同一选择加载 GLB，不把 SIM truth 或 actor identity 传给 ROBOT。
- `test_experience_manifest`、`test_experience_runtime` 和 `test_experience_catalog` 覆盖 registry 负例、typed 参数、digest 变化与 13 个章节首 tick；现有章节没有 assembly 时保留原始硬件配置。

### E2 进度档案第一切完成检查点（2026-10-30）

- 新增 `sim/viewer/player_profile.*`：本地 JSON profile 只保存章节完成次数、独立 verdict 产生的 0–3 星、公开耗时/净空最佳值和已获得知识卡。星级顺序为 safety → mission → experiment，不能用速度或 SIM truth 越过安全失败。
- profile 首次不存在时从空档案开始；损坏、版本不匹配、重复知识卡或非法数值 fail closed 并回退空档案；保存使用同目录临时文件原子 rename。`test_player_profile` 覆盖星级、去重、round-trip 和损坏回退。
- Viewer 已接入 profile：章节简报显示当前章节星级、完成次数、最佳耗时/净空和知识卡进度；复盘显示已发布的知识卡内容。默认路径遵循 `XDG_STATE_HOME/lost-signal/player-profile.json`（无该变量时为 `~/.local/state/lost-signal/player-profile.json`），也可用 `--profile FILE` 指定。
- profile 只在不可变 artifact 通过独立 progression gate 后发布；A/B 章节必须等 baseline+variant 完成且 Experiment verdict 通过才记一次。profile 路径、星级和知识卡从未进入 ROBOT、SIM 配置、Episode digest 或任务判定。
- 损坏/版本不匹配仍回退空档案并提示；保存继续使用同目录临时文件原子 rename。Viewer 的当前章节进度和知识卡 UI 已接入，完整跨章节目录仍按内容批次继续扩展。
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
- forklift 与 pedestrian 已迁入 MuJoCo mocap body：碰撞几何参与引擎接触，pose/heading/ground height 由 engine read-back 驱动历史与实时 GLB；LiDAR 仍通过匿名 actor 观测路径消费，不把 actor identity 或真值穿透给 ROBOT。配置驱动的四关节 ArmParams、MuJoCo link/joint chain、SIM-only `ArmLinkViz`、RO-6 四个 link-frame GLB 以及 `arm_lab` 机械臂控制/接触验收均已完成；后续只增加真实抓取/负载内容，不回退到 draw-time 姿态。
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
- 下一恢复点：在当前干净工作区完成工业长测与 Release 门禁复测，并继续 E1/E2 的教学存档/章节差异课程；worker 动画、forklift link 拆分、Blender 环境资产和碰撞验收已经是历史完成项。每个新增内容仍必须独立提交。

### 持续恢复锚点（2026-10-27，`d9cebd6`）

- 当前分支为 `refactor/clean-rebuild`，工作区在本锚点应保持干净；恢复时先执行
  `git status --short --branch` 与 `git log -5 --oneline`，确认 HEAD 至少包含
 `d9cebd6 feat: drive worker animation from sim motion`。
- 已完成的机械臂配置、MuJoCo link/joint pose、effort/encoder feedback、回放 v4、动态 actor mocap、GLB link binding、COM/contact/joint-axis overlay 和物理音频均视为已验收边界。恢复工作不得回退这些契约，也不得把 SIM-only 字段加入 `src/robot/`。
- 第 7 阶段的 worker 行人、forklift 分部件和 Blender/GLB 场景资产 manifest 已完成；机械臂第一切也已完成。后续模型扩展必须继续通过同一 backend/frame/viewer 验收，且不得把 SIM-only pose 或 actor identity 加入 `src/robot/`。
- 工业长测的 `4/5` 结果曾在干净工作区复测为 `5/5`，因此只能作为需重复的稳定性证据；后续长测必须保留定位候选、协方差、恢复动作和时序记录，不能用放宽到达条件、读取真值或关卡专用常量掩盖波动。

### 压缩后恢复协议（必须遵守）

1. 读取本文件的“持续恢复锚点”和“最新恢复顺序”，再检查 `git status`/`git log`；不要从对话记忆猜测已完成内容。
2. 从表格中第一个“进行中”阶段的第一个未完成子任务继续。当前入口是 E1/E2 教学存档与四底盘/算法差异课程；完成后更新本文件、记录测试证据并单独提交。
3. 任何失败先记录可复现命令、输入 revision、SIM/ROBOT 边界和结构化证据，再修改代码。已撤销的动态回波过滤、无条件 prior-improvement、post-step velocity projection 等实验不得重新引入。
4. 每次阶段性提交后立即更新本文件中的提交号、测试结果和剩余风险，确保下一次上下文压缩仍能从同一锚点恢复。

### 第 7 阶段 worker 子阶段（2026-10-27）

- worker GLB 的 `WorkerWalk` clip 已接入 Viewer；动画相位由记录帧的 SIM 时间、MuJoCo mocap 根位姿对应的实际速度和 manifest `animation_stride_m` 推导。暂停、回放、Turbo 和多 worker 不共享墙钟相位，停下时回到站立帧。
- `AssetDefinition`/manifest 现在显式声明动画步长；Blender `inspect_assets.py` 对带 clip 的资产强制检查有限的正步长。资产仍保持 `actor_root` 地面 pivot，根位姿直接来自 MuJoCo read-back。
- 验收：`test_asset_manifest`、`test_viewer_frame`、`test_mujoco_backend`、`test_sim_actor` 通过；Blender 5.2.2 `inspect_assets.py` 报 `ASSET_MANIFEST_OK assets=44`；Xvfb 下 `sim_viewer --3d --map configs/worlds/industrial_200k.json --mute` 成功加载 `WorkerWalk (50 frames, 0.833333s)`。
- 本子阶段不改变 ROBOT observation/decision/actuation，也未把 SIM actor identity 或真值加入 `src/robot/`。forklift 的车体、驱动轮、转向轮、门架、carriage 和货叉 link 已绑定真实 MuJoCo body/joint pose；后续只补模型细节和内容，不改变边界。

### 阶段 1 回环修订检查点（2026-10-27）

- 回环异步搜索的 pending proposal 不再按墙钟超时清理；proposal 绑定同一历史 keyframe anchor，只有该 anchor 被有界 keyframe ring 丢弃时才失效。这样不会在 coarse/refine/reverse 分片执行期间丢失第一次真实重访证据。
- coarse retrieval 与 dense refinement 只负责候选关联；reverse association 使用 endpoint occupancy，最终接受仍由不相交的 historical/recent causal submap 的 expected-range 双向证据、pooled overlap、score margin、信息矩阵和同锚点时序确认共同决定。SIM 真值仍只出现在测试诊断。
- `test_loop_closure` 在目标完成后保留固定观测尾窗，验证异步 matcher 的最终状态；首次闭环在完成前三个目标后的真实重访阶段发生。当前 Debug 重复回归：`truth_isolation`、`slam`、`loop_closure`、`slam_experience_mission`、`robot_boundary` 共 5 项各重复 3 次通过。
- 该修订提交后，继续从第 6 项 Viewer/音频独立提交，再进入第 7 项模型与场景升级；不要把工作区中未提交的 Viewer/音频文件混入本阶段提交。

最新恢复顺序：

1. 删除 `stage6-*.png` 临时截图；跑 Debug/Release 全量 CTest、truth isolation、ROBOT boundary 和 `git diff --check`。
2. 将算法、物理求解选择和相关测试单独提交；不得把这些修改与 Viewer/音频提交混为同一阶段提交。
3. 再单独提交 Viewer、物理音频、WAV、文档及其测试；随后立即进入阶段 7。

### 当前修复阶段（2026-10-28）

- surveyed-map recovery 的候选不再把重复的秩亏墙面扫描当成独立绝对证据。每个候选在
  有实际运动视角时累加 `[x,y,yaw]` information；只有累计观测达到 full-rank 并满足既有
  covariance/confirmation 合同才允许 promotion。静止重复扫描不会增加信息。
- 若唯一候选仍只有 rank-deficient 证据且 ROBOT covariance 已越过
  `LocalizationSupervisor` 的既有 recovery 门限，tracker 会释放并交回 covariance-gated
  whole-map search；没有新增关卡阈值，也没有读取 SIM truth。
- `test_localization` 已覆盖 guarded partial promotion 与 full-observation promotion；
  `industrial_200k_mission` 的离线诊断现在同时打印 ROBOT continuous odom estimate，
  但真值仍只在 SIM 诊断/评分侧。
- Debug 最新复测：`industrial_200k_mission --steps=60000 west_hub north_depot central_gate`
  完成 `3/3`、零碰撞；实际在第 44405 步完成第三目标，最终 ROBOT estimate 与物理目标相差
  约 0.1 m。Release 需用本次改动重新构建后再做同样长测，五目标完整结果仍是下一道门禁。
- 本阶段工作区包含 `docs/source-layout.md`，用于固定 `src/robot`、`src/sim`、provider、
  transport、MCU 和 Viewer 的依赖边界；公共 adapter seam 不因静态分析的 unusedFunction
  提示而删除。
- Viewer catalog 现在还包含独立 `ice_lab`：低摩擦冰面由 SIM 的 material/zone 进入
 MuJoCo 接触求解，ROBOT 只能从轮速、IMU、LiDAR 和自身 slip belief 推断响应；体验入口
 与世界校验共用现有 manifest/runtime，不增加关卡专用算法或真值通道。

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

### 当前恢复点（2026-10-28）

- 第 1 项的当前验收已重复确认：`loop_closure` 与 `slam_experience_mission` 各自通过；首次回环仍发生在真实重访阶段，跨历史候选比较和同一 causal anchor 时序确认保持启用。
- Release 工业长测已重新构建并通过：`build-release/test_industrial_200k_mission`，75997 步、5/5、0 碰撞、464.95 m；五个目标的 SIM truth miss 为 0.143/0.193/0.079/0.176/0.146 m，最终 ROBOT estimate 与物理真值只用于离线报告。
- Release 场景回归：`signal_town_big` 五目标 4695 步、5/5、0 碰撞、最大估计误差 0.237 m；`ice_lab` 两目标 1653 步、5/5、0 碰撞、最大估计误差 0.128 m。
- `0752b1a fix: stabilize shadow camera basis` 修正了阴影 texel snapping 与实际阴影相机 up basis 不一致造成的投影漂移；Xvfb 3D evidence 截图和 `viewer_frame`、`scene_manifest`、`experience_manifest`、`playback_controller` 回归通过。
- `dc47513 feat: author town presentation scenes` 为 `signal_town` 与 `signal_town_big` 增加同名 authored scene。世界文件仍是唯一碰撞/物理来源；scene 中无碰撞的装饰只使用 `surface_only`，高度约束由 scene loader 强制检查。Viewer catalog 现在将两个世界标为 `scene=authored`。
- 直接 `sim_viewer --map` 会执行该地图的全部 named places；当前 big town 运行到第 3/5 个目标时仍持续推进，未复现“第一目标后停住”。若 UI 处于 briefing/prediction，必须先完成首屏 walkaround 和 prediction 选择，暂停是教学流程状态，不是仿真卡死。
- 第 7 项的物理驱动模型、环境 GLB、碰撞/COM/joint/contact 工程视图已完成；当前工作转入发布目录门禁和第 8/9 项真机 profile/HIL 收尾。坡面、冰面、复杂场景和手动 sandbox 已在 catalog、Release smoke 与独立任务测试中可见。

### Release 性能复测（2026-10-28）

- `build-release/sim_viewer --sandbox --3d --map configs/worlds/industrial_200k.json --quality fast --shot 120`：1x 实际 `1.00x / 50 steps/s`，ROBOT tick p99/max `19.61/20.72 ms`；8x 实际 `5.16x / 258 steps/s`、`8.07/9.47 ms`；64x 实际 `5.19x / 260 steps/s`、`7.67/9.13 ms`；Turbo 实际 `4.57x / 228 steps/s`、`8.31/11.39 ms`。请求倍率超过单线程 Viewer 吞吐时仍显示实际 RTF/LIMIT，不改变固定步长。
- Release headless `signal_town` 完整五目标、`--profile`：5961 步、5/5、0 碰撞、RTF `5.38x`；physics p99/max `1.674/2.742 ms`，ROBOT tick p99/max `6.273/12.458 ms`，total p99/max `7.645/14.031 ms`。
- 同一 Release 输入连续两次输出 summary JSON 的 SHA-256 均为
  `d9f92110de56170fc8ad2cd85d5efd87ecb37cca0a705f0c4442c27ca6e48b6a`，证明 observation、command、终态和 score 在相同输入下可复现。
| 4. Release 性能与一致性 | 已完成 | `fba2d87` | Release 75/75；1×/8×/64×/Turbo/headless 实测；逐步确定性门禁；发布 signal_town 5/5、0 碰撞 |
| 5. SIM 游戏体验 | 已完成 | `f22acc9` | typed ExperimentSpec；immutable EpisodePair；四类 verdict；10 秒内首次预测；Experience 回归 10/10；Viewer 截图通过 |
| 6. 物理证据可视化与效果 | 已完成 | `cdea986` | 算法/物理与 Viewer/音频完成；Debug 全量 78/78；Release 定向 9/9；物理音频单测覆盖暂停、回退、Turbo、接触、打滑、制动和冲击 |
| 7. 模型与场景升级 | 已完成 | `525116c`, `53bd9cf`, `d9cebd6`, `f2e3a5b`, `e6e17fd`, `904f0cb`, `4e3a75a`, `8fa94cb`, `62d741a` | 机械臂配置、MuJoCo effort/feedback、worker `WorkerWalk`、叉车六分部件 link 绑定、Blender 环境资产、碰撞几何/COM/joint/contact 工程视图和姿态回归均已完成；当前只保留发布后的场景体验扩展 |
| 8. MCU 确定性控制域与真机适配 | native HIL 契约已完成，实板待验 | `b235727` + 既有提交 | UART/CAN/SPI/USB transport、虚拟 MCU、STM32 reference、typed chassis ownership、native profile startup handshake、ROBOT 执行反馈故障闭环和固定时间 HIL replay 已接入；仍需按具体板卡实测 profile/急停/负载验收 |
| 9. 自动标定与 hardware profile | 记录质量/commissioning 门禁已完成，实板 profile 待验 | `0760aa5`, `7dc60b9`, `47695ea` + 既有 calibration 提交 | 离线 median wheel-scale/IMU bias、artifact digest/JSON、在线有界更新、rollback、温度/载荷/过期 gate、CSV 记录导入、乱序/NaN/负残差拒绝和 nominal commissioning profile 已通过单测；来源、digest、样本数和典型默认值一致性已加门禁；具体硬件 profile 仍需真实记录验收 |

### 大目标校对（2026-10-30）

| 目标 | 当前结论 | 证据/边界 |
|---|---|---|
| 1. 无图 SLAM 回环可信度 | 已完成 | 跨历史候选统一比较、同一 causal submap 时序确认；首次回环在 east/north/west 完成后的真实重访；`loop_closure`、`slam_experience_mission` 通过 |
| 2. SLAM 教学体验与文档 | 已完成 | 2.5 m 无先验地图路线 4/4、零碰撞；SLAM Toolbox/REP-105/map-odom 文档已更新 |
| 3. 算法与系统回归 | 已完成 | 当前 Debug 全量 CTest `90/90`，含 signal_town、20 万平方米、四底盘、坡面/冰面、DWA/TEB、定位、回环和 truth isolation |
| 4. Release 性能与一致性 | 已完成 | Release 基线已记录 1×/8×/64×/Turbo/headless 的 RTF、tick p99 和结果 digest；当前新增 platform API 的 Release 定向回归通过 |
| 5. SIM 游戏体验 | 已完成 | EpisodePair、ROBOT/SIM/Mission/Experiment 独立 verdict、教学/工程视图和首屏流程已落地 |
| 6. 物理证据可视化与效果 | 已完成 | LiDAR 时序、Safety swept envelope、costmap/safety 证据层、物理粒子/声音已由真实 contact/slip/load/brake/impact 驱动 |
| 7. 模型与场景升级 | 已完成基线 | GLB/Blender 资产、MuJoCo body/joint/contact/COM 驱动和环境场景已接入；后续新增资产属于内容扩展，不改变 ROBOT 边界 |
| 8. MCU 确定性控制域与真机适配 | 软件契约完成，实板待验 | typed ownership、VirtualMcu、UART/CAN/SPI/USB、STM32 reference、NativeRuntime watchdog、native HIL replay 已完成；仍需具体板卡 profile、急停、负载和传输延迟实测 |
| 9. 自动标定与 hardware profile | 软件契约完成，实板待验 | `7dc60b9`, `47695ea`；measured artifact、CSV 质量门禁、online rollback、profile 聚合验证和 nominal commissioning 已完成；nominal 只能显式用于 bench，生产仍需实测 artifact |

### 阶段 7 叉车 link 子阶段（已完成，`f2e3a5b`）

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
 WorkerWalk 和 3D 渲染。子阶段已提交为 `f2e3a5b feat: bind forklift visuals
 to MuJoCo actor links`；下一步是扩展 Blender 环境资产和最终 Release 视觉/碰撞
 验收。

### 阶段 7 环境资产子阶段（已提交，`e6e17fd`）

- 在现有米制、`+Z` up、`+X` forward 和 world-fixture manifest 契约下新增三类
 可复用 Blender 资产：`traffic_signal`（红黄绿信号灯）、`safety_bollards`
 （带链条的安全导流柱）和 `pallet_stack`（带标签的双颜色货物托盘）。它们的
 支撑面均落在 z=0，材质角色、三角形预算、hash 和 bounds 进入 manifest。
- `research_base.scene.json` 将三类资产作为 `surface_only` 视觉实例放到
 `signal_cross_tower`、叉车通道和 `supply_station`；不创建隐含碰撞体，碰撞世界
 仍只由 world JSON 的 proxy 负责，ROBOT 不读取这些渲染资产。
- Blender `inspect_assets.py` 报 `ASSET_MANIFEST_OK assets=53`；
 `test_asset_manifest`、`test_scene_manifest` 通过。Xvfb 下 3D viewer 成功加载
 三个新 GLB，临时截图已删除；后续新增资产继续使用同一 manifest 和
 collision-proxy 契约。

### 阶段 7 碰撞几何可视化（已修复姿态）

- MuJoCo backend 现在导出每个活动 link 的碰撞 geom（shape、size、world pose、
  body id），容量来自 `model->ngeom`，按实际数量动态分配，没有任何固定上限或
- 魔法数字；静态世界碰撞体则直接使用 `Frame::boxes` 中与引擎一致的 authored
 footprint/height，在同一诊断通道展开为 12 条棱线。`height_m <= 0` 的平面标线
 与退化 footprint 不绘制，避免把视觉 paint 错标成碰撞体。
- `PresentationSnapshot` 保存该表，回放帧和实时帧走同一条 `FrameBody` 路径；
  engineering overlay 用洋红线框绘制 box/capsule/cylinder/sphere 代理，与 COM、
  joint axis、contact 点/法向同屏，可直接核对“碰撞体是否包住视觉模型”。
- 新增 `--physics-overlay` 作为可复现验收开关，与 K 键共用 `sim_truth_debug_allowed`
  门禁，experience 运行期仍禁止显示 SIM 证据。`test_mujoco_backend` 断言导出非空
  且位姿有限；`robot_boundary` 通过，未新增 ROBOT 入口。
- 后续审查发现首版的姿态基错误：中心点已换到 raylib 基，而朝向向量仍是 MuJoCo
 SIM 基，导致直立 actor proxy 横躺；box 也只在八角和画小方块。现已拆出
 `collision_geom_wireframe` 纯模块，在同一 SIM 世界坐标中完成四元数刚体变换后
 一次换基，box 输出 12 条边，capsule/cylinder 输出沿真实局部 Z 的线框。
- 新增 `test_collision_geom_wireframe` 无窗口测试；`signal_town` 与
 `research_base` 的 `--physics-overlay` 3D 截图均显示机器人代理直立并随车体
 运动。静态 box 测试覆盖 footprint 轴、地面到顶面高度、反向坐标归一化、平面
 标线、NaN 和零面积退化输入；`research_base` 截图可见静态障碍棱线贴合碰撞体。
 静态碰撞线框使用独立 `show_physics_overlay` 参数并受 SIM truth 门禁控制；
 shadow-map pass 明确传 `false`，不会把诊断线框烘入阴影。定向回归
 `collision_geom_wireframe`、`viewer_frame`、`mujoco_backend`、
 `scene_manifest`、`world_place_clearance` 通过。
- 叉车渲染路径选择已抽成 `actor_render_policy` 纯策略：完整模块化 link
 资产、完整 actor GLB、程序化 fallback 三者严格互斥；新增 `test_actor_render_policy`
  覆盖行人、模块化资产不完整和模型定义缺失分支，避免程序化圆柱再次覆盖真实叉车。

### 工业动态 actor 回归修复（已完成，`0d3cfec`）

- 20 万平方米全量回归曾在 `central_gate` 发生双方等待：让行行人在机器人
 已经停止后仍坚持包含 clearance margin 的 yield latch，机器人 Safety 又不能
 穿过该匿名障碍，最终 100000 步只完成 2/5。该失败不涉及目标放宽或 SIM 真值
 穿透，且不是视觉资产加载错误。
- `apply_actor_robot_yield()` 现在把 actor/robot 半径之和作为唯一硬接触边界；
 当机器人按声明的单量程加速度已近似静止、actor 连续路径的物理最小距离仍
 大于该边界时，沿已有 reaction time 证据释放额外 clearance 的让行锁存。ROBOT
 仍独立使用自己的 Safety/costmap margin，actor 不会穿过物理接触包络；这只是
 通用 cooperative-deadlock 处理，不读取 ROBOT 估计或场景 ID。
- 新增 `test_actor_behavior` 覆盖“clearance margin 内但物理半径外”的释放，
 保留原有物理接触和 teleport 单量程测试；`test_dynamic_crossing_mission` 通过。
 `industrial_200k_mission central_gate --steps=40000` 现在 14337 步完成、0 碰撞，
 truth miss 0.142 m。完整长测已在定位修复后重新验收，见下节。

### surveyed-map recovery promotion 修复（工作区，待提交）

- 原完整工业长测曾出现 `71223 steps, 4/5`：`south_yard` 走廊的 LiDAR 只观测到
  墙面法向和航向，匹配信息矩阵沿走廊方向秩亏；recovery tracker 却把连续的
  partial wall observation 当成绝对 map pose promotion，ROBOT 估计因此锁在走廊
  别名，SIM 真值落后目标 2.820 m。SIM 真值只用于离线定位失败诊断。
- `StartupHypotheses` 现在支持 recovery-only 的
  `require_full_observation_for_promotion`。partial observation 仍可沿可观测方向
  跟踪，只有 `[x,y,yaw]` information full-rank 且达到既有 confirmation 数，才可
  promotion；策略在 `seed()` 时锁定，途中切换配置不会重解释已经积累的历史证据。
  默认冷启动行为保持不变。
- `LocalizationSystem::reset()` 只为 surveyed-map recovery 打开该策略；ROBOT 不
  读取 SIM truth，未放宽 SIM 到达判定。新增定位单测覆盖 partial 保持 tracking、
  full-rank 连续证据 promotion 和默认冷启动回归。
- 修复后完整 `industrial_200k_mission`：71520 steps、5/5、0 collisions、
  464.89 m；五目标 truth miss 为 0.100/0.154/0.121/0.106/0.061 m，最终
  `goal_hold/arrived/stop`，末端估计与真值差约 0.075 m。`test_localization`、
  `test_slam`、`test_loop_closure` 均通过。

### 当前工作区检查点（2026-10-28）

- Viewer/Experience 入口扩展已在工作区完成：新增 `signal_town` 五目标章节，主战役现在从 First Contact 经过未知障碍、无图闭环、连续地形、Signal Town、Signal Town Big 到 industrial 200k；另外保留 proving ground、orchard hillside、stormglass station 和 washout recovery 工程实验。新增章节仍只通过 `ExperienceRuntime` 建立 world/ROBOT/SIM，未增加关卡特化算法或真值入口。
- `sim_viewer --catalog` 现在明确显示 world 类型、actor sidecar 和 `authored/fallback` scene。没有专属 scene 的 world 会加载 `lab_default.scene.json`，保证 3D 镜头、灯光、粒子和模型绑定不因缺少装饰文件而消失。
- 修复 `test_scene_manifest` 对通用 fallback 误认为缺失 `lab_default.json` 的旧假设；真正成对的 scene/world 仍逐个严格解析。Experience manifest 回归新增章节图校验：所有边必须指向存在的 manifest，主战役不能成环且必须到达 industrial 200k。
- Viewer 的模块化叉车路径不会再在真实 MuJoCo link GLB 外叠加旧程序化圆柱；程序化几何只在没有完整模型/绑定时使用。worker、叉车、轮、caster 和机械臂的渲染继续消费 SIM presentation snapshot 的 body/joint pose，ROBOT boundary 不变。
- 已验证：`test_scene_manifest`、`test_experience_manifest`、`signal_town_mission`、`lost_signal_sim ... signal_town_big ... --require-pass`（4695 steps、5/5、0 collisions）、`test_slam_experience_mission` 和 `test_terrain_mission`。工作区截图和临时文件已清理。

### 当前恢复点（2026-10-28，发布前回归）

- `c72aba9` 修复 Viewer 入口优先级：显式 `--experience`、`--chapter` 或
  `--map` 不会再被 `--play` 静默替换为 First Contact；`15ec297` 让没有必看
  walkaround 部件的章节按 Enter 直接进入预测，避免第二关看起来“点不动”。
- `9484924` 将大地形的 Viewer 网格采样按地图范围有界化（每轴最多 160 个
  可视单元，小地形仍保持 0.5 m 细节），不改变 MuJoCo/测量/ROBOT 地形；
  `2af9dbf` 修复高度场米制高度被二次缩放的问题，并用 raycast 回归锁定接触
  面；`d89e0a8`/`f73ba9e` 修复阴影 FBO 完整性和 depth attachment 重复释放。
- `a89893f` 统一记录 transition 的一基 step 序号，使首个 Episode 记录与
  score/truth 的完成步一致；`0251d98` 明确 Coordinator 对 mapper 的唯一所有权，
  并为 RobotApplication 的证据接口补齐 `override`，没有改变 ROBOT/SIM 边界。
- `ddb135c` 保留高不确定度终端的 chassis-derived 观测动作；`32f97d6` 删除
  发布路径中的 DWA 临时 stderr 诊断。该修复复测 `dwa`、`slam_experience_mission`
  和 `terrain_mission` 均通过；任何会让冰面坡道失败的额外终端阈值实验均已撤回。
- `signal_town_big_mission` 的五目标任务实际在 6329 步完成；测试预算已改为
  直接复用章节的 `time_limit_steps=12000`，任务仍在全部目标完成时提前退出，
  不放宽到达条件。当前 Debug 定向回归（MuJoCo、DWA、SLAM、terrain、Viewer、
  assets、scene、catalog、boundary）通过，85 项非工业全量回归中仅旧的 20 万
  平方米长测需要单独串行采集。
- 每个 catalog world 的 Xvfb 3D smoke 已通过；`signal_town`、
  `signal_town_big`、`industrial_200k`、`orchard_hillside`、
  `proving_ground_map`、`slope_lab`、`stormglass_station` 和 `washout_canyon`
  现在都有同名 authored scene。场景只增加灯光、地面材质和已验证的贴地
  GLB 装饰，碰撞/地形/粒子/物理模型仍由 world 与 MuJoCo 权威驱动。
- 代码整理已完成一项低风险收敛：ObjectiveDriver 在每个 tick 只采样一次
  truth acceptance 数据，再更新稳定窗口，不重复计算同一物理状态；工业、冰面和
  Experience 回归仍通过。下一步是把四底盘与手动 sandbox 纳入同一 Release 发布门禁，
  同时继续清理真正私有的重复 helper，不删除公共 adapter/provider seam。

### 发布目录与代码整理阶段（工作区，待提交）

- `test_experience_catalog` 现在严格检查全部 15 个发布章节：每个章节的
  world、authored scene、asset manifest 和首个 ROBOT/SIM transition 都必须可加载；
  不再使用 `>= 11` 这种会掩盖章节丢失的下限。
- `test_scene_manifest` 严格检查全部 14 个有配对场景的物理 world 都有同名 authored scene；
  `lab_default.scene.json` 仍只作为显式的开发 fallback，不会替代已发布 world 的
  reviewed scene。
- 局部匹配、SLAM 和全局搜索共用 `kMaximumScanMatchHypotheses` 的实现容量；
  搜索策略仍由各自 typed config 决定，不再在三个算法文件中复制同一无名数字。
- Debug 全量 CTest 已重新执行并通过 89/89；20 万平方米长测 269.54 s，直接
  `central_gate` 长测 49.23 s。`signal_town_big` 五目标、冰面、连续地形、四底盘、
  视觉资产、场景 manifest、experience catalog、truth isolation 和 ROBOT boundary
  均通过。
- Release 定向门禁也已通过：`test_native_runtime`（profile handshake、watchdog、
  typed MCU setpoint）、`test_multichassis_mission`（differential/Ackermann/
  Mecanum/swerve）、`test_experience_catalog`、`test_scene_manifest`、
  `test_asset_manifest` 和 `test_truth_isolation`；`python3 tests/check_boundary.py`
  通过。Xvfb 已对全部 14 个有配对场景的物理 world 完成 `--3d --shot` smoke，且每个 world 的
  40-step headless smoke 均能推进。
- Release Viewer 对主战役 7 章和 6 个工程课程逐章执行 `--chapter ... --3d
  --shot 8` smoke，全部成功退出并生成截图；这覆盖了第二章入口的 Enter/briefing
  状态、无图 SLAM、连续坡面、冰面、复杂场、交通场和工业长图，避免把 UI 暂停误报
  为仿真卡死。

### 终态定位与冰面教学回归（2026-10-29）

- DWA 在原始终点位置已被估计捕获、但置信半径仍宽于到达包络时，继续允许可执行的
  航向观测动作；这只改变控制可观测性，不改变 `GoalArrivalPolicy` 的置信包络、稳定
  停车或完成条件。
- 局部扫描匹配被创新门拒绝时，`try_local_correction` 不移动位姿，也不改写协方差，只向
  supervisor 记录一次弱负证据（`record_local(false, match)`）；全局恢复仍只在融合协方差
  自身越过 loss envelope 时开始（`supervisor.cpp:89`）。此前把这类残差经
  `IEstimator::note_absolute_rejection` 回灌协方差的做法（`20aaf74`）已撤回，不得重新
  引入，证据见下节“局部拒绝回灌撤回与制动窗口回归”。已被匹配器或恢复追踪器接受、
  但被协方差创新门拒绝的全局候选/恢复假设仍通过该接口扩大协方差
  （`localization_system.cpp` 两处调用）。
- 冰面教学路线补充了不侵入行驶通道的稀疏几何地标。此前的空旷冰面终点只有不足以
  区分位姿的激光回波，低摩擦原地转向产生的真值漂移属于可解释的不可观测失败；地标
  使路线具备真实可观测条件，未向 ROBOT 注入地形或物理真值。
- `test_dwa`、`test_localization`、`test_autonomy` 和 `test_terrain_mission` 定向回归
  通过；冰面仍验证滑移与抓地力收紧，2500 步、零碰撞、完整目标完成。

### 局部拒绝回灌撤回与制动窗口回归（2026-10-29）

输入 revision 为 `191d3a8` 加未提交的 max 形式 `estimator.cpp` 改动。该工作区的全量
CTest 为 88/90：`industrial_200k_mission` 与 `multichassis_mission` 失败；把
`estimator.cpp` 换回 HEAD 版本后 `multichassis_mission` 仍失败。复现与对照均在
`build/` 和独立构建中完成；SIM 真值只用于离线断言，没有进入 ROBOT。

- 失败 1：`build/test_multichassis_mission`。差速底盘的真值停在距目标 (6, 0) 约 0.25 m
  处，定位 σ 为 0.503 m，越过 lost 阈值 0.45 m，也宽于到达包络 0.25 m，`done` 断言失败。
  只关闭局部拒绝回灌后，四种底盘全部通过。
- 失败 2：`build/test_industrial_200k_mission --trace --trace-interval=500`。前往
  `central_gate` 时 step 25951 的 σ 为 0.451 m，随后增长到 step 29226 的 5.215 m；
  step 29247 `recovery-promoted` 采纳的估计相对真值为 (+2.783, +0.026) m，σ 只有
  0.300 m，该目标最终以 `no-response` 锁存。关闭同一条回灌后 `central_gate` 到达，
  任务推进到第 5 个目标（4/5，零碰撞）。
- 根因（失败 1、2 相同）：被拒绝的局部匹配残差沿 rank-deficient 走廊 ridge 的不可观测
  方向取值任意，而局部搜索窗为 `clamp(4σ, floor, cap)`，随 σ 增大。回灌使 σ、搜索窗和
  残差相互放大，反复的走廊扫描把 σ 推过 loss envelope，整图恢复随后采纳了错误位姿。
  `supervisor.cpp:89` 已规定被拒绝的扫描匹配只是弱负证据。
- 处理：移除 `try_local_correction` 中的第三处 `note_absolute_rejection`；
  `test_localization` 的 partial ridge 用例改为断言被拒绝后位姿、σ 和航向都不变，并
  连续重放 4 次（间隔 6 s）仍不变；`IEstimator::note_absolute_rejection` 的接口注释写明
  只有已被匹配器/追踪器接受的绝对证据才会上报。
- 失败 3（失败 1、2 修复后暴露）：`signal_town_mission` 与工业长图最后一个目标以
  `execution_fault/latched: uncommanded-motion` 结束，估计与真值只差 0.14–0.23 m
  （工业终态 0.139 m，signal_town 全程最大 0.231 m）。
  signal_town 日志中 step 3765 路线失效、新规划挂起（`route_hold/global-plan-pending`），
  命令归零，此时估计速度 1.41 m/s、偏航率 −0.42 rad/s；锁存发生在 step 3790–3795 之间
  的制动尾部（估计速度 0.08 m/s、偏航率 −0.20 rad/s，仍大于 2 倍到达带 0.16 rad/s）。
  旧 `ExecutionMonitor` 每个 tick 用当前速度重算允许时间
  `delay + reaction + |v|/brake + confirmation`（差速配置为 0.1 + 0.2 + |v|/2.0 + 0.2 s），
  该时间随机器人减速同步缩短到基线 0.5 s，被制动尾部越过；同一公式下，加速度达到制动
  减速的失控车辆会让自己的允许时间同步增长（算术结果，未在仿真中观测）。
- 处理：允许时间在分歧事件开始时由测得速度固定（`pending_speed_mps_`）；
  `test_execution_monitor` 新增三个用例：1.6 m/s 健康制动不锁存、半减速制动在预算处
  锁存、持续加速不延长预算。
- 独立验证方式：`git archive HEAD` 导出到 `build-verify/src`，依赖通过
  `-DFETCHCONTENT_FULLY_DISCONNECTED=ON` 与 `FETCHCONTENT_SOURCE_DIR_*` 复用
  `build/_deps/*-src`，在工作区同时被其他会话修改时仍能得到可归因的结果。
- 结果：执行监控修复为 `e1b4150`；局部拒绝回灌撤回为 `0fef7b5`
  （`fix: stop feeding rejected local scan matches into covariance`）。Release 验证中发现的
  `native_runtime` 挂住是独立缺陷，已由 `4da2bc1` 修复（见下节）。
  - HEAD + 执行监控修复（不含撤回）：除三个 `industrial_200k*` 任务外 86/87，仅
    `multichassis_mission` 失败（进程中止，5.80 s）。
  - HEAD + 撤回（不含监控修复）：同口径 86/87，仅 `signal_town_mission` 失败（27.87 s）。
  - 两者合并（RelWithDebInfo）：全量 90/90，536.65 s；`industrial_200k_mission` 362.92 s、
    `signal_town_mission` 25.27 s、`multichassis_mission` 5.29 s 均通过。
  - Release（-O3）：89/90，唯一失败是下节的 `native_runtime` 挂住（728 s 后手动终止）；
    `industrial_200k_mission` 通过：steps=70292，5/5，零碰撞，max_pos_err=2.175 m；
    `industrial_200k_direct_mission` 55.92 s、`signal_town_mission` 24.03 s、
    `signal_town_big_mission` 14.64 s、`ice_lab_mission` 5.07 s 均通过。
  - 三个提交（`e1b4150`、`4da2bc1`、`0fef7b5`）之后的 HEAD：独立导出，RelWithDebInfo，
    `ctest -j6` 全量 90/90，439.93 s；`industrial_200k_mission` 273.06 s、
    `industrial_200k_direct_mission` 61.25 s、`signal_town_mission` 24.22 s、
    `signal_town_big_mission` 15.68 s、`multichassis_mission` 4.41 s、
    `native_runtime` 0.36 s 均通过。
- 未入库的 max 形式候选：工作区中的另一份未提交改动把
  `FusedOdometryEstimator::note_absolute_rejection` 改为将每个对角元提到
  `max(方差, 残差²)`，并配有 `test_localization` 用例 2a。在与上面相同的基线（撤回 +
  监控修复，RelWithDebInfo）上，`industrial_200k_mission` 为 4/5：`south_yard` 以
  `execution_fault/latched: no-response` 结束，steps=100000，max_pos_err=5.082 m；同一
  基线的累加形式（HEAD）为 5/5。这份改动没有入库。恶化的机理、触发它的两项独立缺陷、
  以及两项原先未验证的风险（纯原地转向停车预算、step 3765 路线失效的触发条件）已在下节
  给出证据与结论。

### max 形式候选的回归归因与任务门禁的相位依赖（2026-10-29）

输入 revision 为 `c51292c`。工作区另有两组未入库改动：其他会话的 max 形式
`estimator.cpp` 与 `test_localization` 用例 2a；本节的积分重置候选 F-C
（`ackermann_controller.{hpp,cpp}` 与 `test_actuation` 新增块，见缺陷 D1）。所有运行都在
`build-verify/` 的独立 RelWithDebInfo 构建中完成。探针与环境开关只存在于该目录（未入库）：
`LS_FORM=acc|max` 选择协方差形式，`LS_NOFC=1` 还原 HEAD 的控制器，`LS_TRACE_CTRL` 打印
速度环内部量，`LS_TRACE_MONITOR`/`LS_TRACE_ROUTE` 打印监控与路线状态，`LS_CONFIG_DIR`
指向只改动 SIM 侧 actors sidecar 的配置副本。SIM 真值只用于离线断言与 trace，没有进入
ROBOT。

- 规范任务（`test_industrial_200k_mission`，5 个目标）在六种代码组合下的结果：

  | 代码 | 结果 |
  |---|---|
  | HEAD（累加形式） | 5/5，steps=70292，零碰撞，max_pos_err=2.175 m |
  | max 形式 | 4/5，`south_yard` 以 `execution_fault/latched: no-response` 结束，steps=100000，max_pos_err=5.082 m |
  | max + F-C（仅方向反转时重置） | 5/5，steps=81570，零碰撞，max_pos_err=3.216 m |
  | 累加 + F-C（仅方向反转时重置） | 4/5，同样在 `south_yard` 以 `no-response` 结束，max_pos_err=3.329 m |
  | 累加 + F-C（停车与反转时重置） | 3/5，碰撞 58 步，max_pos_err=1.689 m，第 4 个目标途中被 `east_shuttle` 撞停 |
  | max + F-C（停车与反转时重置） | 3/5，碰撞 68 步，max_pos_err=1.971 m，现象同上 |

  任一环节变化都会使结果在这六种之间翻转，规范任务没有鲁棒余量。第 2、4 行的失败是
  D1，第 5、6 行的失败是 D2；第 1 行的 5/5 与 D2 并不矛盾，HEAD 在 m03 的相位扫描中
  同样有一个相位被撞（见 D2）。两种协方差形式都落入过 D1（证据 A 为 max 形式，证据 B
  为累加形式），差别只在触发位置，所以这些运行没有证明 max 形式本身是恶化的原因。
- 缺陷 D1：速度环 PI 积分在制动或停车之后残留，造成 `no-response` 误锁存。
  - 机理：`AckermannController::update` 的两个后轮速度环是 PI（`kp=0.6`、`ki=2.0`、
    `integral_limit=2.0`、`ConditionalIntegration`，`ackermann_controller.hpp:24-29`、
    `ackermann_controller.cpp:12-17`）。设定值下降或停车时被测速度滞后，积分累积出维持
    制动斜坡的负占空比；设定值随后反向或同向再起步时这份积分仍在，驱动占空比停在制动
    侧，要等积分以约 `ki·err` 的速率退回才转正。`ExecutionMonitor` 的窗口是响应延迟 0.1
    + 确认 0.2 + 反应 0.2 = 0.5 s（`execution_monitor.cpp:93-95,213-214`），退回时间超过
    它就锁存。
  - 证据 A（max 形式的 4/5 失败，速度环内部量）：t=1800.42 s 设定 0.076、实测
    0.276 m/s，`i_l=-0.345`、`duty_l=-0.544`；t=1800.58 s 设定跳到 +0.277，实测
    0.077，`i_l=-0.320`、`duty_l=-0.106`，`no-response` episode 从这里开始；`duty_l`
    随后为 −0.044（1800.74）、+0.032（1800.90）、+0.114（1801.06）；实测速度保持
    ≤0.054 m/s，窗口在 1801.08 s 耗尽并锁存。同一运行锁存时估计与真值相差 0.18 m，
    不是定位丢失。
  - 证据 A 的设定值跳变发生在规划侧的尖点处理里。取自同一 4/5 失败的逐步探针（t = step ×
    0.02 s，所以 step 90021、90029、90054 就是上面的 1800.42、1800.58、1801.08 s；探针源码
    与日志在 `build/evidence/ind_probe.cpp`、`ind-probe-gear.log`，未入库）：路线共 13 点，
    step 90003 起第 1 段 `gear_change_pending` 且 braking armed，到该段终点 0.394 m；
    step 90018–90022 设定 0.183→0.030、实测 0.383→0.230 m/s；step 90023 段号 1→2、
    `requested_gear=-1`，命令 −0.008（实测 0.192）；step 90024 命令 −0.047（实测 0.153），
    到第 2 段终点 0.320 m；step 90025 段号 2→3、`requested_gear=+1`，命令 +0.323（实测
    0.123）；step 90031 整条路线被替换（`seg=0/11`）。三条代码事实产生这一串：
    1. 停稳判据放行：`settled_for_gear_change` 是 `|前向速度| <= decel * dt`
       （`dwa_planner.cpp:469-470`），其中 `dt` 是规划步长 `dt_s = 0.1`
       （`dwa_planner.hpp:40`），不是 0.02 s 的运行周期；`decel = 2.0`
       （`description.hpp:76`）。门限 0.2 m/s，0.192 m/s 通过。
    2. 倒车段比捕获半径短：`cusp_capture_m = footprint_radius_m = 0.4`
       （`dwa_planner.cpp:459`、`configs/robot/ackermann.json:19`），第 1 段终点距离
       0.394 m、第 2 段终点距离 0.320–0.323 m 都小于它。进入第 2 段后第 1 个决策 arm
       （step 90024），第 2 个决策满足 `cusp_reached` 并推进（step 90025，
       `dwa_planner.cpp:471-486`）。倒车段只存在了这 2 个决策，其间 truth 前向速度
       0.189、0.152 m/s，机器人没有后退。
    3. 动态窗口以实测速度为中心：`next_velocity_window(est.forward_mps, ...,
       accel * dt, decel * dt)`（`dwa_planner.cpp:386-387`），窗口是实测 ±0.2 m/s
       （`accel = 2.0`，`description.hpp:75`）。−0.008、−0.047、+0.323 分别等于
       0.192−0.2、0.153−0.2、0.123+0.2，设定值每次取窗口边界，step 90024→90025 一步
       跳 +0.37 m/s。
    速度环正是带着负积分（上面的 `i_l=-0.320`）在这一步跳变上起步。Hybrid-A* 为什么在
    这个尖点前后产生 0.39 m 与 0.32 m 两个相邻短段，需要验证。
  - 证据 B（累加形式 + 仅反向版 F-C 的 4/5 失败）：t=1931.20 s 从 1.043 m/s 停车
    （设定 0，`duty_l` 饱和在 −1.0），`i_l` 由 −0.273 退到 −0.405（t=1931.34）；
    t=1931.56 s 同向设定 +0.208 到来，`i_l=-0.408`、`duty_l=-0.212`，之后每 0.12 s 为
    −0.164、−0.121、−0.065、−0.023，整个 0.5 s 窗口内驱动占空比没有转正，实测
    ≤0.038 m/s，t=1932.06 s 锁存。这一路径不经过方向反转，所以仅反向版 F-C 覆盖不到。
  - F-C 的处理（候选，未入库）：设定为 0（停车或保持）或与上一个非零设定反号时，重置两个
    后轮 PID，前馈与比例项立即作用于新需求。`test_actuation` 新增块断言：反向与停车之后
    的输出与全新控制器逐位相同（1e-12）；同向减速到 0.05 m/s 仍保留积分
    （`rear_left_duty < reference − 0.05`）。红绿验证：HEAD 控制器在反向断言
    （候选分支上的 `test_actuation.cpp:188`）中止；只含反向规则的控制器在停车断言
    （`:203`）中止；含两条规则的控制器输出 `actuation ok`。
  - F-C 的副作用测量：在没有 actor 交互的相位下，m03 在 F-C 上 29773 步完成，HEAD 为
    31754 步（−6.2%）。
  - 另有证据指向同一类机理但没有取速度环内部量：HEAD 的 m04、m09 都在
    (214.344, 31.392) 以 `no-response` 锁存（两者终态逐位相同），F-C 下 m09 在该位置
    之后全程通过；该位置的内部量需要验证。
- 缺陷 D2：定位恢复后的爬行速度上限使机器人停在非让行车辆的车道里，被撞。
  - SIM 侧事实：`east_shuttle` 半径 0.7 m、1.4 m/s，在 y=72 上 x∈[310,470] 往返，没有
    `yield_to_robot`（`configs/worlds/industrial_200k.actors.json`）；激光 72 束、最大
    量程 10 m（`configs/sim/default.json:16`）。ROBOT 侧常数：
    `dynamic_escape_speed_cap_mps = 0.3`（`safety.hpp:115`）。
  - 失败轨迹（累加 + F-C 停车版，规范任务，`--trace-interval=25`）：step 54825–54875 一次
    定位恢复把估计误差从 1.54 m 拉到 0.38 m（σ 由 5.821 m 降到 0.836 m）；step 54900
    路线被清除（`route_hold/global-plan-pending`，速度 1.09 m/s）；重规划后从 step 54975
    起速度被限在 0.25 m/s（`loc=recovery-tracking hypotheses=4`，`speed_cap`）。step 55200
    `tracks=0`，shuttle 中心距机器人 6.2 m；step 55250 `tracks=1`、`dynamic=0.048`，进入
    `dynamic_escape`（命令 0.25 m/s，距 4.7 m）；step 55325 `candidates rejected: dynamic
  collision on hold` 加 `safety:dynamic-proximity` 停车，机器人在 (403.0, 72.8)，距车道
  轴 y=72 为 0.8 m，小于 shuttle 与机器人半径之和 1.1 m，shuttle 距约 2.4 m；step 55425
  接触并锁存 `uncommanded-motion+safety:contact`，累计碰撞 58 步。
  - 相位扫描（m03 = `central_gate east_terminal`；只改 SIM 侧 sidecar 中 shuttle 的初始 x，
    取 310…470 共 17 个点，间隔 10 m 约 7.1 s）：HEAD 控制器 16/17 无碰撞，x=440 一次
    碰撞（25 步）；F-C 15/17，x=340 与 x=350 碰撞，规范任务的 x=350 落在其中。相位不
    触发交互的运行轨迹逐位相同（HEAD 恒 31754 步，F-C 恒 29773 步）；HEAD 控制器在
    x=350 复现了独立基线（31754 步，2/2）。
  - 结论：D2 存在于 HEAD。规范任务过或不过，取决于该控制器版本进入车道的时刻是否落在
    车辆的碰撞窗口内。1/17 与 2/17 的差别在该样本量下不能区分，需要验证；规范任务本身
    没有做相位扫描（单次约 5 分钟），需要验证。
- 十变体矩阵（目标序列 `m01`…`m10` 共 22 个目标）。完成的目标数：HEAD 14/22，max 形式
  14/22，累加 + F-C 18/22，max + F-C 18/22；完整通过的任务数分别为 5、5、7、6。
  F-C 列为停车与反转版：

  | 任务 | HEAD | max 形式 | 累加 + F-C | max + F-C |
  |---|---|---|---|---|
  | m01 `west_hub north_depot` | 2/2 | 2/2 | 2/2 | 2/2 |
  | m02 `north_depot central_gate` | 2/2 | 2/2 | 2/2 | 2/2 |
  | m03 `central_gate east_terminal` | 2/2 | 2/2 | 1/2 车道碰撞 | 1/2 车道碰撞 |
  | m04 `east_terminal south_yard` | 0/2 `no-response` | 0/2 `no-response` | 1/2 估计误差 44.4 m | 1/2 估计误差 50.7 m |
  | m05 `south_yard central_gate` | 1/2 误差 40.3 m | 1/2 误差 40.3 m | 2/2 | 1/2 误差 414.2 m |
  | m06 `west_hub central_gate east_terminal` | 3/3 | 3/3 | 1/3 误差 156.4 m | 3/3 |
  | m07 `central_gate south_yard` | 2/2 | 2/2 | 2/2 | 2/2 |
  | m08 `north_depot east_terminal south_yard` | 1/3 `no-response` | 1/3 `no-response` | 3/3 | 3/3 |
  | m09 `east_terminal central_gate` | 0/2 `no-response` | 0/2 `no-response` | 2/2 | 2/2 |
  | m10 `south_yard north_depot` | 1/2 误差 233.8 m | 1/2 误差 389.3 m | 2/2 | 1/2 误差 18.1 m |

  F-C 消除了 HEAD 上三处 `no-response` 终态（m04、m08、m09 都不再在该处锁存）。四个
  版本都在 m04、m05、m06、m10 的若干个任务上出现 18.1–414.2 m 的估计误差（终态
  `global-plan-failed`、`hold/no-plan` 或 `speed_cap/recovery`），丢失的任务随协方差
  形式与时序翻转，机理没有定位，需要验证。
- 缺陷 D3：局部 `global recovery resolved` 的过度自信。累加 + 仅反向版 F-C 的 m07
  （steps=28090，1/2，max_pos_err=4.982 m）中，全局重定位事件 step 22466、22666、22866
  都是 `insufficient geometric returns`（回波 2、7、11，低于 `minimum_returns=12`，
  `global_search.hpp:63`、`global_search.cpp:282`），σ 从 12.285 m 增长到 13.156 m，估计
  误差约 4.8 m；step 22987 `local_correction:global recovery resolved` 之后 σ 降为
  0.212 m，误差仍是 4.33 m（truth (440.331, −144.876)，estimate (439.918, −149.187)），
  最终 `south_yard` 以 `robot_done=yes truth_arrived=no miss=3.342` 结束。其他位置的
  同类事件需要验证。
- 缺陷 D4：路线失效与瞬时停车。`signal_town` 在 HEAD 上有 5 次 `route_deviated`：偏离
  距离 0.800–0.803 m，对应管道宽度 0.800 m（`2.0 × footprint_radius_m`，
  `autonomy_pipeline.cpp:25-28`），速度 1.41–1.86 m/s；step 3765 是第 4 次
  （0.803 m，1.687 m/s，目标 (1.5, 2.0)）。触发条件是 `route_deviated`，不是
  `request_changed`（探针中 `request_changed=0`）。`begin` 在无待定计划时清除路线
  （`route_pipeline.cpp:100-115`），保持分支随后下发零速命令
  （`motion_pipeline.cpp:265-275`），机器人从行驶速度瞬时转入停车。DWA 的横向误差储备
  限速只在 `car_like && route_tracking_valid` 时生效（`dwa_planner.cpp:715,732-746`），
  差速底盘没有这条限速。已否决的实验变体 `LS_SHORTCUT=nocancel` 在车式底盘上走到更坏
  的结果：待定计划期间继续沿陈旧路线行驶，偏离距离最长一段从 0.845 m 增长到
  3.107 m（1.91 m/s，目标 `south_yard`）；step 59198 起 `service_brake=1.0`，真值速度
  1.913 → 0.444 m/s 用时 1.08 s（平均 −1.36 m/s²，低于声明的 2.0 m/s²），位置在湿滑
  停车区（μ=0.42，x 410–470、y 85–185），最终以不带接触的 `uncommanded-motion` 锁存
  （真值 (453.98, 149.75)）。
- 纯原地转向停车的监控预算：窗口 0.5 s 不含平移项，因为 `pending_speed_mps_` 只取
  平移速度（`execution_monitor.cpp:201-202`）。按声明的偏航加速度 8.0 rad/s²
  （`configs/robot/*.json`）、响应延迟 0.1 s 与到达带 0.08 rad/s（判定阈值 0.16 rad/s，
  `description.hpp:65-66`、`coordinator.cpp:104-109`）估算，最大偏航率 2.0 rad/s 的停车
  需要 0.1 + (2.0 − 0.16)/8.0 = 0.33 s，余量 0.17 s；履带底盘上限 1.2 rad/s 需要
  0.23 s，余量 0.27 s。这是按声明参数的算术结果，实测偏航尾部没有取；十变体矩阵与相位
  扫描各运行（独立构建目录中）的终态里没有不带接触的 `uncommanded-motion` 锁存。结论：
  目前没有失败证据，实测尾部需要验证。
- 历史记录：F-C 曾以 `d6a9612` 保存在本地分支 `wip/ackermann-integral-reset`
  （只含 `ackermann_controller.{hpp,cpp}` 与 `test_actuation.cpp`，基于 `c51292c`）。
  在 `c51292c` 加 F-C 的独立导出（无任何探针，RelWithDebInfo，`ctest -j6`）上，全量
  89/90，661 s，唯一失败是 `industrial_200k_mission`：steps=100000，3/5，碰撞 58 步，
  max_pos_err=1.689 m，与带探针的构建逐位相同。F-C 与 max 形式都没有进入当前分支：
  max 形式单独应用使规范任务翻到 4/5（原因是 D1），F-C 单独应用使它翻到 3/5（原因是
  D2）；两个翻转的原因都已定位，且各自在对照运行中独立存在（D1 在两种协方差形式中都
  出现，D2 在 HEAD 的相位扫描中也出现）。
- 未实现的补救方向（均需要验证，没有改动）：保持分支按声明的制动减速度斜坡下降而不是
  瞬时零速加服务制动（`add_recovery` 的 `clamp_next_velocity`/`clamp_next_symmetric`，
  `motion_pipeline.cpp:13-25,330-340` 是已有先例）；把横向误差储备限速扩展到非车式底盘；
  待定计划期间不沿陈旧路线行驶；动态逃逸与定位观察限速的关系；局部 resolved 恢复的
  时间确认；短于 `cusp_capture_m` 的相邻换挡段如何处理（D1 证据 A 的规划侧触发：该倒车段
  在 2 个决策内被穿过，设定值一步跳 +0.37 m/s）。

### 自适应拒绝证据与 Ackermann 控制段修复（2026-10-30，待本阶段提交）

- 当前工作区已经 supersede 上述历史记录：`FusedOdometryEstimator` 对已经被上层接受、
  但随后因创新门拒绝的绝对证据，只按 `[x,y,yaw]` 逐轴取
  `max(当前方差, 残差²)`，重复同一墙面证据幂等；局部拒绝仍不改变位姿或连续 odom。
  当全局搜索在现有协方差支持范围内没有候选时，`LocalizationSystem` 将本次实际搜索包络
  回馈为下一轮不确定度下界。它只使用 ROBOT 的 LiDAR 几何和既有搜索包络，不读取 SIM 真值，
  也没有新增关卡阈值。
- `AckermannController` 现在把命令停车和带符号驱动反转视为物理控制段边界，清除旧段后轮
  PI 积分；同向调速继续保留积分。这样在服务制动释放后，新的低速请求不会被旧的反向积分
  遮住，执行监控仍按声明的响应延迟、反应时间和制动能力判定真实无响应。
- 新增 `test_actuation` 覆盖停车重启与反向重启的逐位等价性；`test_localization` 覆盖重复
  拒绝证据幂等；`test_motion_pipeline` 覆盖可达速度逃逸必须经过同一 Validator。Release
  50,000 步工业回归已不再在 `east_terminal` 以 `no-response` 锁存，前三个目标零碰撞通过；
  完整五目标长测在本阶段提交前继续执行。
- 该段落将取代本节早先“局部拒绝不更新协方差”和“F-C 未进入当前分支”的工作区描述；
  历史结果保留用于解释此前失败，不作为当前验收结论。

### NativeRuntime 关闭竞态（2026-10-29）

全量 Release CTest 中 `native_runtime` 挂住，728 s 后被手动终止；CMake 没有为该测试设置
`TIMEOUT`。`src/robot/app/` 相对 `191d3a8` 没有改动，该挂住在 RelWithDebInfo（`build/`
和独立构建）与 Release 中都出现。复现方式是反复启动旧版 `test_native_runtime`（每次
8 s 超时）：本次复测的单次进程挂住率为 3/30 与 4/12，早先的长序列统计约 3–4%，随
机器负载变化。

- 现象：以 gdb 子进程方式运行测试并在挂住时发送 SIGINT，主线程停在
  `NativeRuntime::shutdown()` 的 `watchdog_thread_.join()`（`native_runtime.cpp:362`），
  看门狗线程停在 `watchdog_loop` 的 `watchdog_cv_.wait`（`native_runtime.cpp:323`）。
- 根因：`shutdown()` 在不持有 `state_mutex_` 的情况下写 `watchdog_exit_` 并
  `notify_all()`，而看门狗线程在持有该互斥锁时求值等待谓词。通知落在谓词求值之后、线程
  真正阻塞之前时，没有等待者可唤醒，线程随后阻塞，`join()` 不再返回。`start()` 在创建
  线程之前写该标志，`publish_output()` 在锁内修改 `armed_expiry_ns_` 与
  `watchdog_fired_`，只有 `shutdown()` 这一处在锁外发布状态。
- 窗口位置：独立构建中的探针（未入库，`LD_BIND_NOW=1` 以排除惰性符号绑定的影响）按纳秒
  级随机延时在 `start()` 之后调用 `shutdown()`。延时范围 0–4 µs 与 0–8 µs 各 8 次运行
  全部命中，首次命中的循环序号在 1–1680 之间，命中延时集中在 2–5 µs（最小 1.98 µs，
  最大 7.0 µs）；延时范围 30–100 µs 的 6 次运行只有 2 次命中。这说明看门狗线程的第一次
  等待紧随 `start()` 返回；该位置只在本机测得，其他机器需要验证。
- 处理：`shutdown()` 在 `state_mutex_` 下写 `watchdog_exit_`，再通知。
- 测试：`test_native_runtime` 新增 3000 轮 start → 0–16 µs 随机延时 → shutdown 的扫描，
  并在 5 s 内无进展时以诊断信息中止，卡住不再拖到套件超时。未修复代码上
  RelWithDebInfo 20/20、Release 10/10 次以退出码 134 中止；修复后两种构建各连续运行
  300 次全部通过，单次约 0.37 s。
- 局限：该用例靠延时扫描覆盖窗口，不是对线程交错的确定性控制；窗口位置不同的机器上
  检出率需要验证。

### Native HIL 确定性契约（2026-10-30）

- 第 8 项 HIL 收尾已加入 `tests/test_native_hil.cpp` 和 `native_hil` CTest target。
  测试用固定 `MonotonicNs` 事件脚本，不使用 `sleep` 推进控制结果，也不链接 SIM
  `VirtualMcu`；fake board 只实现 `IChassisMcuTransport`，因此 native HIL 与物理
  真值、actor identity、score 和渲染层保持隔离。
- 重放脚本覆盖固定 20 ms release、5/10 ms acquisition delay、一次反馈丢失后在
  liveness window 内恢复、连续丢失触发 feedback timeout、sequence replay，以及
  board 在最后一个 setpoint expiry 后的本地 watchdog stop。相同脚本运行两次，比较
  cycle result、typed setpoint kind/body 字段和 watchdog 结果，证明调度与通信结果
  不依赖墙钟速度。
- 该阶段证明的是 `NativeRuntime`/`IChassisMcuTransport` 的确定性契约，不宣称真实
  STM32 已完成 HIL。真实板卡仍需按 profile 实测 control/feedback period、传输延迟、
  急停、驱动负载和 expiry，再通过同一回放与现场测试门禁。

### Calibration record quality gate（2026-10-30）

- 第 9 项的现场记录入口现在在 CSV 读取和离线拟合两层都要求有限、非负且严格递增的
  acquisition timestamp；加速度、陀螺、轮行程和外部参考距离的每个轴/字段都必须是有限
  数值。重复或回退的记录会在进入 median wheel-scale/IMU-bias solver 前拒绝，不会通过
  重排悄悄改变证据权重。
- `OnlineCalibrator` 同样拒绝负 residual/baseline residual；已有 confirmation window、
  bounded step、range、checkpoint 和 journal 语义不变。`test_calibration` 增加乱序、
  NaN、负残差和 CSV 回退时间戳覆盖。
- `validate_native_hardware_profile()` 将 typed chassis capabilities 与 calibration
  artifact 组成一个 adapter 可复用的 bring-up gate，校验 profile id/revision、ownership、
  chassis mode、驱动/转向关节布局以及 calibration 的 robot hash、温度、载荷和过期条件。
  它只处理部署配置和实测 artifact，不把校准求解器或 SIM 状态带入 ROBOT tick。
- 首次 bench bring-up 不再要求用户先手工制作完整标定文件：
  `make_nominal_native_hardware_profile()` 可由 RobotDescription 与板卡 capability
  自动生成 `nominal:` commissioning artifact，比例使用声明几何的 1.0。该 profile 默认
  仍被 measured gate 拒绝，只有 adapter 明确设置 `allow_nominal_commissioning` 才能通过；
 生产部署仍必须替换为真实记录并关闭该开关。来源标签、`nominal:` digest 和零样本数
  现在要求彼此一致，不能把 measured artifact 重新标成 commissioning。首次使用路径已
  收敛为：板卡握手 → 自动生成 nominal bench profile → 采集 CSV 并生成 measured artifact。

### Sensor realism profiles（2026-10-30）

- F4 已完成：`configs/sim/default.json` 现在显式选择版本化 `sensor_realism` 的
  `nominal` profile；`ideal`、`nominal`、`adverse` 三个 profile 由
  `resolve_sensor_realism_profile()` 统一解析，集中定义 IMU 噪声/偏置随机游走、LiDAR
  测距误差/量化/近场盲区/丢包和 envelope 延迟/丢包。profile seed 是唯一随机源输入，
  同 profile/seed 的 provider 输出可重复。
- JSON loader 和 programmatic `ScenarioSession::apply_sim_config()` 都 fail closed：
  未知 profile、版本不支持、非法模型或同时混用 profile 与旧的 delivery/IMU 覆盖都会被
  拒绝，不能留下半应用配置。旧的直接 `delivery`/`imu_realism` 入口保留给实验和单测，
  没有 profile 时保持 `ideal` 兼容行为。
- 2D 与 3D LiDAR 使用同一个 resolved device model 和 seed；IMU 与 envelope delivery
  仍在 SIM provider 内生成。ROBOT 接收到的只有经过时间戳、噪声、延迟和丢包后的 typed
  observation，不会获得 truth、actor identity、score 或 profile 的内部参数。
- 验收：`test_sim_config` 覆盖 profile 解析、未知版本/名称、混用拒绝、确定性和 adverse
  参数；`test_realism`、`test_experience_mission`、`test_terrain_mission` 通过；Debug
  全量 CTest 为 **90/90**，包含 industrial 200k、signal_town、四底盘、truth-isolation
  和 robot-boundary。F4 阶段在本次变更中独立提交。

### Physics/presentation contract gate（2026-10-30）

- F3 已完成：`test_mujoco_backend` 现在按 differential、Ackermann、Mecanum、Swerve
  四种底盘从正式 robot config 构建 MuJoCo plant，检查引擎 read-back 的轮子 pose、接地
  高度、驱动/转向关节轴、collision geometry、COM 和 finite 状态；差速轮没有隐式
  steering joint，Ackermann 只有前方 idler 的转向轴，不能由渲染器自行补姿态。
- 同一测试对每种底盘比较 20 ms public step 与 20 个 1 ms fixed substep 的结果，验证
  physics/render snapshot 不依赖调用方批处理方式。已有 arm、caster、actor link 和
  terrain ramp 检查继续保留，所有姿态仍来自 MuJoCo body/joint/contact read-back。
- Asset manifest loader 新增 grounding-point containment gate：每个视觉资产的接地点
  必须落在 `expected_bounds_m` 内，越界声明 fail closed；这项元数据检查与 GLB hash、
  frame/axis、scene collision binding 一起阻止视觉 mesh 被错误抬高或埋入碰撞体。
- 验收：`test_asset_manifest`、`test_mujoco_backend` 通过；Debug/Release 的 F4 定向
  回归继续通过，Debug 全量 CTest 基线为 90/90。F3 阶段在本次变更中独立提交。

上一段记录的是 F4/F3/D2–D4 的历史收尾顺序；当前恢复点从 E1/E2 教学存档和机械臂课程验收继续。动态 actor 教学章已完成并保留在下方阶段记录；真实硬件 profile、急停、负载、传输延迟和 calibration artifact 仍按上面的外部验收队列执行，不用模拟数据冒充现场完成。

### Viewer composition split（2026-10-30）

- Viewer 的进程入口已经收敛为 `viewer_main.cpp` 的 5 行转发函数；完整应用编排移到
  `viewer_app.cpp`，避免 `main()` 同时承担进程边界和运行时实现。命令行、章节、镜头和
  性能选项继续由独立的 `viewer_options.*` typed parser 负责，`viewer_app.cpp` 只消费
  解析结果并负责 ScenarioSession/ExperienceFlow 生命周期、输入、播放循环和 HUD 编排，
  不再持有 GLB 资产加载、地面材质、阴影/bloom、2D/3D 场景绘制或 MuJoCo 姿态到显示
  坐标的转换实现。
- 直接 `--map` 启动在没有 `--robot/--sim` 时补用默认 differential 与 nominal SIM
  profile，再进入同一个 runtime hash/scene 流程；不会因空的必需输入路径失败。没有
  X11/Wayland 显示时 Viewer 在创建 GLFW 前 fail closed 并提示 `xvfb-run`，避免 raylib
  初始化阶段崩溃。
- 新增 `scene_renderer.hpp/.cpp` 作为 SIM-side presentation 模块：统一管理
  `VisualAssets` 生命周期、manifest grounding/axis 转换、物理 body/link/contact 的
  绘制、LiDAR/估计/碰撞证据层、相机和场景后处理。它只消费 `Frame`、`World` 和已记录
  的粒子/表面轨迹，不向 ROBOT 发布任何数据。
- `experience_overlay` 与 `player_progress_ui` 继续独立于渲染模块；章节档案仍只读取本地
  profile 和 authored manifest。这样后续加入机械臂关节、更多 GLB 和教学视图时，可以在
  renderer/overlay 中扩展，不再把逻辑堆回启动循环。
- 验收：`cmake --build build --target sim_viewer -j4` 通过，入口/选项/应用拆分后的
  Debug 定向回归 6/6 通过（含 `viewer_options`），
  包含 industrial 200K、signal_town、terrain/ice、四底盘、SLAM/loop closure、
  truth-isolation 与 robot-boundary。该阶段是 Viewer 结构提交点，后续算法/内容阶段继续
  单独提交。

### C4 typed chassis comparison（2026-10-30）

- 实验注册表现在同时支持连续变量和显式分类变量。`chassis.kind` 的域由代码白名单
  校验（differential、ackermann、mecanum、swerve、tracked），分类值以字符串保存，
  不用数字索引或 frontend 分支伪装成 typed 配置。
- 分类 variant 必须声明一个相对 manifest 的 `variant_robot` profile。Runtime 在构建
  ROBOT/SIM 之前解析该 profile，复用同一 world/scene/SIM 输入；invariant digest 保留
  baseline robot profile 作为固定输入，variant 的 chassis label 记录在 EpisodeArtifact，
  `EpisodePair::experiment_valid()` 要求两次 label 不同且 digest、地图、代码和预算一致。
  因而 chassis 差异可被审计，同时不会把 SIM truth、actor identity 或 score 送进 ROBOT。
- 新增连续、可观测但无路线特化障碍的 `configs/worlds/chassis_lab.json` 及其 scene：同一
  `chassis_goal` 任务比较 differential 与 Ackermann，两个 profile 从相同起点沿同一物理
  路段运行；侧向 landmark 保留 LiDAR/地图观测差异但不把 research_base 的复杂叉车通道
  误当作底盘能力门禁。两次运行都必须经过统一 ROBOT 估计、Validator、Safety 和 SIM
  truth arrival，不能通过放宽到达条件解决 Ackermann 的真实终端偏差。
- `test_experience_manifest`、`test_experience_runtime` 和 `test_experience_catalog` 覆盖
  分类域、variant profile、不同 chassis plant、digest 不变性、pair label 和同一路线双
  运行完成；本阶段定向回归（含 `truth_isolation`、`robot_boundary`）通过。

### Dynamic actor teaching lab and chassis entry points（2026-10-31）

- 动态 actor 教学章不再借用 Signal Town 的远处交通作为“动态避障”证据，新增连续的
  `dynamic_safety_lab` world、scene 和 actor sidecar：匿名叉车从走廊外侧横穿机器人
  的路线，ROBOT 只能从带 acquisition timestamp 的 LiDAR 观测建立 anonymous
  `DynamicTrack`，预测、Validator 和 Safety 共同产生让行/恢复动作；actor id、未来
  waypoint、MuJoCo pose 和物理接触仍只在 SIM/Viewer 侧存在。
- 新增 `test_dynamic_safety_experience_mission`：从 manifest/runtime 真实加载 sidecar，
  检查 anonymous track、预测响应、恢复前进、零碰撞和独立物理到达；`dynamic_crossing_mission`
  继续覆盖不同横穿时相。章节首帧、scene binding、catalog 和 Xvfb 3D smoke 均通过。
- Viewer 新增 `--chassis differential|ackermann|tracked|mecanum|swerve` 标准 profile
  快捷入口，直接解析到 `configs/robot/*.json`；`--robot FILE` 保留给自定义 profile，
  两者互斥。`--chapter chassis_lab` 仍在 baseline 完成后通过 `A` 或复盘按钮运行
  Ackermann variant。底盘选择只改变 ROBOT/物理配置契约，不改变场景真值或注入任何
  SIM 观测。
- 定向回归：`viewer_options`、`dynamic_crossing_mission`、
  `dynamic_safety_experience_mission`、`experience_manifest`、`experience_catalog`、
  `scene_manifest` 六项通过；`check_boundary.py` 通过。

### Grounded visual asset refinement（2026-10-31）

- 本机 Blender 资产检查入口固定为 `/disk/blender-5.2.2-linux-x64/blender`
  （Blender 5.2.2 LTS）；无系统级 `bpy` 时必须用该二进制执行
  `tools/modeling/inspect_assets.py`。

- RO-1 被动万向轮的视觉 GLB 现在与 MuJoCo 的三段冠状胎面同构：中心名义半径胎面
  承载，左右肩部半径降低，轮毂和旋转见证仍在同一 `wheel_roll_link` 坐标系中。
  渲染器继续只读取 MuJoCo 的 caster fork/wheel body pose 和 roll/swivel joint；GLB
  不再添加第二个解析轮胎或独立动画，因此坡面、打滑和转向时视觉与物理接触保持一致。
- 同一轮胎 mesh、RO-1 车体前部防擦件和 LiDAR 状态环由 Blender 生成脚本重新导出，
  保持 manifest 的米制 bounds、grounding point、材质和 triangle budget；资产 hash 已
  更新。`inspect_assets.py` 完整 53 项通过，`test_asset_manifest`、`test_viewer_frame`
  和 `test_mujoco_backend` 通过。
- 这项只改变 SIM presentation asset，不改变 ROBOT、碰撞几何、轮半径、底盘参数或
  传感器输入；MuJoCo 物理模型仍是姿态、接触和粒子/音频证据的唯一来源。

### Chassis-specific local planner projection（2026-10-31）

- DWA 与 TEB 继续共享感知、局部地图、预测、Validator 和制动尾迹，但不再把第一条
  body twist 当成所有底盘都可直接执行。TEB 的时间缩放候选现在也经过已测左右轮速、
  differential track 和轮速上限的可达窗口；Ackermann 继续在速度相关曲率区间内采样，
  Mecanum/Swerve 只有在声明的横向能力存在时才允许横移。执行器不会再把“已验证命令”
  隐式改成另一条命令。
- `test_local_planner_compare` 新增高初速、高偏航差速回归，逐候选检查左右轮的一周期
  加速度窗口与轮速上限；原有 DWA Ackermann 曲率、TEB Ackermann 零速不可原地转向和
  四底盘任务回归继续通过。该变化只使用 ROBOT 配置和反馈，不读取 SIM 真值。

### Terminal route and loop closure recovery（2026-10-31）

- A* 增加了真实起始位姿的保留：当估计起点和语义目标落在同一个栅格时，仍输出至少两
  个世界坐标位姿，局部规划器可以完成终端航向，路线管线不会把可达目标误报为
  `global-plan-failed`。新增 `test_astar` 覆盖同栅格、不同终端航向。
- DWA 的终端航向机动可以在声明的原始位置合同内开始；`GoalArrivalPolicy` 仍要求
  confidence-bounded 位置/航向和连续停稳证据才会报告到达，因而没有放宽完成条件或把
  SIM 真值传入 ROBOT。回环测试现在用固定仿真步覆盖闭环修正后的真实收敛尾段，首次
  回环仍发生在 east/north/west 三个目标完成后的重访阶段，`astar` 与 `loop_closure`
  定向回归通过。
- 路线管线保留结构化 planner diagnostic 并在 ROBOT evidence reason 中呈现，便于长图
  失败区分 endpoint、地图快照和搜索耗尽；该诊断不含真值、actor identity 或 score。

### Planner escape seed and wheel presentation polish（2026-10-31）

- 第二个真实失败点已修复：定位修正后估计起点可能落在保守的膨胀清障区内。A* 现在只
  把原始占据单元判为硬失败；自由但落在膨胀边界内的起点保留为 ROBOT 估计的逃逸种子，
  路线首点也保留精确估计位姿，避免先把底盘命令到膨胀栅格中心。局部 Validator/Safety
  仍检查第一段实际运动，未放宽清障合同，也没有新增关卡阈值或 SIM 真值入口。
- `survey_longitudinal_grade()` 不再把墙体/障碍物 authored box 顶面当作可行驶坡面；只有
  未占据的 ground elevation 才参与有限差分。新增 traversal 回归锁定墙边不会制造假坡度，
  真正的连续 ramp 仍保持其测得坡度。
- 标准 RO-1 驱动轮的 Blender presentation asset 增加了保持在声明半径内的窄冠状胎面环，
  仅用于侧视滚动识别，不改变 MuJoCo tire/contact geometry、轴向、轮心高度或碰撞尺寸。
  `inspect_assets.py`、asset manifest hash、`test_asset_manifest`、`test_viewer_frame` 和
  `test_mujoco_backend` 已验证该 GLB 的 bounds、grounding、triangle/material budget。

### Presentation asset beauty pass（2026-10-31）

- 叉车的完整资产和可绑定的 base/drive-wheel/steer-wheel/fork 模块增加了车灯、后部
  状态灯、轮胎侧壁环和货叉安全标识。新增网格全部位于原有 link-frame bounds 内，模块仍
  由 MuJoCo 的 base、轮、门架、carriage 和 fork body/joint pose 驱动；没有新增碰撞体、
  draw-time 动画或独立姿态积分。
- RO-2/RO-3/RO-4/RO-5 的底盘增加了低成本的 front/side status witness，使用统一的
  颜色语义区分四底盘能力，仍按各自 chassis link frame 导出。RO-1 既有轮胎冠状胎面、
  LiDAR 状态环和车体细节保持不变。
- 生成入口仍是 `build_actor_assets.py` 与 `refine_assets.py`，并显式处理 Blender
  Z-up 与 GLB/runtime `(x,y,z)` 基变换，防止细节把竖直 bounds 错写成横向尺寸。
  全部 53 个模型重新通过 `inspect_assets.py`：hash、单位/轴、pivot、grounding、link
  binding、三角面/材质预算和动画清单均一致；ROBOT 不读取任何 presentation asset。

### Teaching archive index（2026-10-31）

- Viewer 导出的 A/B report 与两份 v4 observation replay 现在写入本地 profile 的有界
  `archives` 索引；重复 report 路径幂等更新，最多保留 32 条，损坏、重复或超量数据
  fail closed。复盘进度面板显示当前章节的存档数量；profile 仍是 presentation-only，
  不进入 ROBOT/SIM digest 或任务判定。
- `test_player_profile` 覆盖 archive round-trip、幂等去重和重复路径拒绝；Debug/Release
  Viewer 构建与定向回归通过。

### Navigation envelope audit and Ackermann route recovery（2026-10-31）

- Ackermann 的声明底盘外接圆是 `sqrt(0.28² + 0.25²) = 0.377 m`，配置中的
  `footprint_radius_m = 0.4 m` 已经是半径，不是直径。全局 A*、Hybrid-A*、DWA、TEB、
  Validator 和 Safety 的硬清障语义仍是 `0.4 + clearance_margin(0.1) = 0.5 m`；不以
  直径替代硬膨胀，也不放宽 GoalArrival 合同。
- 修复了路线偏离阈值的直径误用：`DecisionCyclePipeline` 现在按底盘半径触发全局重规划；
  soft inflation 仍只参与局部偏好代价。规划刷新期间继续保留上一条已验证路线，新路线
  成功后原子替换，避免车式底盘在规划空窗期驶向墙边。
- `test_astar` 锁定 0.39/0.41 m 的半径边界、同目标增量刷新期间的旧路线保留；`test_dwa`
  和 `test_motion_pipeline` 回归覆盖车式恢复、动态逃逸和终端机动。Ackermann 无演员
  `signal_town` 长测完成 5/5、0 碰撞；带演员长测的剩余阻塞仍是动态让行恢复时间，
  不是碰撞膨胀或直径误用。

### Dynamic track coast and release regression（2026-10-31）

- 横穿回归暴露出短暂 LiDAR 遮挡后的丢轨问题：旧的 3 次 miss 窗口约为 60 ms，
  会在预测间隙重新放行局部规划。已将默认 coast 窗口设为 15 次 miss（约 300 ms）；
  预测仍以 track 的最后采集时间为起点，并按声明的不确定度增长扩展占用包络，超出
  预测年龄预算后才丢弃。没有引入 actor identity、未来 waypoint 或 SIM truth。
- `test_dynamic_crossing_mission` 三个到达相位现均为零碰撞并能恢复前进；`test_perception`
  的离场测试显式保留 3 次 miss 的老化契约。Release 差速 `signal_town` 带演员任务
  重新完成 5/5、0 碰撞，`execution_monitor` 的低速换向误报也已由定向回归锁定。

### Ackermann terminal direction under high covariance（2026-10-31）

- 终端末段的“目标是否已经落在当前 gear 的行驶方向后方”现在只由 ROBOT 估计均值的
  有符号投影和声明的位置容差判定。定位协方差继续只参与到达置信合同、安全包络和观察动作，
  不再把一个几何上已经落后的目标重新解释成仍处于前向可达盘内。
- 旧判据把置信半径直接加到终端投影上；协方差变宽时，Ackermann 会继续保留一条不可收敛的
  reverse 末段，反复尝试同一方向并在目标附近进入 `planner-hold`。修复后会请求新的全局
  pose maneuver，保留高协方差下“不能宣称到达”的约束。
- `test_dwa` 新增 `position_sigma=0.80 m` 的终端越过回归；Debug/Release 的 DWA、DWA/TEB
  对比、Ackermann planner 和四底盘 mission 定向回归通过。该修复不改变 `footprint_radius_m`
  的半径语义，也没有放宽 GoalArrival 的置信门槛。

### Surveyed-map submap frame ownership regression（2026-10-31）

- 已测绘地图下的 scan-to-submap tracking 现在只保留为 ROBOT 运动/诊断证据，不再调用
  `correct_relative_pose()` 改写绝对 map estimate。该 tracking 与轮速和同一 keyframe 链
  相关，直接融合会把 map->odom 逐步拉到车后；绝对地图帧只由独立的 surveyed scan matcher
  和受门控的 recovery/global correction 更新。
- 新增 `test_localization` 的 surveyed correlated-track 回归：tracking 仍能被接受，
  但 map-frame estimator 没有收到 relative correction。完整 Debug/Release
  `test_localization` 与 Release `test_terrain_mission`（2/2、0 碰撞）通过；没有改变
  到达阈值、底盘半径或 SIM/ROBOT 边界。

### 3D follow presentation cadence（2026-10-31）

- 3D 跟随视角的抖动来自显示频率与固定 20 ms physics cadence 不整除：窗口在 60 Hz
  绘制时会重复一个离散 MuJoCo frame 后突然跳到下一个 frame；原有相机 target 平滑
  不能平滑车体本身。Viewer 现在保留一个已完成的真实 physics frame 作为 presentation
  前帧，在 live 3D follow 期间按 PlaybackController 的固定步 debt 做相邻帧插值。
- 插值只作用于 body/caster/wheel/steering/link/actor 的可见位置、姿态和运动字段；最新
  frame 仍提供 observation、decision、collision、contact、costmap、LiDAR、粒子和音频
  证据。不会把插值结果写回 Episode、ScenarioSession、ROBOT 或 MuJoCo，也不会对关节
  使用墙钟积分；debrief、历史 review、Turbo 和 2D 路径保持原始记录帧。
- `test_viewer_frame` 覆盖相邻姿态的平移、短弧 yaw/steer 插值和最新离散证据保留；
  `test_playback_controller` 覆盖 presentation phase 在固定步之间推进并在 step 后归零。
  Release `test_viewer_frame`、`test_playback_controller` 和 Xvfb 3D `--sandbox --3d
  --shot 30` smoke 通过，渲染器正常加载 GLB、创建阴影/bloom framebuffer 并退出。

### Ackermann occupied-prefix experiment（2026-10-31，未提交）

- 对“已处于局部硬膨胀层时允许单调脱困前缀”的临时 DWA 方案做了 Release 编译和
  `test_dwa` 回归。完整 Ackermann `signal_town` 14000 步没有通过（卡在
  `market_gate`，出现 265 次碰撞，`done=no/passed=no`），说明该方案绕过了过多的
  局部硬安全约束；改动已撤回，未提交，也没有改变底盘半径、footprint 或到达条件。
- 第五目标右侧墙洞仍需基于局部 costmap/Validator 的真实证据继续定位，后续先补诊断和
  可重复的最小回归，再提交任何规划修改。

### Ackermann constrained boundary egress（2026-10-31）

- 最小复现确认了另一种可重复的死锁：估计的车体中心落在栅格化硬膨胀的首个边界格，
  但源地图仍观测到中心及底盘半径范围内的净空。Ackermann 的前向曲率轨迹无法离开这
  个离散边界，原有路线档位门控因而只剩停车候选，车辆无法继续请求新路线。
- DWA 现在只在这个初始状态允许受限的反向脱困前缀：源地图线段必须可通行、净空逐步
  增加，反向完整制动尾必须离开 lethal 层；前向移动样本仍拒绝，未知/真实占用、
  Validator、Safety 和独立制动尾约束保持不变。脱困后立即请求从新位姿原子刷新路线。
- `test_dwa` 新增该边界回归并验证产生反向命令和重规划请求；原有真实墙体导致无移动
  候选的回归仍保持拒绝。Release `test_dwa` 与 `test_multichassis_mission` 已通过，
  长测继续验证第五目标的墙角场景。

### 3D follow target filtering（2026-10-31）

- 固定步插值消除了 50 Hz physics frame 的停顿/跳帧后，跟随镜头仍会把底盘在刹停和
  倒车换向时的瞬时速度噪声放大成前后抖动：原实现每帧直接重算 lookahead 和镜头距离，
  速度在零附近变号就会把 target 拉到车头和车尾两侧。
- 新增 `FollowCameraFilter`。它只作用于 viewer 的 target/distance：对前后/横向速度和
  距离做时间低通，对前进/倒车方向设 0.12 m/s 滞回，并让 lookahead 也连续过渡；录制的
  Frame、SIM、ROBOT 和相机轨道角度都不被改写。2D follow 保留原有屏幕居中路径。
- `test_viewer_frame` 新增了零速交替噪声与持续倒车回归；Release `test_viewer_frame`
  与 Xvfb `--sandbox --3d --scene configs/worlds/lab_default.scene.json` smoke 通过。

### Ackermann confidence-bound terminal correction（2026-10-31）

- `test_multichassis_mission` 暴露了一个与镜头无关的终端边界问题：Ackermann 在
  `(5.892, 0.001)` 停住时，原始 0.25 m 到达半径已满足，但位置协方差半径约 0.144 m
  使置信度包络多出约 2 mm，GoalArrival 永远保持 `approaching`，DWA 却只生成零速 hold。
- DWA 现在把置信度包络剩余距离带入终端制动速度，并把这个小的可达速度显式加入采样
  偏好；它仍受 Ackermann 曲率、加速度、制动尾和 Validator/Safety 约束，GoalArrival
  仍是唯一完成判定。新增回归确认该状态生成正向修正；Release `test_dwa`、
  `test_multichassis_mission`（diff/ackermann/mecanum/swerve 全部完成）通过。
