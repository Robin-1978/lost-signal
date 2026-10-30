# 架构审查：新框架与业界实践对照

审查日期：2026-10-27  
分支：`refactor/clean-rebuild`  
对照：ROS 2/Nav2 常见分层、REP-103/REP-105、ros2_control/工业底盘控制器、HIL/回放确定性实践

## 结论

总体架构合理，核心边界已经具备发布级系统应有的形状：ROBOT 是可部署锚点，SIM 是通过 provider 接入的测量源；`map → odom → base_link` 的职责符合 REP-105；Coordinator 只负责生命周期和串联；规划、预测、验证、安全、控制、标定和硬件适配各有归属；SIM 真值只能在传感器生成、物理、渲染和离线评分链路中出现。

当前不能把它称为“所有真机能力已完成”。代码仍有几项需要验收的闭环。正确的判断是：**架构基线正确，SIM/回放路径成熟，真机路径正在从接口级可用推进到设备级可部署**。

## 与业界实践的对照

| 领域 | 当前设计 | 对照结论 |
|---|---|---|
| 进程和依赖 | `src/robot` 不依赖 `src/sim`；SIM 只能依赖 ROBOT | 符合可部署核心与仿真适配器分离；`tests/check_boundary.py` 是有效的架构门禁 |
| 生命周期 | `RobotApplication` + `BringupManager`，配置/激活/回滚/清理有序 | 对应 ROS 2 lifecycle node；还应把硬件 profile 和 conformance 结果纳入 activate 门禁 |
| 传感器契约 | `ObservationFrame` 外层传输头，scan/motion/IMU 各自有 acquisition header；有界队列、去重、过期和回放 | 符合 ROS QoS 的时间/来源分离原则；比旧的整帧时间戳更可靠 |
| 坐标系 | estimator 分开维护连续 `odom` 与可跳变的 `map`；`map→odom` 吸收校正 | 符合 REP-105；不能再把 `world→base_link` 当 ROBOT 静态 TF |
| 算法分层 | ObservationPipeline 持有异步测量状态；SLAM、mapping、prediction、global/local planning、validator、Safety、controller 分开 | 符合 Nav2 的 server/layer 方向；Coordinator 没有重新吸收算法策略 |
| 局部安全 | rolling local costmap 给规划；raw scan Collision Monitor 独立最终门；validator 做连续扫掠和制动尾迹 | 符合 Nav2 Collision Monitor 与 costmap 分层；仍需动态 footprint 和故障恢复状态机 |
| 底盘控制 | chassis 负责运动学和 host PID；typed setpoint 区分 duty、joint velocity、body twist；ownership 只有一层外环 | 符合 ros2_control/工业 MCU 的单一闭环所有权原则；HIL 还需覆盖三种 ownership |
| SIM 控制时钟 | VirtualMcu 使用 sim-time deadline；默认单线程、并发测试可替换；倍率不改变事件顺序 | 符合确定性仿真和可回放 HIL；墙钟只属于真机 pacing/watchdog |
| 真值隔离 | SIM truth 不进入 ROBOT；score/renderer/actor identity 是离线或展示状态 | 符合 sim-to-real 和可证伪评估要求；静态门禁与 truth-isolation 测试已经存在 |
| 标定 | 已有带 hash、协方差、适用范围、父版本和回滚约束的设计文档 | 方向符合工业 calibration artifact；artifact 读写、签名/持久化和启动握手仍需代码化 |
| 可观测性 | diagnostics、DecisionEvidence、replay、RTF/tick p99、底盘反馈字段已存在 | 适合教学和工程排查；provider conformance 尚未成为所有运行时的强制门 |

## 已经成立的关键原则

1. **ROBOT 只接收测量，不接收仿真事实。** `WorldTruth` 只在 SIM provider、MuJoCo、viewer、recorder 和离线 scorer 之间流动。RobotApplication 接口没有 actor id、MuJoCo handle、碰撞真值、材质、脚本路线或 score 字段。
2. **估计和控制的时间语义正确。** measurement 的采集时刻不会被 envelope 到达时刻覆盖；慢 LiDAR 在自身 freshness 有效时仍可用，快 IMU/轮速不会抹掉扫描；replay 走同一 channel 和 tick 路径。
3. **Coordinator 没有变成“大算法类”。** 异步排序、定位状态、scan dispatch、地图适配和感知结果属于 ObservationPipeline；决策和安全由自己的模块拥有，Coordinator 只把证据传给下一层并发布最终 envelope。
4. **底盘有明确的跨 SIM/真机边界。** `IChassis` 不含物理或渲染；MuJoCo 只消费 actuator；真机通过 `INativeAdapter`/`IChassisMcuTransport` 翻译标准消息；过期和独立机械停车不依赖普通命令队列。
5. **视觉没有反向污染算法。** GLB、关节姿态、COM/contact overlay、粒子、声音都属于 SIM viewer；ROBOT 只能看到测量和自身估计。模型由配置绑定，姿态应来自物理 body/joint read-back。

## 当前仍未达到最佳实践的缺口

### A. MCU ownership 需要完成设备级验收

接口现在支持 `host motion loop`、`MCU joint loop`、`MCU body loop`，`IChassis::to_mcu_setpoint()` 也已经按差速、Ackermann、swerve、mecanum 做运动学转换；`NativeRuntime` 会校验 profile、底盘模式、关节数量并轮询 MCU feedback。主机在 MCU ownership 下不会运行 host PID。

仍需完成：

- 每种 ownership 的 fake board、丢包/乱序/过期、重连和机械停车测试；
- 真实板卡 HIL，证明单线程 SIM、线程压力适配器和 STM32 固定周期对同一 setpoint 序列给出同一结果；
- feedback endpoint 的 profile/sequence/故障字段和启动 zero-state 自检；
- adapter 的非阻塞 I/O 约束。`poll_observation()` 和 `write_actuator()` 是接口合同，但目前没有运行时检测阻塞或独立 I/O 队列。

### B. Provider conformance 已接入 native runtime，仍需设备级故障策略

`ProviderConformanceMonitor` 已检查来源、帧、序列、采集时间、future skew、速率和静默；
`NativeRuntime` 在 bring-up 注册 endpoint expectation，并在每个 cycle 观察原始 provider
frame，native stats 暴露累计失败数。`RobotApplication::publish_observation()` 继续负责
结构/ownership/队列校验，两层职责没有合并：

- activate 前检查 endpoint expectation、单位、frame、nominal rate；
- 运行期把 silence、重复序列、时间倒退和过期升级为可观测 health；
- health 只允许安全降级或停车，不能绕过 Safety。

NativeRuntime 当前对 conformance failure 直接机械停车并返回失败，下一步仍需把
设备级重连/恢复状态和 HIL fault injection 纳入发布门禁；不会在 ROBOT 内部重放
坏帧或用场景真值修复测量。

### C. 标定 artifact 已有第一切，仍需设备 profile 持久化

`robot/platform/calibration.*` 已提供版本化 schema、序列化 digest、CSV 实测记录导入、
median wheel-scale/IMU bias 拟合、在线有界更新和 rollback checkpoint；真实设备仍需
补原子持久化、温度/载荷 profile 与 HIL journal。没有实测 profile 时必须 safe hold；不能
用源码默认轮径、PID、波特率或摩擦系数冒充标定结果。

### D. 几何安全仍以圆形 footprint 为主

规划、validator、Safety 和部分行为逻辑使用 `footprint_radius_m`。这对差速圆形底盘可用，对叉车、Ackermann、托盘和将来的机械臂会产生过度保守或扫尾漏检。下一步应在 `RobotDescription` 增加 base polygon/载荷 envelope，并让全局膨胀、局部 swept volume、最终 Collision Monitor 和 SIM judge 使用同一几何版本。机械臂关节改变 envelope 时，必须先更新安全包络再允许动作。

### E. 执行跟踪还没有成为独立故障闭环

`TrajectoryTrackingSystem` 已提供 command-vs-odom 诊断，但持续超差、执行器卡死、反向运动或 feedback 丢失还应由独立 execution monitor 触发 latch、软停车/急停和明确恢复条件。Coordinator 只串联 monitor 的 verdict，不应自行实现超差策略。

### F. 物理真实性和默认 profile 需要分层发布

SIM 已有 MuJoCo 接触、被动制动、轮胎载荷/打滑、3D LiDAR、IMU 噪声和多速率调度能力，但默认 profile 仍需要明确划分 `ideal`、`nominal`、`adverse`，并用标定数据约束。叉车/行人若仍是 kinematic actor，只能作为确定性障碍测试；需要撞击反作用、质量和摩擦验证时，必须切换为 dynamic physical actor。渲染 mesh 和 collision shell 需要静置接地、轮心高度、关节轴和 COM 的自动回归。

### G. 任务调度能力仍是单 active goal

`RobotExecutive` 现在采用单目标、后来的请求抢占前一个目标的确定性策略。这适合当前教学路线，但不等同于 Nav2 的多目标资源/行为树能力。若发布版本承诺多站点任务，应把排序、暂停、恢复、取消、dwell、arrival speed 和失败重试放进 Executive/Task module，不能把队列状态写回 Coordinator。

## 推荐完成顺序

1. **真机闭环**：完成 A 的 ownership/HIL 和 B 的运行时 conformance；把 profile/feedback/watchdog 作为 activate 门禁。
2. **标定产品化**：实现 C 的 artifact schema、hash、原子提交、在线候选、健康门禁和回滚。
3. **几何和执行安全**：实现 D/E 的 polygon footprint、动态载荷 envelope 和 execution monitor。
4. **真实性分层**：完成 F 的 nominal/adverse 设备 profile、dynamic actor 和 mesh/collision 回归。
5. **任务能力**：按产品需求决定是否升级 G；单 active goal 不应被误称为完整行为树/任务队列。
6. **发布门禁**：Release 构建分别测 1×/8×/64×/Turbo/headless 的事件序列、RTF、tick p99、内存和 replay command digest；任何倍率只允许改变墙钟吞吐，不得改变 ROBOT 观测或命令序列。

## 最终判断

这套架构可以继续作为产品主干，不需要回到 master 的旧耦合方式。它已经满足“新框架优先、真值不穿透、算法下沉、SIM 可加速、真机可替换”的根本要求。发布前必须把上面的缺口作为工程验收项处理，尤其不能把接口存在误认为 MCU/HIL、标定和执行安全已经完成。

## 状态增量（2026-10-30）

上面的审查保留为当日基线；截至当前分支，以下闭环已经从“接口存在”推进到可在
CI 重放的契约测试：

- A 的 native fake board 已扩展为 `tests/test_native_hil.cpp`：固定
  `MonotonicNs` release、采集延迟、单次丢包恢复、连续丢包停车、sequence replay、
  typed setpoint 重放和 board expiry watchdog 都在同一 `NativeRuntime` seam 验证。
  两次固定脚本的 cycle result/setpoint body 字段/watchdog 结果一致；真实板卡急停、
  负载和 transport latency 仍需现场 HIL，不能由该测试代替。
- B 的 native provider conformance 已在 `NativeRuntime` 运行期 fail-closed；profile
  接口现在在注册 provider 前执行。设备级重连策略和非阻塞 I/O 线程仍属于具体 adapter，
  尚未被抽象成通用恢复状态机。
- C 的 artifact JSON、CSV 实测记录、原子保存/加载、温度/载荷/过期 gate、online
  checkpoint/journal 已代码化；`validate_native_hardware_profile()` 将 typed chassis
  capability 与 calibration artifact 一起校验。CSV/solver 还会拒绝乱序、重复时间戳、
  NaN 和负 residual。具体设备 profile 仍必须来自实测。
- E 的 execution monitor 已成为独立故障闭环，持续无响应、反向/失控运动和反馈过期会
  latch safe stop，恢复需要显式 acknowledgement；它不读取 MuJoCo contact、碰撞真值或
  actor identity。

因此当前真正的外部依赖只剩具体硬件 profile/急停/负载的实测验收，以及 D/F/G 的产品
范围扩展；SIM、回放和 native contract 层不应通过猜测数据伪造这些结果。

### 当前状态校对（2026-10-30）

历史审查中“标定产品化仍需代码化”的结论保留为当时的审查记录；当前分支已完成
artifact schema、CSV 质量门禁、原子保存、online rollback、native profile 聚合校验和
nominal commissioning。`7dc60b9` 又要求 source、`nominal:` digest 与零样本数一致，并
把首次使用收敛为自动生成典型 profile 后再采集实测 CSV。软件契约和回放测试已完成，
仍不能替代具体板卡的急停、负载、传输延迟和真实 calibration profile 实测。

## Git 仓库与发布边界

当前阶段保持一个 monorepo，使用目录、CMake target、边界检查和 CI 区分产品面：

```text
robot_simu/                         # 当前主仓库
  src/robot/                         # 可部署 ROBOT 库
    robot_core/iface/platform/...    # 稳定消息、算法和控制
    robot_app/                       # 组合根与 NativeRuntime
    robot_transport/                 # 通用字节/链路后端
  src/sim/                           # SIM 库和 viewer，只能依赖 robot
  examples/stm32_chassis_mcu/       # MCU 可移植参考固件/host test
  configs/ tests/ docs/              # 契约、场景、回归和教学材料
```

这里的 `lost_signal_robot_stack` 是可链接的 ROBOT 库集合，`lost_signal_robot` 是一个很薄的 native 可执行壳；它不是算法的第二份实现。真实机器人产品应在自己的硬件包中实现 `INativeAdapter`/`IChassisMcuTransport`，链接已安装的 `robot_stack`，提供设备 profile、串口/CAN/UDP/USB 等 transport 和传感器翻译。真机应用不复制 estimator、planner、Safety 或 chassis 算法。

SIM 则链接同一个 `robot_stack`，再链接 `lost_signal_sim_world`、`lost_signal_sim_providers`、`lost_signal_sim_scenario` 和 viewer/eval；依赖方向只能是 `sim → robot`。SIM 和真机都通过同一 `ObservationFrame`、`MotionCommand`、`ActuatorCommand`、`ChassisSetpoint` 和回放/conformance 契约。

现在拆成独立仓库的代价大于收益：LSRI schema、REP-105 语义、ownership 和安全字段仍需要跨算法、SIM、HIL 原子修改；分仓会把边界测试、回放一致性和真值隔离测试拆散。等以下条件同时成立，再抽出硬件适配器仓库：接口版本有明确兼容策略；profile/标定 artifact 有独立发布节奏；厂商 SDK 或许可证要求隔离；硬件仓库能在 CI 中运行 robot stack 的 conformance/HIL contract tests。

即使未来拆仓，也应保留三类边界：

1. `robot_stack` 以版本化 CMake package/export（或内部包管理器）发布；真机和 SIM 只能依赖公开头文件与契约，不引用 ROBOT 私有实现。
2. `robot_hw_<vendor>` 只实现 adapter、设备 profile、诊断和硬件测试；设备差异不能进入 ROBOT 算法分支。
3. `sim` 可以独立发布，但必须消费固定版本的 `robot_stack`，并在发布 CI 中运行 truth-isolation、replay command digest 和跨倍率一致性测试。
