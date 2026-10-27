# MCU 控制域与真机适配设计

状态：设计基线（2026-10-27）

目标是让同一套 ROBOT 决策、运动学和安全语义可在 SIM、HIL 和真机 MCU 之间迁移，同时保持确定性、真值隔离和单一闭环所有权。

## 结论

MCU 是独立的确定性控制时钟域，不是 ROBOT 中普通的 IProvider，也不是主机决策线程里的一个隐式回调。

- 真机：MCU 由硬件定时器、中断或 RTOS 固定优先级任务驱动；主机 ROBOT 决策、主机 I/O、主机 watchdog 和 MCU 伺服各自有周期。
- SIM：默认不创建墙钟线程。ScenarioSession 在固定 1 ms 物理网格上按仿真时间 deadline 调用虚拟 MCU。需要测并发、通信压力和主机卡顿时，测试适配器才映射到独立线程；结果仍必须与同线程模式一致。
- 倍率：只改变墙钟上执行多少已经定义的事件，不改变 MCU period、命令 expiry、时间戳或事件顺序。
- 安全：主机和 MCU 都有 watchdog。主机停止时，MCU 在本地 expiry 内直接进入 safe stop，不依赖主机再次发命令。

### 调试调度和真机调度

SIM 的默认实现就是单线程。一次 `ScenarioSession::step_once()` 按固定顺序完成物理
quantum、传感器发布、ROBOT tick、setpoint 投递和 Virtual MCU deadline；可以在
`VirtualMcu::tick()` 上断点，逐步执行一个控制周期，或把同一周期重复回放。这里的
“独立控制域”表示它有自己的周期、expiry、sequence 和 watchdog 合同，不表示必须
创建一个操作系统线程。单线程是调试和确定性回放的首选；并发压力测试才启用线程
适配器，而且必须通过与单线程模式相同的 accepted sequence、反馈和终态测试。

真机不把这个单线程实现直接当作实时 MCU。Linux/ROS 主机负责 ROBOT 决策和非阻塞
通信，传感器由 DMA/中断或驱动采集任务提供带 acquisition timestamp 的数据，底盘
MCU 由硬件定时器或 RTOS 固定优先级任务运行 PID、expiry 和本地机械停车。三者共享
typed setpoint/measurement 合同，但调度器属于各自平台；墙钟只在真机最外层 pacing
和 watchdog 使用，不能改变 ROBOT 算法输入的时间戳。

## 边界

IProvider 描述 ROBOT 进程内一个有明确 role 的端点（LiDAR、IMU、encoder 或 actuator）。远端 MCU 固件不是 C++ 进程中的 provider。主机硬件包实现 INativeAdapter，负责注册 endpoint、翻译标准消息、轮询测量、写入标准命令和独立机械停车。需要 typed MCU 协议时，适配器实现 `IChassisMcuTransport`；它提供 capability、发送 chassis setpoint、轮询 MotionFeedback 和独立机械停车，但不是 IProvider。`NativeRuntime` 已支持这条可选 transport：未提供时保持现有 duty adapter 路径，提供时发送 sequence/expiry 完整的 typed setpoint，并在 bring-up 拒绝非 host-motion ownership，避免未来 body/joint 转换未完成时重复闭环。

固件不能接触 RobotApplication、estimator、planner、SIM truth、actor identity、score 或渲染状态。它只处理已经定义的 setpoint、编码器/IMU/执行反馈、故障和本地安全状态。

## 单一外层闭环

控制所有权必须由版本化硬件能力和配置共同确定：

| ownership | 主机输出 | MCU 负责 | 主机关闭 |
| --- | --- | --- | --- |
| host motion loop | 当前 ActuatorCommand 的 duty/steer | 传输保护、限幅、过期停车和板级内环 | 无；不得重复同一外环 |
| MCU joint loop | 每轮角速度、转向角和制动意图 | 轮速/转向位置闭环、编码器和本地停车 | host wheel/steer PID |
| MCU body loop | 已验证的 body twist 或 stop | 底盘运动学、joint targets、joint loop 和本地停车 | host chassis 外环和重复运动学闭环 |

主机和 MCU 的 ownership、底盘模式、关节数量、profile id/revision 不一致时，bring-up 失败；运行中失配时进入机械 safe stop。任何板级电流环必须标记为 MCU 内环，不能冒充同一层的主机外环。

## Typed capability 和 setpoint

已在 `robot/iface/motion.hpp` 增加正式能力描述，不把单位偷偷塞进现有 duty
数组：

 `ChassisControlOwner`、`ChassisCapabilities`、`ChassisSetpointKind` 和
 `ChassisSetpoint` 现在是接口类型（保留 `Motion*` 源兼容别名）；
 `valid_chassis_capabilities` 与 `valid_chassis_setpoint` 在
 `lost_signal_robot_iface` 边界执行 owner、模式、
 关节计数、frame、expiry、profile revision 和字段单位校验。duty 使用独立的
 `drive_duty`/`steer_duty` 字段，速度/角度 setpoint 使用 `drive_radps`/
 `steer_rad`，因此 transport 无法把同一数组解释成不同单位。

 ```cpp
 enum class ChassisControlOwner { kHostMotionLoop, kMcuJointLoop, kMcuBodyLoop };

 struct ChassisCapabilities {
 ChassisControlOwner control_owner;
      ActuatorMode chassis_mode;
      uint8_t drive_count;
      uint8_t steer_count;
      MonotonicNs control_period_ns;
      MonotonicNs feedback_period_ns;
      MonotonicNs maximum_command_age_ns;
      string profile_id;
      uint32_t profile_revision;
    };

 enum class ChassisSetpointKind { kActuatorDuty, kJointVelocity, kBodyTwist, kStop };

 struct ChassisSetpoint {
 ChassisSetpointKind kind;
      uint64_t sequence;
      MonotonicNs stamp_ns;
      MonotonicNs expiry_ns;
      Frame frame;
 array<double, 4> drive_duty;
 array<double, 4> steer_duty;
      array<double, 4> drive_radps;
      array<double, 4> steer_rad;
      double forward_mps;
      double lateral_mps;
      double yaw_rate_radps;
      double service_brake;
    };
 ```

kind 唯一决定有效字段，其他字段必须为零或按校验规则忽略；不得按数组位置猜单位。MCU 回显最近接受的 sequence、应用时刻、拒绝原因和制动状态。

## 时间和通信语义

SIM 的执行顺序固定为：完成一个物理 quantum，推进 sim time，判断 MCU deadline，消费带 session/sequence 的传输队列，执行一次 MCU tick，再产生带 acquisition stamp 的反馈。命令在送达和 MCU tick 两个边界检查 expiry。不可表示为物理网格整数倍的生产周期拒绝加载；延迟、丢包和乱序使用 sim-time 队列建模，不能用 sleep 决定结果。

真机主机决策线程产生有 expiry 的 setpoint；I/O 线程只做非阻塞收发、重连和健康状态；MCU 在自己的固定周期消费最新合法命令。超过 expiry、sequence 回退、session 变化、CRC/version 错误或本地故障时不执行旧命令。启动必须经过 handshake、profile/ownership/关节布局校验和 zero-state 自检，失败回到 safe hold。

反馈保留传感器自己的 acquisition header，不把接收时刻改写成采集时刻。执行反馈至少包括最近接受 sequence、应用时刻、applied setpoint、watchdog、限位、过流、过温、饱和和传输丢包诊断。这些是 ROBOT 的执行证据，不是 SIM 真值。

## 真机适配路径

供应商只提供一个硬件包，不复制 ROBOT 算法：

1. 实现 CAN、EtherCAT、串口、共享内存或厂商 SDK transport。
2. 提供 device profile：ownership、底盘模式、joint layout、周期、单位、限位、expiry、故障码和 revision。
3. 实现 INativeAdapter 的 endpoint 注册、标准 observation 轮询、标准 setpoint 写入和 mechanical_safe_stop。
4. 将 vendor 数据翻译为标准消息，不把私有结构透传到 chassis、planner 或 Safety。
5. 运行与 SIM 相同的 provider conformance、运动学、expiry、watchdog、重连和任务回放测试。

部署入口通过版本化 plugin factory 或显式构建选择加载硬件包；RobotApplication 不包含设备型号分支。

## 实施阶段

1. 契约：增加 chassis ownership、capability 和 typed setpoint；现有 SIM/native 默认保持 host ownership。
2. SIM MCU：`ScenarioSession` 已通过 `VirtualMcu` 把默认 host-duty chassis
 setpoint 放入 sim-time deadline/transport queue；过期、session 失配、sequence
 回退和本地 watchdog 都在 MCU 边界拒绝或停车，物理引擎只消费已接受的命令。
 `test_virtual_mcu` 和 `signal_town_mission` 已通过；测试还比较了逐 1 ms
 调用和 5 ms 批量 transport 调用在相同 sim time 的最终 accepted sequence/state，
 证明调度方式不改变结果。下一步再增加 joint/body ownership 的转换 adapter。
3. Native fake transport：用 fake board 跑 bring-up、过期、乱序、断线、重连和独立停车。主机侧 `IChassisMcuTransport` seam 与 native fake board 单测已在 `test_native_runtime` 中建立。
4. HIL：将固定周期、延迟、丢包和 MCU watchdog 对应到真实板卡。仓库提供
 `examples/stm32_chassis_mcu` 的 HAL-free reference：body twist 在板端转成
 差速轮速，固定 `control_period_ns` 调 PID，使用 conditional anti-windup，
 encoder 是唯一闭环反馈，expiry 触发本地机械停车；独立 CMake host test 已通过。
5. Hardware profile：用实测 step response 填入能力参数；未测量时保持 safe hold，不能用猜测的默认数字上线。

仓库提供两组可以直接放进 CubeMX 工程的无 HAL 核心：
`examples/stm32_chassis_mcu/` 负责固定周期底盘 PID、运动学、expiry 和本地
机械停车；`examples/stm32_sensor_bridge/` 负责 LiDAR/IMU/编码器固定宽度报文的
版本、boot、sequence、时间戳和有限值校验，并附带一个 native 侧
`ObservationAdapter` 把接受的测量转换为 `ObservationFrame`。两组核心都在主机上
有确定性测试，也可以用 `-DSTM32_FIRMWARE=ON` 编译成嵌入式库。

它们是可移植到真实板卡的参考实现，不是假装已经适配某一块 STM32：CubeMX 工程仍
必须提供实际的时钟、DMA/中断、CAN/UART framing/CRC、引脚、电机驱动、急停输入和
经过测量的底盘/传感器 profile。没有这些实测参数时，启动保持 safe hold；不能把示例
里的轮径、PID 或量程当成通用魔法默认值。

## 验收

- 1x、8x、64x、Turbo、headless 的 MCU 事件序列和结果一致。
- 主机决策停顿时 MCU 仍能在本地 expiry 内停车。
- 三种 ownership 各有单测，证明只有一个外层 PID 生效。
- capability/profile 不匹配时 bring-up 失败；旧 session 的迟到命令永不执行。
- SIM、HIL、真机共用 typed setpoint、measurement header、conformance 和 safe-stop 语义。
- ROBOT 边界扫描确认没有 SIM truth、actor identity、future script、score 或 render state。
