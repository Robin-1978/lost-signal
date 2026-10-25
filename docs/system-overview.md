# System overview: modules + data flow

> 与代码一一对应。库名 = `CMakeLists.txt` target 名，括号里是源文件目录。

## 1. 全局模块架构

```mermaid
flowchart TB
  subgraph ROBOT["ROBOT deployable — src/robot (never includes sim)"]
    direction TB
    Core["robot_core<br/>core/time, frames, status"]
    Iface["robot_iface<br/>version, message, observation,<br/>command, endpoint, channel,<br/>platform, provider, task, map"]
    Plat["robot_platform<br/>description, chassis"]
    Aut["robot_autonomy<br/>coordinator + EstimatedState"]
    Exe["robot_executive<br/>executive"]
    App["robot_app<br/>IRobot + RobotApplication"]
    Rob["robot_robots<br/>DifferentialRobot"]
    Prov["robot_providers<br/>Real2DLidar (replay/native)"]
    Core --> Iface
    Iface --> Plat
    Iface --> Aut
    Iface --> Exe
    Aut --> App
    Exe --> App
    Plat --> App
    App --> Rob
    Iface --> Prov
  end

  subgraph SIM["SIM only — src/sim (may depend on robot)"]
    direction TB
    World["sim_world<br/>World + WorldTruth"]
    SProv["sim_providers<br/>SimSensorSuite + SimMotion + Sim2DLidar + SimImu"]
    Eval["sim_eval<br/>EpisodeRecorder + scoring"]
    Scn["sim_scenario<br/>ScenarioSession + SimRobot"]
    World --> SProv
    World --> Scn
    SProv --> Scn
  end

  Stack["robot_stack (INTERFACE)<br/>app + robots + providers"]
  App -.-> Stack
  Rob -.-> Stack
  Prov -.-> Stack

  Scn -->|"depends on"| Stack
  SProv -->|"depends on"| Stack
  Eval -->|"depends on"| Stack

  BinR["lost_signal_robot<br/>native binary, links stack only"]
  BinS["lost_signal_sim<br/>sim binary, links sim_scenario"]
  Stack --> BinR
  Scn --> BinS

  Tests["tests/<br/>robot_cycle, transport, truth_isolation,<br/>chassis, lidar, sim_robot<br/>+ check_boundary gate"]
  Tests -.->|"asserts boundary"| ROBOT
```

依赖方向只有一条：`sim → robot`，绝无反向。门禁见
`tests/check_boundary.py`（禁 `sim/` include、禁真值 token、禁 robot 链 sim 库）。

## 2. 数据流总览（一张图）

```mermaid
flowchart LR
  subgraph Live["Live loop (same-cycle, in-process)"]
    direction LR
    W["WorldTruth"] -->|"truth → measurement"| P["Providers<br/>SimSensorSuite / SimMotion / Sim2DLidar / SimImu"]
 P -->|"ObservationFrame<br/>per-section acquisition headers"| Ch(["RuntimeChannel<br/>bounded queue"])
 Ch -->|"drain"| A["RobotApplication::tick()<br/>Synchronizer → modules → safe command"]
    A -->|"MotionCommand<br/>expiry enforced"| Ch2(["RuntimeChannel<br/>cmd queue"])
    Ch2 -->|"take_command"| P
    P -->|"apply"| W
    T["TaskRequest<br/>place_id only"] --> A
  end
  subgraph Offline["Offline loop (post-run, never feeds tick)"]
    direction LR
    W -->|"record"| Rec[("EpisodeRecorder<br/>truth_before/after + obs + cmd + diag")]
    A -->|"diagnostics()<br/>estimate (read-only)"| Rec
    Rec --> S["score_episode<br/>estimate_error<br/>reward_step"]
    S -->|"tune estimator<br/>train RL out-of-process"| Opt["Algorithm optimization"]
    Opt -.->|"new code, not data"| A
  end
```

真值只出现在左侧 `WorldTruth → Providers → measurement` 和右侧离线记录里，
永不直达 `tick()`。优化回路是虚线：人/训练改代码，不回灌数据。

## 3. Live 观察流（provider → robot）

```mermaid
sequenceDiagram
  participant W as World (sim)
  participant L as Sim2DLidar / SimImu
  participant P as SimSensorSuite
  participant C as RuntimeChannel
  participant R as RobotApplication
  W->>L: truth (sim-internal)
  L->>L: synthesize Lidar2DScan / Lidar3DCloud
  W->>P: truth (sim-internal)
  P->>P: synthesize ObservationFrame (odom/imu/scan)
  L->>R: scan ranges packed into ObservationFrame
  P->>C: publish_observation (registry + seq/stamp gate)
  C->>R: drain_observations in tick()
 R->>R: coordinator sequences MeasurementSynchronizer and modules
```

- 注册门：`register_provider` 先占位，未注册 source 直接拒收。
- 顺序门：transport 以及 scan/motion/IMU section 各自按 source 的
 `sequence + stamp` 检查；迟到或重复 measurement 不会倒退估计时钟。
- 容量门：obs 64 满时丢最老，记 `obs_dropped`（degraded 可见）。

## 4. Live 指令流（robot → provider → plant）

```mermaid
sequenceDiagram
  participant R as RobotApplication
  participant D as DifferentialRobot wrapper
  participant C as RuntimeChannel
  participant P as Provider (sim or native)
  participant W as World / Driver
  R->>R: coordinator.update → MotionCommand
  R->>C: publish_command
  C->>D: tick returns clamped command
  D->>D: chassis.clamp_command (envelope)
  P->>C: take_command(now) — expired → SafeStop
  C->>P: newest non-expired command
  P->>W: apply to plant
```

`AutonomyCoordinator` is the ROBOT sequencing seam. Measurement ordering,
deduplication, caching, IMU frame conversion and one-shot consumption live in
`MeasurementSynchronizer`; planning policy lives in the planning and decision
modules. The coordinator only passes module outputs to the next module and
holds lifecycle state. Safe stop comes out of the gates (no
observation, invalid command, safety trip, controlled-stop latch), never
from a skeleton stub. 包络、过期、空队列全部 fail-closed。

## 5. 任务意图流（operator / scenario → robot）

```mermaid
flowchart LR
  Op["Operator / Scenario / Test"] -->|"TaskRequest{request_id, place_id}"| Exe["RobotExecutive::submit"]
  Op -->|"request_map(map_id)"| Map["IMapProvider::select_map"]
  Exe -->|"GoalStatus"| Op
  Map -->|"MapSnapshot{map_id, revision}"| Coord["Coordinator::update"]
```

- 语义 place only：坐标由 ROBOT 侧 map 解析，SIM 不给坐标、不给 actor、不给路线。
- 未知 place → `Fail`，plan 永不携带真值。

## 6. 离线评估流（truth-vs-estimate，对比只为调优）

```mermaid
flowchart TB
  subgraph Run["Run (live, no truth in tick)"]
    S["ScenarioSession::step_once / SimRobot::tick"]
  end
  subgraph Record["Record (history)"]
    E[("EvalStep<br/>truth_before / truth_after<br/>obs + accepted + cmd<br/>diag.estimate")]
  end
  subgraph Score["Score (pure functions, post-run)"]
    EE["estimate_error(step)<br/>pos_err, yaw_err"]
    SE["score_episode<br/>mean/max pos/yaw err<br/>safe-stop/accept rates"]
    RW["reward_step<br/>RL label"]
  end
  S --> E
  E --> EE
  E --> SE
  E --> RW
  EE --> Tune["Tune estimator / planner"]
  SE --> Tune
  RW --> Train["Train RL out-of-process"]
```

- `AutonomyCoordinator::estimate`（fused odometry：gyro-led yaw +
  scan-match/SLAM corrections）随 `diagnostics()` 只读带出。
- 对比公式：`pos_err = hypot(est.x - truth.x, est.y - truth.y)`，
  `yaw_err = |wrap_pi(est.yaw - truth.yaw)|`（`sim/eval/scoring.cpp`）。
- 测试 `test_truth_isolation` 断言：估计存在、误差有限、reward 纯离线、
  无 obs 改真值不影响指令。

## 6b. 回放流（record → JSONL → deterministic replay）

- `export_episode` 把 `EpisodeRecorder` 落盘为 JSONL v3：一行 header
 `{"v":3,"kind":"episode","map":"<id>","rev":<n>}`，之后每步一行
 `step_to_json_line(record_step(obs))`（float `%.9g` / double `%.17g`，
 joints 按 `drives:[...]` + `steers:{pos,vel}` 变长记录各底盘布局，
 保留 scan/motion/IMU 各自的采集 header 以及可选的
 intensity/time-offset 扫描字段；导入后保持
 bit-identical round-trip）。
- `ReplayProvider::load` 兼容 v2/v3，读回 header + steps（坏行
 fail-closed，不抛异常），并公开回放文件实际包含的 endpoint 描述；
  `poll` 逐个吐出 live-equivalent `ObservationFrame`，播完返回 false。
- 闭环回放：新鲜 `RobotApplication` + 新鲜 plant，吃回放 obs，`take_actuator`
  驱动新 plant；确定性 stack 的指令序列应与录制 bit 一致，任何分歧都是
  非确定性 bug（`tests/test_replay.cpp` 第 4 节断言）。
- JSON 是手写最小子集（`sim/eval/json.{hpp,cpp}`，无第三方依赖），只支持
  记录格式需要的 object/array/string/number/true/false/null。

## 7. 文件 → 模块对照表

| 模块（CMake target） | 目录 | 关键文件 |
|---|---|---|
| `robot_core` | `src/robot/core/` | `time.hpp`, `frames.hpp`, `status.hpp` |
| `robot_iface` | `src/robot/iface/` | `version.hpp`, `message.hpp/cpp` (`IMessage`, `Lidar2DScan`, `Lidar3DCloud`, `ImuMessage`), `observation.hpp`, `command.hpp`, `endpoint.hpp/cpp`, `channel.hpp/cpp`, `motion.hpp` (`IMotionEndpoint`), `provider.hpp` (`IProvider`, `ISensorProvider`), `envelope.hpp` (canonical pack), `task.hpp`, `map.hpp` |
| `robot_platform` | `src/robot/platform/` | `description.hpp/cpp` (`RobotDescription`), `chassis.hpp/cpp` (`IChassis`, `DifferentialChassis`) |
| `robot_autonomy` | `src/robot/autonomy/` | `coordinator.hpp/cpp` (pipeline order) + `estimation/` (`IEstimator`) + `perception/` (`IPerception`) + `mapping/` (`IMapper`) + `planning/` (`IGlobalPlanner`, `ILocalPlanner`) + `validation/` (`IValidator`) + `control/` (`IController`). 详见 `docs/algorithms.md` |
| `robot_executive` | `src/robot/executive/` | `executive.hpp/cpp` (`RobotExecutive`) |
| `robot_app` | `src/robot/app/` | `robot.hpp` (`IRobot`), `robot_application.hpp/cpp` (+`Diagnostics`) |
| `robot_robots` | `src/robot/robots/` | `differential_robot.hpp/cpp` |
| `robot_providers` | `src/robot/providers/` | `real_lidar2d.hpp/cpp` |
| `sim_world` | `src/sim/world/` | `world.hpp/cpp` (`WorldTruth`, 真值唯一住所) |
| `sim_providers` | `src/sim/providers/` | `sensor_suite.hpp/cpp` (composer) + `sim_motion.hpp/cpp`, `sim_imu.hpp/cpp`, `lidar2d.hpp/cpp` |
| `sim_eval` | `src/sim/eval/` | `episode.hpp` (`EvalStep`, `EpisodeRecorder`), `scoring.hpp/cpp` |
| `sim_scenario` | `src/sim/scenario/`, `src/sim/robot/` | `scenario_session.hpp/cpp`, `sim_robot.hpp/cpp` |
| binaries | `src/robot/app/main.cpp`, `src/sim/scenario/sim_main.cpp` | `lost_signal_robot`, `lost_signal_sim` |
| tests | `tests/` | `test_robot_cycle/transport/truth_isolation/chassis/lidar/sim_robot` + `check_boundary.py` |

新增模块时：ROBOT 侧只加 `src/robot/` + `robot_*` target + 接口测试；
SIM 侧加 `src/sim/` + `sim_*` target；真值类型永不进 `src/robot/`。
