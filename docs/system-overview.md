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
    SProv["sim_providers<br/>SimPlatform + Sim2DLidar + Sim16Lidar"]
    Eval["sim_eval<br/>EpisodeRecorder + scoring"]
    Game["sim_game<br/>ScenarioSession + SimRobot"]
    World --> SProv
    World --> Game
    SProv --> Game
  end

  Stack["robot_stack (INTERFACE)<br/>app + robots + providers"]
  App -.-> Stack
  Rob -.-> Stack
  Prov -.-> Stack

  Game -->|"depends on"| Stack
  SProv -->|"depends on"| Stack
  Eval -->|"depends on"| Stack

  BinR["lost_signal_robot<br/>native binary, links stack only"]
  BinS["lost_signal_sim<br/>sim binary, links sim_game"]
  Stack --> BinR
  Game --> BinS

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
    W["WorldTruth"] -->|"truth → measurement"| P["Providers<br/>SimPlatform / Sim2DLidar"]
    P -->|"ObservationFrame<br/>typed, validated"| Ch(["RuntimeChannel<br/>bounded queue"])
    Ch -->|"drain"| A["RobotApplication::tick()<br/>Coordinator → safe command"]
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
  participant L as Sim2DLidar / Sim16Lidar
  participant P as SimPlatformProvider
  participant C as RuntimeChannel
  participant R as RobotApplication
  W->>L: truth (sim-internal)
  L->>L: synthesize Lidar2DScan / Lidar3DCloud
  W->>P: truth (sim-internal)
  P->>P: synthesize ObservationFrame (odom/imu/scan)
  L->>R: scan ranges packed into ObservationFrame
  P->>C: publish_observation (registry + seq/stamp gate)
  C->>R: drain_observations in tick()
  R->>R: coordinator.ingest_observation
```

- 注册门：`register_provider` 先占位，未注册 source 直接拒收。
- 顺序门：每 source `sequence + stamp` 严格递增，迟到/重复计数 `obs_rejected`。
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

v1 骨架 planner 恒发 safe-stop；包络、过期、空队列全部 fail-closed。

## 5. 任务意图流（operator / game → robot）

```mermaid
flowchart LR
  Op["Operator / Game / Test"] -->|"TaskRequest{request_id, place_id}"| Exe["RobotExecutive::submit"]
  Op -->|"request_map(map_id)"| Map["IMapProvider::select_map"]
  Exe -->|"TaskStatus"| Op
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

- `AutonomyCoordinator::estimate`（v1 里程计航位推算）随 `diagnostics()` 只读带出。
- 对比公式：`pos_err = hypot(est.x - truth.x, est.y - truth.y)`，
  `yaw_err = |wrap_pi(est.yaw - truth.yaw)|`（`sim/eval/scoring.cpp`）。
- 测试 `test_truth_isolation` 断言：估计存在、误差有限、reward 纯离线、
  无 obs 改真值不影响指令。

## 7. 文件 → 模块对照表

| 模块（CMake target） | 目录 | 关键文件 |
|---|---|---|
| `robot_core` | `src/robot/core/` | `time.hpp`, `frames.hpp`, `status.hpp` |
| `robot_iface` | `src/robot/iface/` | `version.hpp`, `message.hpp/cpp` (`IMessage`, `Lidar2DScan`, `Lidar3DCloud`), `observation.hpp`, `command.hpp`, `endpoint.hpp/cpp`, `channel.hpp/cpp`, `platform.hpp` (`IPlatformProvider`), `provider.hpp` (`IProvider`, `ISensorProvider`), `task.hpp`, `map.hpp` |
| `robot_platform` | `src/robot/platform/` | `description.hpp/cpp` (`RobotDescription`), `chassis.hpp/cpp` (`IChassis`, `DifferentialChassis`) |
| `robot_autonomy` | `src/robot/autonomy/` | `coordinator.hpp/cpp` (`AutonomyCoordinator`, `EstimatedState`) |
| `robot_executive` | `src/robot/executive/` | `executive.hpp/cpp` (`RobotExecutive`) |
| `robot_app` | `src/robot/app/` | `robot.hpp` (`IRobot`), `robot_application.hpp/cpp` (+`Diagnostics`) |
| `robot_robots` | `src/robot/robots/` | `differential_robot.hpp/cpp` |
| `robot_providers` | `src/robot/providers/` | `real_lidar2d.hpp/cpp` |
| `sim_world` | `src/sim/world/` | `world.hpp/cpp` (`WorldTruth`, 真值唯一住所) |
| `sim_providers` | `src/sim/providers/` | `sim_platform.hpp/cpp`, `lidar2d.hpp/cpp`, `lidar16.hpp/cpp` |
| `sim_eval` | `src/sim/eval/` | `episode.hpp` (`EvalStep`, `EpisodeRecorder`), `scoring.hpp/cpp` |
| `sim_game` | `src/sim/game/`, `src/sim/robot/` | `scenario_session.hpp/cpp`, `sim_robot.hpp/cpp` |
| binaries | `src/robot/app/main.cpp`, `src/sim/game/sim_main.cpp` | `lost_signal_robot`, `lost_signal_sim` |
| tests | `tests/` | `test_robot_cycle/transport/truth_isolation/chassis/lidar/sim_robot` + `check_boundary.py` |

新增模块时：ROBOT 侧只加 `src/robot/` + `robot_*` target + 接口测试；
SIM 侧加 `src/sim/` + `sim_*` target；真值类型永不进 `src/robot/`。
