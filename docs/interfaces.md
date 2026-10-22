# Interfaces: IRobot / IMessage / IProvider

Three independent axes. Each has one deployable interface in `src/robot/`
and deployment-specific implementations. Tests program against the
interfaces, never the concrete classes.

## IRobot — robot facade

`src/robot/app/robot.hpp`. Task/map intent in, validated command out.

```mermaid
flowchart TB
  IRobot["IRobot<br/>submit_task / request_map<br/>publish_observation / tick / take_command"]
  App["RobotApplication<br/>src/robot/app/<br/>composition root"]
  Diff["DifferentialRobot<br/>src/robot/robots/<br/>+ chassis clamp"]
  SimR["SimRobot<br/>src/sim/robot/<br/>world + sim providers"]
  IRobot --> App
  App --> Diff
  IRobot --> SimR
```

- `RobotApplication`: owns Executive + Coordinator + channel + map + description.
- `DifferentialRobot`: wraps any `IRobot`, clamps every tick through
  `DifferentialChassis` (`src/robot/platform/chassis.hpp`). Ackermann and
  holonomic robots follow the same wrapper pattern.
- `SimRobot`: sim harness implementing the same facade (world-stepping tick,
  `src/sim/robot/sim_robot.hpp`). Truth stays inside; only LSRI crosses.

## IMessage — typed sensor messages

`src/robot/iface/message.hpp`. Every message carries
`schema_version / stamp_ns / source / sequence / frame` and `valid()`.

- `Lidar2DScan`: planar ranges + angular spacing. Absent return, never
  max-range fill.
- `Lidar3DCloud`: one rotation, `ring_count` rings, per-point
  `time_offset_s` in `[-scan_period, 0]`.

Wire-agnostic: in-process struct today, serialized form tomorrow. Version
bump on meaning change (`version.hpp`).

## IProvider — sensor providers

`src/robot/iface/provider.hpp`. Non-template `IProvider` base
(`descriptor() + is_ready()`) plus `ISensorProvider<T>` typed `poll()`.

```mermaid
flowchart LR
  IP["ISensorProvider<T>"] --> S2D["Sim2DLidar<br/>src/sim/providers/"]
  IP --> S16["Sim16Lidar<br/>src/sim/providers/"]
  IP --> R2D["Real2DLidar<br/>src/robot/providers/<br/>replay/native, no SDK"]
```

Same seam, backend flag differs (`kSim` vs `kNative`). `SimRobot` packs the
typed scan into the LSRI `ObservationFrame`; `Real2DLidar` replays recorded
rotations (nothing polled before `feed()`).

## IChassis — chassis kinematics (bonus axis)

`src/robot/platform/chassis.hpp`. `DifferentialChassis` implements
`inverse()` / `forward()` wheel kinematics and `clamp_command()` envelope.
`DifferentialRobot` applies the clamp; a planner can never publish what the
base cannot track.
