# Source layout and ownership

The repository has two deliberately separate products. `src/robot` is the
deployable autonomy library; `src/sim` is a simulator, evaluation harness and
viewer that composes the library through providers. Code in `src/robot` must
compile without MuJoCo, Raylib, actor sidecars, world truth or score types.

## Robot library

- `robot/iface` defines timestamped observations, commands, providers,
  chassis ownership and transport contracts. It is the stable seam used by
  both native adapters and the simulator.
- `robot/platform` owns robot description, geometry, chassis kinematics,
  hardware profiles, calibration artifacts and transform-tree validation.
- `robot/providers` contains deployable sensor adapters. Protocol framing and
  non-blocking I/O live in `robot/transport`; a serial, UDP, CAN, SPI or USB
  device is an adapter around an interface endpoint, never a simulator hook.
- `robot/autonomy` is grouped by responsibility: observation and estimation,
  localization/SLAM, mapping, perception, planning, validation, safety,
  decision and diagnostics. Each module consumes interface measurements or
  robot-owned estimates and exposes its own tests.
- `robot/app`, `robot/executive` and `robot/lifecycle` are composition and
  lifecycle seams. `Coordinator`/application code sequences modules and owns
  no planner, matcher or safety policy.
- `robot/control` contains low-level host/MCU controllers. `robot/robots` is a
  thin deployment facade retained for native integration; chassis selection
  remains data-driven through `RobotDescription::chassis_kind`.

## Simulation library

- `sim/world` is the MuJoCo plant and its contact, terrain, actor and joint
  read-back state. This is the only owner of physical truth.
- `sim/providers` turns the plant into timestamped sensor and actuator
  endpoints. Providers publish measured joints, scans and IMU data through the
  same interface contracts a real device uses.
- `sim/scenario` composes a deterministic simulation-time session, objectives,
  actors and the virtual MCU. It records truth for offline scoring only.
- `sim/eval` loads authored maps/configuration and computes episode artifacts
  and verdicts. Its results never flow back into `src/robot`.
- `sim/experiment` owns immutable episode pairs, controlled variables and
  chapter runtime construction.
- `sim/viewer` is presentation only. It consumes recorded frames and MuJoCo
  presentation snapshots for 2D/3D rendering, replay, effects and audio. It
  must never write an observation, estimate or command.

## Test ownership

Unit tests under `tests/` target one robot or simulator module. Mission tests
(`signal_town`, `signal_town_big`, terrain, dynamic crossing and industrial)
are integration gates and diagnostics; they are not substitutes for planner,
localization, safety or truth-isolation tests. Viewer and asset tests use the
same frame/presentation snapshot path as live and replay rendering.

When a source file appears unused, first check whether it is a public adapter
or a test-facing contract. Remove only a private implementation that has no
callers; do not delete the serial, MCU, chassis or planner seams merely
because the simulator does not instantiate every hardware variant.
