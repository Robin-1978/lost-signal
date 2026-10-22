# Lost Signal — clean rebuild (`refactor/clean-rebuild`)

This branch is a complete rewrite. No backward compatibility with `main`.

## Anchor

**ROBOT is the anchor.** Everything deployable lives under `src/robot/` and
must build without any `src/sim/` header, source, or library.

```
src/robot/   deployable: core, iface, autonomy, executive, app
src/sim/     non-deployable: world truth, providers, game/session
tests/       boundary gate + robot cycle test
```

## Boundary (non-negotiable)

1. SIM owns: MuJoCo/physics state, actor truth, authored scenario facts,
   rendering, scoring, replay truth, `MissionSnapshot`.
2. ROBOT owns: estimation, perception, mapping, prediction, planning,
   trajectory validation, control, safety decisions, task/resource policy.
3. SIM → ROBOT crosses only through:
   - `RobotApplication::submit_task()` / `request_map()` (typed task intent),
   - `publish_observation()` (LSRI `ObservationFrame`),
   - `RobotApplication::tick()` returning a validated command.
4. ROBOT → SIM crosses only through:
   - validated `MotionCommand` / `ActuatorCommand` with expiry,
   - immutable `MapSnapshot` revisions and `TaskStatus`.
5. Simulator truth may be recorded as an offline label/score. It must never
   correct or enter the same-cycle ROBOT decision path.

## Build

```sh
cmake -S . -B build
cmake --build build -j8
ctest --test-dir build -V
```

## Gate

`tests/check_boundary.py` fails the build if any file under `src/robot/`
includes a `sim/` header or if a `lost_signal_robot_*` target links a
`lost_signal_sim_*` library.
