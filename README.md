# Lost Signal — clean rebuild (`refactor/clean-rebuild`)

This branch is a complete rewrite. No backward compatibility with `main`.

## Anchor

**ROBOT is the anchor.** Everything deployable lives under `src/robot/` and
must build without any `src/sim/` header, source, or library.

```
src/robot/   deployable: core, iface, autonomy, executive, app
src/sim/     non-deployable: world truth, providers, scenario/session
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

Requires gcc-13 (full C++20). The default preset pins the compiler:

```sh
cmake --preset default
cmake --build --preset default
ctest --test-dir build -V
```

## Gate

`tests/check_boundary.py` fails the build if any file under `src/robot/`
includes a `sim/` header or if a `lost_signal_robot_*` target links a
`lost_signal_sim_*` library. `ctest --test-dir build` currently runs 47
tests.

## Headless run

```sh
./build/lost_signal_sim --map configs/worlds/signal_town.json \
  --steps 600 --out /tmp/run.json
./build/lost_signal_sim --map configs/worlds/signal_town.json \
  --objectives place_a,place_b --duration 60 --out /tmp/run.json
./build/lost_signal_sim --map configs/worlds/signal_town.json \
 --steps 600 --out /tmp/run.json --episode /tmp/run.jsonl
```

`--robot` and `--sim` select the declared hardware contract and the SIM
cadence. `--actors` overrides the sibling `<map>.actors.json` sidecar
(loaded automatically by default), `--no-actors` disables it. The runner
prints steps, objective completion, collisions, distance, and estimate
error; `--out` writes the same data plus per-objective results as JSON.
`--episode` writes the deterministic observation stream accepted by
`ReplayProvider` for closed-loop offline replay.
Without `--steps` or `--duration`, an explicit objective list gets a 4000
step budget and a single implicit goal keeps the legacy 5 step smoke run.

## Capability notes

- Differential and tracked bases run the full mission path. Ackermann now uses
 a ROBOT-side Hybrid-A* global route plus DWA candidates bounded by the
 declared steering curvature; it never turns in place. Mecanum and swerve use
 a slew-limited lateral command window when their declared `max_lateral_mps`
 is non-zero.
- DWA samples forward and reverse velocity. The validator checks a
  continuous swept footprint (point-to-segment), not just discrete poses.
- Scan consumers share the `angle_min + i * increment` layout helper;
  an unset increment falls back to the legacy full rotation.
- SLAM submaps exclude returns that disagree with the static reference, so
  moving actors do not become map geometry.
- Sensor delivery realism is opt-in. Decision time is the plant clock;
  observation stamps stay at acquisition, so latency can trip freshness.
- Scoring is offline only and includes displacement, path length, stops,
 dynamic holds, terrain/static/dynamic collisions, estimate error, elapsed
 time, clearance, objective completion ratio, and a duty/speed energy proxy.
 Coordinator remains a sequencing seam; route refresh, motion decisions, and
 conflict prediction live in autonomy modules. Conflict-zone resource policy
 and Hybrid A* path smoothing remain separate extension points.
