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
`lost_signal_sim_*` library. `ctest --test-dir build` currently runs 56
tests.

## Headless run

```sh
./build/lost_signal_sim --map configs/worlds/signal_town.json \
  --steps 600 --out /tmp/run.json
./build/lost_signal_sim --map configs/worlds/signal_town.json \
  --objectives place_a,place_b --duration 60 --out /tmp/run.json
./build/lost_signal_sim --map configs/worlds/signal_town.json \
 --steps 600 --out /tmp/run.json --episode /tmp/run.jsonl
./build/lost_signal_sim --map configs/worlds/signal_town.json \
 --steps 1000 --profile
```

`--robot` and `--sim` select the declared hardware contract and the SIM
cadence. `--actors` overrides the sibling `<map>.actors.json` sidecar
(loaded automatically by default), `--no-actors` disables it. The runner
prints steps, objective completion, collisions, distance, and estimate
error; `--out` writes the same data plus per-objective results as JSON.
`--episode` writes the deterministic observation stream accepted by
`ReplayProvider` for closed-loop offline replay.
`--profile` reports p50/p95/p99/max wall time for physics, sensors, the ROBOT
tick, actuation, recording, and the complete step. This timing remains in the
SIM runner and is never published into the ROBOT decision path.
Without `--steps` or `--duration`, an explicit objective list gets a 4000
step budget and a single implicit goal keeps the legacy 5 step smoke run.

## Capability notes

- Differential and tracked bases run the full mission path. Ackermann now uses
 a ROBOT-side Hybrid-A* global route plus local candidates bounded by the
 declared steering curvature; it never turns in place. Robot JSON selects
 `"local_planner": "dwa"` (default) or `"teb"`; both consume the same
 ROBOT-side dynamic prediction and independent validator. Mecanum and swerve use
 a slew-limited lateral command window when their declared `max_lateral_mps`
 is non-zero.
- DWA samples forward and reverse velocity. TEB optimizes a short route-aligned
 position/time band. The validator checks both static swept footprint and
 continuous relative motion against predicted tracks, not just discrete poses.
 Position and yaw remain separate terminal constraints. Differential and
 holonomic bases settle the requested final yaw inside the position tolerance;
 Ackermann follows the route heading and remains bounded by
 `|yaw_rate| <= |speed| * curvature`.
- Arrival is a versioned ROBOT machine policy rather than a scenario radius.
 The declared probability converts estimator covariance to position/heading
 confidence radii; error plus confidence radius must fit the configured pose
 tolerance while measured motion remains below the configured settle limits.
 SIM independently records physical miss distance after ROBOT reports Done.
- Global A*, Hybrid-A*, DWA, and TEB share a two-stage costmap. The hard layer
 expands known obstacles by the declared footprint plus the robot JSON
 `navigation.clearance_margin_m`; the outer
 `soft_inflation_radius_m` band adds a linear preference for usable wall
 clearance. Hybrid-A* sweeps its bicycle primitives through the full envelope.
 Map revisions trigger a fresh global search; dynamic actors are handled by
 the rolling local map and predictor rather than a D* Lite implementation.
- Behavior time-parameterizes the active route against the same anonymous
 dynamic hypotheses, then derives a hold or speed cap from braking distance.
 A hold is emitted only when staying still is also forecast safe; otherwise
 the local planner remains free to choose a retreat or lateral escape.
- Scan consumers share the `angle_min + i * increment` layout helper;
  an unset increment falls back to the legacy full rotation.
- SLAM submaps exclude returns that disagree with the static reference, so
  moving actors do not become map geometry.
 Explicit SLAM-only startup anchors the ROBOT odometry estimate once and then
 builds/localizes in its own map; unknown cells remain unknown until sensor
 rays or SLAM observations clear them.
- Sensor delivery realism is opt-in. Decision time is the plant clock;
  observation stamps stay at acquisition, so latency can trip freshness.
- The robot JSON is the shared geometry and inertia contract for the physical
  shell, Viewer body, planner footprint, wheel mass, payload, and centre of
  mass. MuJoCo compiles a fixed collision shell plus a contactless internal
  ballast; changing COM never moves the belly. Builds fail closed when the
  footprint does not enclose the shell or the declared vehicle mass cannot
  cover its wheels, knuckles, and supports. Tests read back total compiled
  mass and whole-system COM from MuJoCo.
- Passive supports are explicit `caster` assemblies with their own base-frame
 pivot, trail, radius, mass, contact friction, rolling-friction length,
 swivel damping, and swivel friction. MuJoCo owns fork swivel and wheel roll;
 the Viewer reads those joint poses. Ackermann idler trail is a separate
 parameter and can no longer alter caster geometry.
- A MuJoCo rebuild restores state by stable joint name, joint type, and state
  dimension. Compatible base, drive, steering, caster, and future arm joints
  keep their positions and velocities when topology changes; new joints retain
  their authored defaults and removed joints are discarded. Root height alone
  is raised when required to clear the rebuilt terrain/contact envelope.
- Scoring is offline only and includes displacement, path length, stops,
 dynamic holds, terrain/static/dynamic collisions, estimate error, elapsed
time, planar obstacle clearance, separate terrain/belly clearance, objective
completion ratio, and a duty/speed energy proxy.
 Coordinator remains a sequencing seam; route refresh, motion decisions, and
 conflict prediction live in autonomy modules. The old authored conflict-zone
 authority stays SIM-only; its deployable equivalent is the route/prediction
 entry constraint above. Hybrid A* path smoothing remains in planning.
- The SIM viewer is presentation-only. It loads optional GLB assets from
 `assets/models` automatically (or `--assets DIR`) through the versioned
 `assets/models/manifest.json` contract. Every model declares its units,
 axes, pivot, link transform, reference dimensions, expected bounds, and
 source. Missing metadata or changed bounds fail visibly and use deterministic
 procedural geometry instead of guessed rotations or sizes. Blender headless
 validation covers the complete model inventory. Chassis parts use live MuJoCo joint angles/velocities;
  wheel effects use MuJoCo contact-point slip, normal force, and the SIM-only
  ground material. Dry paving stays clean during ordinary rolling; sand,
  gravel, mud, water, and tire slip render distinct deterministic effects.
  These values never enter ROBOT observations.
In 3D map mode the viewer starts with a close robot-follow camera and a clean
world view. Press `F` to toggle follow; right-drag or middle-drag releases
follow for free orbit/pan, and `R` restores the default orbit. `L` shows a
sparse LiDAR diagnostic, `E` shows truth/estimate history, and `K` reveals
SIM-only actor script routes. Script routes are hidden by default because they
are not information available to ROBOT.
The viewer advances in deterministic 20 ms simulation steps at 1x by default.
Use `--rate N` or `+`/`-` for 0.05x–64x playback, and `--turbo` or `T` to run
as fast as the per-frame work budget allows. Playback never changes physics,
sensor stamps, or ROBOT control periods. The HUD reports requested rate,
measured real-time factor, steps/s, sliding-window tick p99, and `LIMIT` when
the rendering budget throttles simulation work.
