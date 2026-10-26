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
`lost_signal_sim_*` library. `ctest --test-dir build` runs the complete
algorithm, physics, content, boundary, and mission suite.

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
tick, actuation, recording, and the complete step, followed by total wall
time, simulated time, measured real-time factor, and steps/s. With `--out`,
the aggregate throughput is also written under `performance`. This timing
remains in the SIM runner and is never published into the ROBOT decision
path. Use a Release build for throughput measurements; Debug is intended for
diagnostics.
Without `--steps` or `--duration`, an explicit objective list gets a 4000
step budget and a single implicit goal keeps the legacy 5 step smoke run.

## First playable field lab

```sh
./build/sim_viewer
# Equivalent explicit entry:
./build/sim_viewer --play
```

With no authored source arguments, the viewer opens the First Contact field
lab. Use `--sandbox` for the open engineering demo, or `--experience FILE`
for another authored mission. The field lab pauses before simulation time
advances. Its briefing camera
shows the assembled robot from MuJoCo's zero-time body and joint transforms;
this preparation publishes no sensor sample, ROBOT tick, episode step, or
score. Use `Left`/`Right` or the on-screen buttons to inspect drive-wheel,
caster, LiDAR, and chassis cards; each camera target comes from the prepared
physical link or sensor pose. Click `START EXPERIMENT`, then click a prediction
card or press its number to begin the run.
The debrief separates ROBOT evidence from SIM-only truth evaluation and uses
collision as a hard safety gate. `Left`/`Right` then browse the recorded first
valid observation, braking command, and stable-stop evidence. Their speed and
settle rules come from the selected robot's versioned arrival policy. The
experience selects ordinary robot, sim,
map, scene, and named-goal files before startup; runtime autonomy receives the
same sensor and actuator interfaces used without the game presentation.

Run the same field lab as a controlled scan-cadence A/B experiment:

```sh
./build/sim_ab_experiment \
 --experience configs/experiences/first_contact.json \
 --scan-period 0.10 --steps 1200 --out /tmp/first_contact_ab.json
```

The runner reloads both variants from the same experience and verifies equal
delivery, IMU, and actor seed digests before writing task, collision,
clearance, localization, path, energy, first-scan, first-brake, stable-stop,
effective-config-digest, and command-digest results. The only override is
`lidar.scan_period_s`; each ROBOT run receives its own ordinary timestamped
observation stream.

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
- Scripted actors may explicitly declare their own reaction time,
 acceleration, braking deceleration, clearance, and `yield_to_robot` policy.
 Their SIM-only controller solves continuous relative-motion envelope entry
 and brakes with the declared capability, preventing a cooperative pedestrian
 from tunnelling through a stopped robot. Unaware actors remain available for
 avoidance tests. Actor motion never enters ROBOT except through sensors.
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
 A fixed-capacity presentation pool advances particles by simulation step,
 suppresses duplicate emission while paused, and resets on replay rewind.
  These values never enter ROBOT observations.
In 3D map mode the viewer starts with a close robot-follow camera and a clean
world view. Press `F` to toggle follow; right-drag or middle-drag releases
follow for free orbit/pan, and `R` restores the default orbit. `L` shows a
sparse LiDAR diagnostic projected with the ROBOT estimate, `E` shows the
estimate, and `K` reveals the labeled SIM truth debug layer. Truth-assisted
scan projection, actor script routes, and physical slip state are hidden by
default because they are not information available to ROBOT.
When `<map>.scene.json` exists, the viewer loads it automatically (or accepts
`--scene FILE`). Solid environment GLBs must bind to a stable `box_id` in the
world file, while floor paint must declare `surface_only`; invalid, missing,
or multiply bound collision proxies fail closed. This scene data remains on
the SIM presentation side and cannot enter ROBOT observations.
The research-base kit now includes collision-bound 11 m storage racks and a
glass safety gate plus a surface-only charging pad. Regenerate those assets
with Blender 5.2 using `tools/modeling/build_research_base_kit.py`, then run
`tools/modeling/inspect_assets.py` to verify the complete inventory and exact
bounds before committing generated GLBs.
The viewer advances 20 ms scenario transitions composed of deterministic
1 ms physics quanta at 1x by default.
Use `--rate N` or `+`/`-` for 0.05x–64x playback, and `--turbo` or `T` to run
as fast as the bounded Turbo work budget allows. Use `--quality high|fast` or
`Q` to choose the Viewer pipeline; Turbo selects Fast while active, then
restores the chosen quality. Fast skips shadow/HDR passes but keeps physical
link poses and simulation events. Playback never changes physics, sensor
stamps, or ROBOT control periods. The HUD and exit summary report requested
rate, measured real-time factor, steps/s, sliding-window tick p99, active
quality, and `LIMIT` when the rendering budget throttles simulation work.
For unattended parameter sweeps and CI, use `lost_signal_sim` directly so no
graphics work competes with the fixed-step simulation.
