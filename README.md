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
./build/lost_signal_sim --map configs/worlds/signal_town.json \
 --objectives energy_hub,freight_yard,housing_door,market_gate,town_gate \
 --steps 8000 --require-pass
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
`--require-pass` returns a nonzero process status when any requested objective
is incomplete or fails the independent SIM acceptance contract, which makes a
headless benchmark suitable for CI without parsing presentation text.
Without `--steps` or `--duration`, an explicit objective list gets a 4000
step budget and a single implicit goal keeps the legacy 5 step smoke run.

## First playable field lab

```sh
./build/sim_viewer
# Equivalent explicit entry:
./build/sim_viewer --play
# Force the Simplified Chinese field lab:
./build/sim_viewer --play --locale zh-CN
```

The authored campaign now runs through seven chapters in order: First
Contact, Outdated Map Live Safety, Mapless Loop, Continuous Terrain, Signal
Town, Signal Town Big, and Industrial Scale (200,000 m²). Four engineering
courses are also catalogued for slope, low-grip, water, hillside, and recovery
work. Use `--experience` for an authored lesson or `--map` for an open run
that uses every named place in that world:

```sh
./build/sim_viewer --experience configs/experiences/slam_loop.json --3d
./build/sim_viewer --map configs/worlds/slope_lab.json --3d --maximized
./build/sim_viewer --map configs/worlds/proving_ground_map.json --3d
./build/sim_viewer --map configs/worlds/industrial_200k.json --3d --rate 8
./build/sim_viewer --map configs/worlds/signal_town_big.json --3d
./build/sim_viewer --chapter proving_ground --3d --maximized
```

The complete Viewer catalog is discoverable without opening a window:

```sh
./build/sim_viewer --catalog
```

The catalog includes the five-stop Signal Town, the 200,000 m² industrial
route, continuous terrain, the proving ground, orchard hillside, stormglass
station, and washout canyon. Every catalog world remains available with
`--map`; when a world has no decorative scene manifest, the Viewer uses
deterministic physical geometry fallback so a missing GLB never removes a
physics test.

These worlds cover mapless SLAM, ramps and wet/low-grip surfaces, braking and
material changes, a 200,000 m² industrial route, and multi-goal traffic in
Signal Town. A successful final debrief says `CAMPAIGN COMPLETE` and offers
`REPLAY FINAL` rather than mislabeling the replay action as a failed retry.

With no authored source arguments, the viewer opens the First Contact field
lab. Use `--sandbox` for the open engineering demo, or `--experience FILE`
for another authored mission. The field lab pauses before simulation time
advances. Its briefing camera
shows the assembled robot from MuJoCo's zero-time body and joint transforms;
this preparation publishes no sensor sample, ROBOT tick, episode step, or
score. Use the on-screen component tabs to inspect the drive wheel and LiDAR
before starting; caster and chassis remain optional detail cards. Each camera
target comes from the prepared physical link or sensor pose. Click
`START EXPERIMENT`, then predict how changing the LiDAR period from 20 ms to
100 ms will affect completion time, physical clearance, and safety.
The debrief separates ROBOT evidence from SIM-only truth evaluation and uses
collision as a hard safety gate. `Left`/`Right` then browse the recorded first
valid observation, braking command, and stable-stop evidence. Their speed and
settle rules come from the selected robot's versioned arrival policy. The
baseline debrief offers `RUN VISIBLE VARIANT`. It rebuilds the authored world,
robot, initial state, and seeds, changes only `lidar.scan_period_s`, and runs
the second condition in the same visible Viewer. The chapter compares both
recorded outcomes and unlocks the next chapter only after both runs pass their
independent safety gates. Its authored comparison tolerances classify the
player's prediction for teaching only; they do not tune ROBOT or SIM
acceptance. The
experience selects ordinary robot, sim,
map, scene, and named-goal files before startup; runtime autonomy receives the
same sensor and actuator interfaces used without the game presentation.
`--locale auto` follows `LC_ALL`, `LC_MESSAGES`, or `LANG`; `--locale en` and
`--locale zh-CN` select a language explicitly. Localized manifests change only
presentation text. They resolve to the same robot, simulator, map, scene,
objective ids, physics, and autonomous software.

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
 axes, semantic pivot, full asset-to-link transform, link binding, grounding
 points, optical center, material roles, LOD inventory, render budgets,
 content fingerprint, license, source artifact, reference dimensions, and
 expected bounds. Missing metadata, changed bytes, or changed bounds fail visibly and use deterministic
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
After an intentional asset rebuild, run
`tools/modeling/refresh_asset_hashes.py` and then the Blender audit. Hash
refresh never changes authored frames, grounding, budgets, bindings, or
provenance.
The viewer advances 20 ms scenario transitions composed of deterministic
1 ms physics quanta at 1x by default.
Use the visible `1x`/`2x`/`4x`/`8x`/`TURBO` controls during a run, `--rate N`
or `+`/`-` for 0.05x–64x playback, and `--turbo` or `T` to run as fast as the
bounded Turbo work budget allows. Use `--quality high|fast` or
`Q` to choose the Viewer pipeline; Turbo selects Fast while active, then
restores the chosen quality. Fast skips shadow/HDR passes but keeps physical
link poses and simulation events. Playback never changes physics, sensor
stamps, or ROBOT control periods. The HUD and exit summary report requested
rate, measured real-time factor, steps/s, sliding-window tick p99/max, active
quality, and `LIMIT` when the rendering budget throttles simulation work.
In an open sandbox (`--sandbox` or `--map`), press `H` to enable manual
driving; `W/S` or Up/Down command forward/reverse and `A/D` or Left/Right
command yaw. Manual input remains a typed ROBOT body command and still passes
the validator, raw-scan Safety monitor, command limits, watchdog, and MuJoCo
actuator path. Press `H` again to clear it. Authored experiences keep manual
input disabled so their evidence remains reproducible.
The window is resizable (minimum 960×640); use the title-bar maximize control,
`F10` to maximize/restore, or `F11` for fullscreen. `--maximized` starts the
Viewer maximized. The 200K industrial map can be inspected in the same 3D
Viewer with:

```sh
./build/sim_viewer --sandbox --3d \
  --map configs/worlds/industrial_200k.json --rate 1 --quality fast
```

The map has no authored presentation scene, so the Viewer uses its physical
collision geometry and procedural fallback assets; `--physics-overlay` or `K`
exposes COM, joints, contacts, and collision proxies for engineering review.
Press `E` for the deployable ROBOT AI overlay: cyan shows anonymous perception
tracks and velocity, while purple ribbons and uncertainty rings show the same
multi-hypothesis prediction consumed by behavior, DWA/TEB, validation, and
safety. Campaign runs keep `K` SIM truth diagnostics locked until debrief;
the overlay never reads actor identities, future waypoints, or truth poses.
Use `--lidar --robot-evidence --shot N FILE` for a reproducible visual audit;
the same flags work in 2D and with `--3d` and never enable SIM truth input.
For unattended parameter sweeps and CI, use `lost_signal_sim` directly so no
graphics work competes with the fixed-step simulation.
Open sandbox runs keep a deque-backed recent replay window (120 simulated
seconds by default) while full-run score and the latest diagnostic continue to
accumulate. Use `--history-seconds N` to choose that local window or `0` for a
complete in-memory history. Authored campaign runs always keep the complete
episode required for debrief and controlled A/B comparison.
The Viewer plays presentation-only cues for ROBOT safety hold/recovery and
independently accepted task completion. Separate motor/gear, rolling, tire
slip, mechanical brake, and impact layers read recorded wheel speed, MuJoCo
contact load/slip, the exact service-brake actuator request, and physical
impact impulse/relative speed. `--mute` disables playback while the evidence
state still advances. Pause, rewind, and Turbo cannot replay compressed
history. Sound has no path into physics, sensors, ROBOT, or scoring.
