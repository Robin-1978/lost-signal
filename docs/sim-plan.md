# SIM Phase Plan (Phase 2: sim re-integration — now with MuJoCo backend track, see below)

Robot side is continuously gated by a clean rebuild, the full CTest suite,
and the boundary check (currently 51/51 green).
This phase ports the **deployable** sim algorithms from `main` onto the new
chain. Presentation never crosses: game UI / client / renderers / audio /
map editor / campaign visuals stay out. The physics backend is MuJoCo
(M-track below); the M2 cutover deleted the analytic S1–S3 integration
loop (effort math stays as the tested spec in contact/motor unit tests).

## MuJoCo backend track (M0–M3, done)

- M0 (done): MuJoCo builds from source (pinned 3.16.0, FetchContent),
  link + drop determinism gate green.
- M1 (done, historical): `MujocoBackend` behind a `World(PhysicsBackend)`
  switch (default analytic, ROBOT untouched): XML-generated diff base
 (programmatic mjSpec: freejoint + drive/steer hinges + passive caster
 swivel/roll hinges), duty-implied motor force + smoothing service brake per
 main's duty path verbatim, 1 ms substeps,
  truth read back for the unchanged providers. Parity gate:
  `test_mujoco_backend` (behaviors + crossing HIL + determinism). The
  switch itself was deleted in M2 — engine-only since.
- M2 (done): all chassis on the engine, traction/slip flags in WorldTruth
  (SIM-side labels only), viscous drag via native joint damping, slope P1
  regression on the engine, analytic integration deleted (tests re-pinned
  to engine semantics as the witness).
- M3: raylib viewer reading truth (SIM-side only). Done, in three cuts:
  M3a 2D live viewer (frame model + sim_viewer binary, --shot smoke),
  M3b map editor (world file + headless model + viewer edit mode),
  M3c 3D orbit view over the same frame (--3d, V toggles).
- M3d (done): presentation hardening against the old game's viewer.
 The new viewer keeps the new `ScenarioSession`/`Frame` seam, but must reach
 parity on the visible contract: chassis-specific component geometry, live
 wheel/steering/track motion, terrain-conforming placement, authored actor
 models where available, deterministic dust/slip/impact particles, and a
 lit 3D pass with stable camera/fog/shadow cues. These are SIM-only draw
 products. They consume recorded `WorldTruth` for presentation and never
 publish material, contact, actor identity, or renderer state to ROBOT.
 `assets/models`, `assets/textures`, and `assets/shaders` are carried in this
 branch. `sim_viewer` auto-loads the matching GLB set and accepts `--assets`
 for an alternate SIM content root; missing files fall back to procedural
 geometry. Acceptance is a headless frame-model regression plus a display
 smoke shot;
 particle lifetime and placement are deterministic from the SIM step, while
 rendering may interpolate between frames without changing the episode.
GLBs are assembled as physical components rather than dropped in as a single
decorative mesh: chassis scale comes from the configured footprint/deck,
drive wheels are mounted at the live MuJoCo wheel links and use their spin and
steer joints, caster fork/wheel assemblies use the support radius and the
body-frame axle offset, and the lidar model is mounted at the same configured
extrinsic that produces the scan origin. Wheel-link assets use an explicit
authored `+Z` axle contract; installation and live roll/steer rotations are
composed once, so a differential wheel cannot acquire a spurious vertical
steering axis. Actor models retain their authored ground anchor and content
reference radius, then follow the canonical SIM terrain height. Terrain mesh
rendering stays within 1 mm of the engine contact surface, preventing visible
tire interpenetration. The renderer does not replace any of these poses with
a wall-clock animation or a ROBOT estimate.

Unpowered supports keep the existing `kind: "ball"` configuration spelling
for compatibility, but the MuJoCo plant now realizes each one as a
low-friction fork swivel plus passive wheel-roll hinge. The viewer reads those
joint angles and the wheel footprint back from the engine, so caster motion is
physical SIM state rather than a time or body-velocity animation. The support
table remains SIM-only and never enters ROBOT observations or actuator
channels.

### Recovery and pose/rendering invariants

Whole-map localization is a recovery operation, not a periodic pose filter.
When a localized estimate has finite covariance below the loss thresholds, the
supervisor only runs the bounded local matcher at its motion cadence. A global
search is due only for an unlocalized start, an invalid estimate, or covariance
above the recovery threshold. A rejected local scan match is diagnostic
evidence, not proof that the pose is globally lost: while local corrections are
absent, prediction uncertainty grows through the estimator and recovery starts
only if that fused covariance crosses the threshold. Confidence, evidence age,
and consecutive local-match failures cannot bypass this covariance gate. The
global retry is rate-limited, and a candidate that would make a trusted
low-covariance trajectory jump beyond its uncertainty-scaled limit is rejected.
Simulator truth is never used in any of these gates.

The incremental whole-map search freezes one scan, so its winning pose belongs
to that scan's acquisition time. Before applying it, localization composes the
ROBOT estimator's relative SE(2) motion accumulated while the search was in
flight. A successful absolute fix resets SLAM keyframes and the rolling map,
because those products belong to the previous frame. With a surveyed map,
local SLAM starts only after that frame is established; mapless startup first
declares the current odometry origin and then starts SLAM. These rules prevent
a delayed result or arbitrary pre-localization SLAM frame from pulling a moving
robot backwards, without consulting simulator truth.

Unknown cells in a sparse prior or a live SLAM raster are eligible search
hypotheses because rejecting them makes a map with unobserved space
unlocalizable. Unknown space contributes neutral endpoint score only. A global
fix still needs measured returns, overlap, score margin, and the final
occupied-cell raycast check; unknown cells cannot manufacture confidence.

The physical/rendering contract follows the same single-source rule. MuJoCo
`qpos`/`qvel`, wheel hinges, steering hinges, and caster swivel/roll joints are
the only pose inputs to the SIM frame. Rendering composes them in this order:
body pose, fixed GLB authored-frame correction, steering/swivel, then wheel
spin/roll. A differential wheel has no steering transform; an Ackermann or
swerve module uses its measured steering joint. GLB scale and axis correction
are fixed asset metadata, never inferred from the current pose. On a model
rebuild, x/y, attitude, velocities, and joint angles are restored, while root z
is revalidated against the new wheel radius and the canonical terrain contact
height so an old snapshot cannot reintroduce an embedded tire. The viewer may
apply only the documented sub-millimetre contact presentation guard; it does
not replace engine pose with a ROBOT estimate or a wall-clock animation.

Master parity boundary: the deployable simulation pieces are carried over
(MuJoCo plant, terrain/material contacts, 2D/3D LiDAR, actor sidecars,
multi-goal headless runs, scoring/artifacts, GLB component rendering, camera
follow and deterministic effects). The old game client's campaign/audio stack,
content catalog, HDR/bloom/shadow-map renderer, and manipulator articulation
remain outside this SIM boundary. They require their own SIM-side adapters;
they must not be copied into ROBOT configuration or used as hidden truth.
- M4 (done): post-viewer hardening, four cuts.
  M4a TF tree (`robot/core/transform`, `platform/static_transform_tree`;
  coordinator is the only odom publisher and the only TF reader/writer).
  M4b true-pose sensing (mj_ray lidar vs compiled scene + actor cylinders,
  mount-aware IMU, mount-matched suite wiring) with true-height prisms
  (height <= 0 is flat paint everywhere: no engine body, ray, map cell,
  or judge contact; real obstacles carry explicit heights; seeds carry
  true mast extrinsics).
  M4c robot/sim config split (ROBOT parses its hardware contract via
  `robot::platform::load_robot_config`, native-deployable; SIM knobs live
  in their own file; extrinsics have one source — the TF fan-out, no
  per-algorithm copies).
  M4d slope-correct 3D sensing (estimate dead-reckons z, odom->base is full
  3D, every deprojection path composes map<-base with the full TF mount;
  graph stays SE(2)).
- M5 (done): 16-line 3D lidar (`Sim3DLidarProvider`, ±15° fan, same static
  + actor split as 2D, sensor-frame points, misses dropped) with a
  suite-side composite front end (`lidar.composite`, opt-in):
  ground-filtered min-per-azimuth feeds the unchanged 2D chain, so low
  plates the horizontal ring flies over become visible. No
  `ObservationFrame`/recording changes — the envelope still carries 2D
  ranges. Proven by a plate pair (sensor: 2D blind vs composite 2.0;
  mission: surveyed wall holds either way) and a 0.1-grade loop mission.

## Standing rules (same as Phase 1)

- SIM truth never enters the ROBOT decision path: truth is recorded for
  offline scoring / replay / RL labels only (`tests/check_boundary.py`
  enforces: no `sim/` includes, no truth tokens, no robot->sim links).
- SIM owns truth + presentation; providers stay thin translators
  (typed messages -> LSRI envelope, same seam native drivers use).
- `World` remains the single truth owner; scenario owns orchestration;
  eval owns scoring. No new singletons, no wall-clock in tick paths.
- Port math verbatim, adapt namespaces/types; single canonical
  representation, same behavior verified by tests.

## Where we start (already on the chain)

- `World`: engine-owned truth (MuJoCo integrates; duties become joint
  efforts through the DC-curve + service-brake math, viscous drag as
  native joint damping), all four chassis on the engine, static boxes +
  linear actors + named places. The S1–S3 analytic force-integration loop
  is deleted (effort math kept as the tested spec in contact/motor unit
  tests).
- Providers: motion/IMU/lidar2d/suite/static-map/replay/package-map.
- `SimRobot` + `ScenarioSession` (world-stepping HIL), eval
  (episode/scoring/JSONL recording/replay/drive-profile-io).
- `DriveProfile`: robot-side contract struct + sim-side JSON merge.

The scenario scheduler keeps independent scan, encoder/motion, IMU, decision,
and record deadlines. A single transport envelope may contain only sections
acquired on that tap; stale scan/IMU/twist sections are invalid and are never
reintegrated by ROBOT. `lost_signal_sim --episode` exports the observation
stream as deterministic JSONL v2, while `--out` writes an offline mission
summary with elapsed time, clearance, and energy proxy metrics.

The current MuJoCo brake uses hinge `frictionloss`: a bounded passive
constraint that can hold zero speed without an explicit torque overshooting
through zero and injecting reverse creep. IMU synthesis computes
`q_prev^-1 * q_now` with shortest-arc axis/angle, differentiates world planar
velocity before rotating acceleration into the body, and obtains vertical
acceleration from the second derivative of height. REP-103 gravity projection
uses `R^T(0,0,g)`. The ROBOT attitude filter downweights non-gravity
acceleration and rate-limits tilt correction while continuing gyro propagation.

## Phases (in order; each lands green before the next starts)

### S1 Motor physics (small) — landed (historical: analytic integration loop deleted in M2; effort math kept as the tested spec)
Port `hardware/actuators/electric_drive` (current-limited 4-quadrant DC
curve, ~30 lines) + `service_brake` (torque + apply/release lag).
Replace the first-order lag in `World::step` with force integration:
duty -> motor force (stall/free from `DriveProfile`) -> wheel accel.
Extend `DriveProfile` with `stall_force_N` + `wheel_mass_kg` (the robot
feedforward already models these — sim and native close against the same
numbers).
Accept: step-response unit test pins the curve (stall force at 0 speed,
zero at free speed, electrical braking authority on oppose); all HIL
tests unchanged behaviorally (re-tune only if the curve moves settling).

### S2 Tire contact (small) — landed (historical: analytic integration loop deleted in M2; effort math kept as the tested spec)
Port `hardware/physics/tire_contact` (Coulomb + smoothing + viscous,
gravity is physics): per-wheel contact force opposes slip, added to the
motor force before integration. Same `DriveProfile` supplies the
coefficients.
Accept: standstill holds on grade without creep; breakaway + rolling
covered by unit tests; sim_nav corridor run unchanged.

### S3 Vehicle dynamics per chassis (medium) — landed (historical: analytic integration loop deleted in M2; effort math kept as the tested spec)
Replace the kinematic rigid-follow with force-driven rigid bodies per
mode (diff / ackermann with servo + linkage / swerve / mecanum), adapted
from main `hardware/vehicle/*` minus the MuJoCo/contact-truth internals.
Steering stays a velocity servo (already is); drive wheels integrate
motor-minus-contact forces; body follows chassis kinematics from true
joint speeds (already does).
Accept: per-chassis HIL loops (existing `test_actuation` plant sections)
pinned to force-model trajectories; encoder quantization added to
feedback (counts/round-trip, matching `encoder_counts_per_revolution`).

### S4 Actor behaviors (medium) — landed
Today's actors fly straight lines. Add a behavior registry + motion
policies on the SIM side only: waypoint follow, crossing (constant-speed
pedestrian/vehicle cuts), wander with seeded RNG (deterministic per
scenario seed — replay bit-identical).
Accept: scripted crossing scenario in `ScenarioSession` produces tracks;
perception + behavior + validator chain reacts; replay of the episode
reproduces the same decision sequence.

### S5 Scenario systems (medium) — landed
Port the orchestration that makes scenarios more than a world step:
- `deterministic_scheduler` (multi-rate sim-time deadlines: scan /
 motion/IMU/decision/record taps from sim time, never wall clock; the
 decision tap gates the ROBOT branch at the configured cadence).
- Scenario goal driver (objective list -> place goals through the same
 `request_goal` seam an operator uses). ROBOT reports Done from its own
 estimate; SIM independently accepts the objective only when the physical
 body is within the authored tolerance. A disagreement fails the scenario
 result and is never fed back into ROBOT.
- `physical_collision_judge` (truth-side contact/clearance assessment for
  mission failure + scoring + replay diagnostics; NOT reachable from
  `RobotApplication`).
- Defer/cut: weather/incidents (`world_activity`) — presentation
  coupling, no deployable algorithm; campaign progress — UI state.
Accept: multi-goal scenario runs headless to completion with a score;
judge disagreements (robot thinks clear, truth says contact) recorded
as eval signals, never as tick inputs.

### S6 Map persistence (small-medium) — landed
The deferred map-package gap: versioned survey persistence (manifest +
checksummed raster layers + semantics + places) over our JSONL subset,
load/save round-trip byte-identical. Backs survey/offline interchange
and the static-map provider from files.
Accept: save -> load round-trip test; `StaticMapProvider` can source
from a package file.

### S7 Sensor realism (medium) — landed
Close the honesty gaps in feedback: encoder quantization (already in
S3), deterministic delivery queue (latency + dropout injection in
providers, seeded), IMU noise/bias (bounded, documented; attitude
estimator already tolerates it). All seeded and replay-deterministic.
Accept: dropout/latency unit tests; HIL tests pass with realism on
(default off) and off (default).

### S8 Terrain & ground materials (medium) — T1a/T1b landed, T1c next
Import complex terrain + ground materials SIM-side to test robot
boundaries. ROBOT sees only measurements (scans/IMU/slip) plus the
surveyed elevation prior; material truth never crosses (map.hpp:
no friction, contact, or live wetness).
- T1a (landed): `sim/world/terrain.hpp` single canonical height function
  (engine + survey share it), `World` holds materials/zones/patches/
  slabs/environment, `lost-signal-world/2` interchange with fail-closed
  load, editor carry-through, `test_map_editor` roundtrip pins.
- T1b (landed): engine per-zone friction/rolling/contact (pair takes the
  max: tire carries min(global, local) x environment grip per wheel,
  ground carries the zone value), oriented ramp/slab contact slabs with
  tops coinciding with the surveyed plane, 2-step slip hysteresis with
  zone-aware thresholds, survey elevation rasterization, grip factor in
  `Diagnostics` (ROBOT's own belief, offline only). `test_terrain` pins
  math/engine/survey/refusals. Roughness + bilinear cross-slope refuse
  the build fail-closed until mesh support (T2).
- T1c (landed): icy-climb + dry-ridge boundary missions (`test_terrain_mission`:
  both complete with zero judge steps; ice tightens the governor to the
  floor on measured slip, ridge pins the climb), terrain editing handles
  (patch/zone add/select/move/delete, `test_map_editor` pins), provider
  honesty for the traction chain: IMU reports true specific force (body
  accel + true-attitude gravity) and odometry derives from measured joints
  through layout kinematics (native-driver parity — truth body rates no
  longer launder slip). Longitudinal evidence (wheel-vs-IMU accel, survey
  grade compensation, 15-tick confirmation) lives in the control-layer
  governor; gripped transients stay clean per `test_autonomy`. Known gap
  found by the missions: the validator reads ramp faces as walls (planar
  rollout vs 3D grade) so approaches grind — no collision, but slow; proper
  terrain-aware validation is T3.
- T2 (landed): heightfield/relief import (T2a: shared relief primitive in
  `robot/core`, canonical height carries relief, mesh patches build as tile
  grids sampling the same function the survey rasterizes), tracked chassis
  (T2b: skid-steer on the (2,0) path with yaw-scrub torque, own config +
  registry entry), engine-contact judge (T2c: belly/high-center clearance
  vs the authored surface, truth-side only, gated on a live plant),
  lidar reflectivity (T2d: return-strength synthesis SIM-side, intensity
  channel through LSRI as measurement metadata — the ROBOT never maps it
  back to materials).
- T3 (landed): ROBOT beliefs over the surveyed elevation prior + motion
  evidence only (materials stay unpublished). `mapping/terrain_math.hpp`
  pure grade/rollover/bump math (+ `survey_pitch_along` so the coordinator
  holds no policy); `mapping/traversal.hpp` roughness belief from the IMU
  vertical-excitation observer with per-cell confidence + expiry, segment
  cost/feasibility on grade; A* prices believed roughness (null model =
  today's behavior); validator is terrain-aware (surveyed-ground returns
  are not obstacles, rollout gated on surveyed grade, new
  `kTerrainInfeasible` verdict); mapper skips painting surveyed ground;
  composite ground-cut is surface-relative (SIM-side synthesis). Missions
  prove it: dry ridge drives (no grind, grip stays 1.00), icy stop-go
  climb still ratchets the governor to the floor on measured slip.
  Coordinator doctrine kept: pure wiring (evidence in, envelope out),
  every threshold lives in the owning module. Localization orchestration
  extracted to LocalizationSystem (supervisor sequencing, seed-vs-blend,
  startup modes, deferred tracking, loop-closure replay — coordinator
  wires estimator/mapper/obs in, consumes action + replan out); grip fire
  severity owned by the governor (slip_severity, coordinator wires
  position/time only).

### S9 Sensor realism parity (landed) — capabilities match-or-exceed
master, framework/data/architecture stay new
Rolling-shutter lidar + ROBOT deskew (landed): SIM casts every beam from
its acquisition pose (truth twist x offset, yaw advanced, actors
propagated; height/roll/pitch frozen — master's stated limits), stamps
per-beam offsets (scan-end frame). ROBOT re-expresses endpoints with the
MEASURED twist through one shared pure helper (`robot/core/deskew.hpp`,
exact inverse of generation, proven by a same-contract round-trip test);
perception/mapper/validator all consume it (mapper paints the true cell
while yawing, pinned — no smear). Frozen scans (empty offsets) read every
beam at the stamp pose: legacy behavior bit-identical. Lidar device
noise + log-odds mapping (landed): SIM applies a seeded device model per
hit (bias/white-noise/quantization/dropout/near-field blanking; driven
no-returns publish miss/max-range, never a clamped boundary value;
intensity stays at the ideal surface; exact default = legacy
bit-identical) on 2D/3D/composite alike. ROBOT fuses evidence through a
pure log-odds layer (`robot/autonomy/mapping/log_odds.hpp`, master's math
in the new framework: +1.6/-0.4, clamp [-2.5, 3.5], 0.65/0.35 hysteresis
— first-sight walls still paint promptly, stray clears need 6 sustained
votes, pinned exactly). Mapper keeps all doctrines (foreground clears but
never paints, surveyed ground never paints, one vote per cell per scan).
IMU device model (landed): per-axis bias drawn per run (seeded) + random
walk per poll (sigma*sqrt(dt), master's form) + exact per-axis trim knob
for bias-robustness probes; full 3D lever arm (attitude-delta roll/pitch
rates + first-class truth yaw + vertical body accel, planar motion reduces
to the legacy form exactly); three independent RNG streams (gyro/accel/
bias draw order never cross-couples); ROBOT untouched (estimator already
consumes 6-axis). Live elevation fusion (landed): surveyed-ground returns
fuse reconstructed endpoint heights by inverse variance with forgetting
(master's math, new framework) into the mapper's dormant elevation layer
— actor/wall returns never enter; validator prefers live where confident
and falls back to the survey prior (live confirms ground only, never
invents obstacles; coordinator wires the overlay, holds no policy).
DWA terrain gate + validator rollover envelope (landed): DWA samples
through an optional TerrainGate (master's StaticFeasibility doctrine,
empty = legacy) wired by the coordinator to the traversal belief;
validator adds the dynamic rollover envelope (surface tilt + v*w
cornering load over the support envelope, master's stability-risk form)
beside the static grade caps. Fixed a real stall found by the suite: the
traversal feasibility grade now reads through the survey-resolution
window (same quantity the validator gates on) so nearest-cell stairsteps
never alias into phantom cliffs on short rollout steps — icy climb and
ridge missions green with pre-change separations. Traction memory
(landed, beyond main): the grip governor exposes its own fire verdict
(slip_firing, thresholds stay in the owning module); the coordinator
latches firing cells into the traversal belief with severity from the
tightened factor (position+time wired only, no policy). Slipped cells
price A* routes + segment cost until expiry (worst latches, ice stays
traversable — pricing only, never a veto). Pinned: memory reroutes
through the clean gap at default weights, stale expires, clamps hold.
Actor lifecycle richness (landed, last item): waypoint dwell (timed stop
per arrival, velocity zero + position frozen, composes with loop/stop, 0
= legacy exactly; per-point waits[] overrides dwell_s per index, short
lists fall back, negative reads 0 — master's dwell doctrine on the
incremental stepper, no pure-time-function port); objective step budget (a live goal older
than N world steps is recorded as abandoned with the truth miss kept,
canceled through the seam so a late Done cannot leak into the next
objective, next dispatched — 0 = legacy, mission budget stays the outer
backstop). Pinned: dwell-then-proceed + loop pauses eat distance;
far-abandon advances onto dock which completes cleanly.
Lidar generation dedup (landed): the three poll paths (2D/3D/composite)
share one shutter-pose + actor-hit helper (raycast_util.hpp) — same
strict-less-than winner rule, bit-identical output, all legacy pins
green untouched. Incidence angular response (landed, closed the audit
table): head-on full, grazing floored at 0.15 (master default), normals
analytic per hit class (actor cylinder radial/top, box faces, canonical
terrain gradient — synthesis and contact share the source). Pinned:
grazing floor ratio exact, backface reads 0, box-top full.
Post-parity hardening (landed): global-search perf audit (50 m map
measured 1.1 s — refine-bound, NOT coarse; beam-count/margin coupling
proven by experiment so the default stays legacy, refined_poses
observability added, premature pre-refine dedupe designed out with the
reason written down); mapper recenter shifts belief state rigidly
(log-odds + elevation CellState round-trip, snapshot-first, published
rasters refresh before the early return — loop-closure replay keeps
ownership of stale-frame evidence, shift owns motion); deferred startup
disambiguation (ambiguous global seeds hypotheses, bounded local match
per scan, merge/drop/promote/timeout rules, supervisor retry backstops —
uniform-wall coordinator run refuses to guess for 25 ticks).
Dynamic prediction and local-planner parity (landed): perception can confirm
anonymous moving clusters from scan history when no static reference exists;
every track preserves its acquisition timestamp and bounded uncertainty.
Behavior, DWA, the new timed-elastic-band planner, and the independent
validator consume one `PredictionBatch`. Relative motion is checked
continuously inside each interval, including fast crossings between forecast
frames. Behavior evaluates a timed suffix of the active route and derives its
hold/speed cap from braking distance; it refuses to hold when a forecast target
would collide with a stationary robot, leaving escape selection to the local
planner and validator. Robot JSON selects `dwa` or `teb`; DWA remains the
default because a 100-call deterministic comparison measured about 0.11 s for
DWA and 0.87 s for TEB on the audit machine (TEB about 8 times slower).
Both planners enforce the declared differential/holonomic/Ackermann
envelope and receive terrain feasibility through the same planning interface.
No predictor input comes from actor identity, MuJoCo state, contact truth, or
scenario future waypoints.

The same `signal_town` `energy_hub` mission with both authored actors provides
an end-to-end comparison. DWA completed in 1160 steps / 23.2 s with 0
collisions and 0.110 m truth miss. TEB completed in 2478 steps / 49.56 s with
0 collisions and 0.241 m truth miss. TEB is intentionally optional: its
conservative rolling-map band produces more stop cycles, while DWA remains the
faster default. From standstill, TEB sizes the initial band from the next
acceleration-reachable command instead of requiring a top-speed multi-metre
corridor before it can move.
Next slices:
ROI-first search (local window, whole-map fallback) + matcher hot-loop
restructure — only if real-time deployment demands it; sim-time needs
neither.

## Explicitly OUT (not deployable sim)

- `game/client/*` on main (game UI, audio, campaign visuals, mission
  content stack), `VisualDefinition`/model assets. Our own SIM-side
  viewer + map editor (`src/sim/viewer/`, M3) are IN: truth-reading debug
  tooling with no game assets and no ROBOT input — not a port of the game
  client.
- MuJoCo bindings (`mujoco_simulation`): the engine is the model since M2
  (this bullet predates the cutover).
- Manipulator (whole stack, both sides).
- `world_activity` weather/incidents, `field_dispatch`/`field_operator`
  game logic, replay shadow/observation-only models (unwired on main).
- Mesh import (heightfield/relief waveform): deferred to S8-T2; prismatic
  terrain + material zones are in, roughness/bilinear refuse fail-closed.
  sha256/random/event-log utilities.

## Order rationale

Physics bottom-up (motor -> contact -> vehicle) because every scenario
result depends on the plant; then scenarios (actors -> systems -> judge)
because eval needs behavior worth scoring; then persistence/sensors,
which are orthogonal and can slip in any order after S3.
