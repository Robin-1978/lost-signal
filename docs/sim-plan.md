# SIM Phase Plan (Phase 2: sim re-integration — now with MuJoCo backend track, see below)

Robot side is done (clean rebuild, 24/24 green, boundary gate green).
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
  (freejoint + 2 hinges + ball casters), duty-implied motor force +
  smoothing service brake per main's duty path verbatim, 1 ms substeps,
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
  record taps from sim time, never wall clock; the decision tap is inert —
  the ROBOT branch gates on the coordinator's own 10 Hz).
- Scenario goal driver (objective list -> place goals through the same
  `request_goal` seam an operator uses; arrival/completion counted
  truth-side for scoring, never fed back into ROBOT).
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
  every threshold lives in the owning module.

### S9 Sensor realism parity (in progress) — capabilities match-or-exceed
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
ridge missions green with pre-change separations.
Next slices:
traction memory, actor lifecycle richness.

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
