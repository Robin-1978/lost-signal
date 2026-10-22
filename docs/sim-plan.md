# SIM Phase Plan (Phase 2: sim re-integration)

Robot side is done (clean rebuild, 24/24 green, boundary gate green).
This phase ports the **deployable** sim algorithms from `main` onto the new
chain. Presentation never crosses: game UI / client / renderers / audio /
map editor / campaign visuals stay out. MuJoCo stays out: the analytic
plant (motor curve + contact + rigid kinematics) is the sim, not a physics
engine binding.

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

- `World`: force-driven truth (DC motor curve -> wheel accel, dissipative
  service brakes), diff/ackermann/swerve/mecanum rigid follow, static
  boxes + linear actors + named places.
- Providers: motion/IMU/lidar2d/lidar16/suite/static-map/replay.
- `SimRobot` + `ScenarioSession` (world-stepping HIL), eval
  (episode/scoring/JSONL recording/replay/drive-profile-io).
- `DriveProfile`: robot-side contract struct + sim-side JSON merge.

## Phases (in order; each lands green before the next starts)

### S1 Motor physics (small) — landed
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

### S2 Tire contact (small)
Port `hardware/physics/tire_contact` (Coulomb + smoothing + viscous,
gravity is physics): per-wheel contact force opposes slip, added to the
motor force before integration. Same `DriveProfile` supplies the
coefficients.
Accept: standstill holds on grade without creep; breakaway + rolling
covered by unit tests; sim_nav corridor run unchanged.

### S3 Vehicle dynamics per chassis (medium)
Replace the kinematic rigid-follow with force-driven rigid bodies per
mode (diff / ackermann with servo + linkage / swerve / mecanum), adapted
from main `hardware/vehicle/*` minus the MuJoCo/contact-truth internals.
Steering stays a velocity servo (already is); drive wheels integrate
motor-minus-contact forces; body follows chassis kinematics from true
joint speeds (already does).
Accept: per-chassis HIL loops (existing `test_actuation` plant sections)
pinned to force-model trajectories; encoder quantization added to
feedback (counts/round-trip, matching `encoder_counts_per_revolution`).

### S4 Actor behaviors (medium)
Today's actors fly straight lines. Add a behavior registry + motion
policies on the SIM side only: waypoint follow, crossing (constant-speed
pedestrian/vehicle cuts), wander with seeded RNG (deterministic per
scenario seed — replay bit-identical).
Accept: scripted crossing scenario in `ScenarioSession` produces tracks;
perception + behavior + validator chain reacts; replay of the episode
reproduces the same decision sequence.

### S5 Scenario systems (medium)
Port the orchestration that makes scenarios more than a world step:
- `deterministic_scheduler` (multi-rate sim-time deadlines: scan /
  decision / replay cadences from sim time, never wall clock).
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

### S6 Map persistence (small-medium)
The deferred map-package gap: versioned survey persistence (manifest +
checksummed raster layers + semantics + places) over our JSONL subset,
load/save round-trip byte-identical. Backs survey/offline interchange
and the static-map provider from files.
Accept: save -> load round-trip test; `StaticMapProvider` can source
from a package file.

### S7 Sensor realism (medium)
Close the honesty gaps in feedback: encoder quantization (already in
S3), deterministic delivery queue (latency + dropout injection in
providers, seeded), IMU noise/bias (bounded, documented; attitude
estimator already tolerates it). All seeded and replay-deterministic.
Accept: dropout/latency unit tests; HIL tests pass with realism on
(default off) and off (default).

## Explicitly OUT (not deployable sim)

- `game/client/*` (UI, renderers, audio, map editor), campaign visuals,
  `VisualDefinition`/model assets.
- MuJoCo bindings (`mujoco_simulation`): analytic plant is the model.
- Manipulator (whole stack, both sides).
- `world_activity` weather/incidents, `field_dispatch`/`field_operator`
  game logic, replay shadow/observation-only models (unwired on main).
- `terrain_relief`/mesh helpers, sha256/random/event-log utilities.

## Order rationale

Physics bottom-up (motor -> contact -> vehicle) because every scenario
result depends on the plant; then scenarios (actors -> systems -> judge)
because eval needs behavior worth scoring; then persistence/sensors,
which are orthogonal and can slip in any order after S3.
