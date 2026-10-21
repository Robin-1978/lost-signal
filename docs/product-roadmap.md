# Product roadmap

This roadmap turns the current simulation into a polished, open-source autonomy learning experience. Work proceeds in the order below. A later stage starts only after the preceding stage has a reviewable result and passes its acceptance gate.

## Product outcome

The first public release should let a new player give RO-1 a high-level intention, watch it handle a dynamic encounter autonomously, understand why it waited or moved, change one relevant variable, and reproduce the result. The experience must remain useful at three depths:

- **Explore:** the field scene communicates the immediate situation through motion, landmarks, light, and sound.
- **Learn:** AI View connects observations, predictions, constraints, and actions.
- **Verify:** headless output and a run artifact make the same encounter reproducible and comparable.

The campaign remains deterministic and fully usable without a language model. Known-map LiDAR localization is part of
the release foundation because normal autonomous runs must not depend on simulator-truth pose corrections. Full map-
building SLAM, general scenario overlays, VLA, multi-process transport, HIL, and native robot drivers remain outside
this release path until a concrete learning or deployment case requires them.

## Content formats beyond point-to-point delivery

The runtime should support different kinds of play rather than reskinning every lesson as a trip from A to B. Each format must give autonomy a different question to solve while the player continues to choose intent, equipment, priorities, or constraints instead of steering the robot through the answer.

| Format | Example experience | Player decisions | Autonomy and physics lesson |
|---|---|---|---|
| Directed recovery under a changing world | **Stormglass Protocol:** choose the recovery order of three relay cores during an ion storm | next core, hardware, risk hypothesis | human goal selection versus autonomous motion, payload/CoG, grip, slope, occlusion, prediction and time pressure |
| Collection and route ordering | **Orchard Scramble:** recover fallen apples across a muddy hillside before weather closes in | harvest zone, chassis, tires, collection priority, safety/time preference | multi-goal ordering, terrain cost, feasible motion, traction, payload growth, rollover and fruit-integrity tradeoffs |
| Search and discovery | **Firefly Survey:** locate intermittent signal nodes in fog | search boundary, sensor package, coverage preference, return reserve | coverage planning, observation range, uncertainty, map discovery, energy-aware return |
| Herding and flow | **Runaway Carts:** guide autonomous carts through gates without entering their predicted paths | gate order, waiting zones, traffic priority | multi-agent prediction, temporal reservations, safe gaps, deadlock recovery |
| Construction and verification | **Bridge the Washout:** place a limited set of ramps or supports, then dispatch RO-1 | where to spend construction pieces, chassis and load | traversability, slope and clearance, route feasibility, experiment/retry |
| Recovery under constraints | **Floodplain Rescue:** collect scattered power cells and serve stranded equipment | task priority, energy reserve, route risk | scheduling, recharging, wet traction, payload and time windows |
| Comparative challenge | **Robot Field Day:** run one seeded course with different chassis or planners | comparison pair and metric target | feasible-motion contracts, algorithm comparison, regression evidence |

The first systems prototype after the release foundation was **Orchard Scramble** because collection, changing load,
terrain choice, and fruit damage can create a physical and strategic loop that is unlike point-to-point delivery.
The design is not limited to features the current runtime already exposes. If truthful apples, payload, terrain, or
autonomous task ordering require a new contract, the runtime changes with the slice; unreleased interfaces receive
no compatibility shim.

Its core loop is:

1. Inspect a seeded orchard with apples distributed across paved rows, mud, side slopes, and a narrow ridge.
2. Choose a chassis/build and a collection priority such as safest fruit, fastest basket, or lowest energy.
3. Dispatch the autonomous robot; it selects the visit order and feasible local motion.
4. Watch the carried mass and center of gravity change after each pickup.
5. Debrief fruit integrity, apples collected, route time, energy, slip, and rollover margin.
6. Retry the same seed with one changed variable or compare another chassis through the run-artifact tooling.

Apples are physical mission items with deterministic pickup radii and payload effects. Their score responds to measured impact, roll, and acceleration; decorative animation does not alter the result. The mode earns inclusion only if at least two builds produce visibly different, explainable strategies on the same seed.

The implementation boundary is data first. Level data owns the task set, payload values, return reserve, and
deterministic environment keyframes. Map data owns 2.5D terrain, surface materials, obstacles, and actors. Scene data
owns prefabs, goal-state visuals, attached payload layout, effects, lighting, and instances. Robot data owns the
physical assembly, motion roles, and cargo attachment frame. Task-policy and audio catalogs own the offered
algorithms and event-to-cue mappings. C++ supplies reusable loaders, registries, simulation contracts, and renderers;
it does not name apples, orchard crates, or harvest strategies.

## Current status and canonical execution plan

Lost Signal is an open-source **alpha platform** with a strong simulation and autonomy foundation. It is not yet the
polished first public product described above. The platform can already support deterministic learning experiments;
the remaining work is to close one autonomy safety gap, rerun the release evidence, and turn one implemented scenario
into a clear, attractive, explainable experience.

The completed foundation includes:

- MuJoCo 3.16.0 as the single production physics engine, with articulated bodies, height fields, physical contacts,
 and a 1 ms contact substep;
- the deployable ROBOT / LSRI / SIM boundary, including mixed provider ownership and a ROBOT-only build;
- the unloaded RO-1 `typical` reference envelope, the 16-line LiDAR, RO-2 articulated Ackermann and service brake,
 and independent RO-3, RO-4 and RO-5 reference gates;
- the LSRM 2.5D map contract and editor output for occupancy, elevation, surface classes and semantic places;
- the Vehicle Proving Ground, Orchard Scramble, Bridge regression fixture, Manipulation Lab and Stormglass Protocol;
- deterministic headless execution, replay/run artifacts, content validation, localization, mapping, prediction,
 global planning, local rollout planning, independent trajectory validation and actuator control.

The [reference platform](reference-platform.md) and [simulation model audit](simulation-model-audit.md) define the
limits of these claims. Typical parameters are engineering assumptions until a named physical target supplies
calibration data and held-out validation. The architecture supports transfer experiments; it does not prove
sim-to-real accuracy.

Existing campaign chapters remain available. The Vehicle Proving Ground is an engineering instrument, Orchard is a
systems prototype, Bridge is retained as a construction/map/physics regression, and Manipulation Lab demonstrates a
platform capability. **Stormglass Protocol is the sole candidate for the first polished gameplay experience.**

### Priority clarified from the current engineering review

The recent design discussions add useful constraints, but they do not create parallel release tracks. The next work
must follow this order:

1. **Close P1 safety and interaction defects.** Finish the slope-lab odometry/traction failure without reintroducing
   simulator truth, then close the Orchard, crossing, stop-what-if and dynamic-interaction cases. Every action —
   continue, slow, wait, brake, retreat and detour — must enter the same ROBOT candidate and independent trajectory
   validation path. Waiting is rejected when a predicted participant can occupy the robot's current footprint, and
   `NO_SAFE_ACTION` remains an explicit result when no candidate is proven safe.
2. **Finish the remaining SIM→ROBOT composition migration.** Move the autonomy coordinator that currently assembles
   decisions in `ScenarioSession` behind a provider-neutral ROBOT interface. SIM may provide task intent, maps and LSRI
   endpoints, but must not pass level, actor, scenario or simulator-truth types into that coordinator. Complete the
   platform-profile split and the magic-number audit together; each moved field needs units, source, range and a focused
   test.
3. **Finish P2 localization and mapping integration.** Run all ordinary known-map levels through ROBOT localization,
   keep odometry-only only for explicitly teaching drift, and complete dynamic/ground return filtering, keyframe
   reconstruction after pose-graph correction, mapless online SLAM replay, cold-start global recovery and the 200,000
   m² long-run benchmark. Planner code must remain provider-neutral; localization quality changes its constraints,
   never its algorithm identity.
4. **Run P3 release evidence.** Only after the first three gates are green: GCC 13, Clang 18, ASan/UBSan, deterministic
   artifact replay, full CTest, provider-conformance traces, UI shutdown/input smoke and packaging from the one `build/`
   tree.
5. **Then improve the product loop.** Use the proving ground as the all-hardware manual/autonomy test instrument,
   then validate Stormglass as the first game encounter. UI polish, score presentation, a new teaching mission and the
   visual pass follow evidence from that vertical slice.

TEB is not a current P1 task. The planner boundary is being kept replaceable so a TEB-style local planner can be added
as a controlled comparison later, but it must first pass the same chassis-capability, actuator-reachability, dynamic
occupancy and independent safety-validator contracts as DWA. Adding it now would add a second unvalidated motion policy
while the existing policy still has open failures.

Work proceeds through the following queue. A later package starts only when the preceding exit gate is met. Every
production algorithm and physical model must first pass its smallest-layer contract gate; a successful mission cannot
stand in for a kinematics, dynamics, sensing, estimation, planning, behavior, control, or interface test.

### P0 — Close the current autonomy safety gap

`town_freight --recommended` previously waited until timeout at a safe pose after traffic left tracker range. That
failure is now closed without extending stale tracks. A vanished track remains unknown; absence of a track is never
treated as proof that the crossing is clear.

The ROBOT-side observability contract records how much of a possible traffic approach current LiDAR free-space rays
cover. A physical return, including a mapped wall, ends visibility; short range, partial coverage, occlusion,
future-dated scans and aged scans remain unknown. Coverage is diagnostic evidence for tracking and risk assessment.
It cannot by itself prove a crossing clear because an ordinary map cannot supply an unknown participant's physical
speed or acceleration. Simulation actor truth, actor paths, actor IDs and actor motion parameters stay in SIM.

Focused tests cover a physical-return occlusion, insufficient sensor range, complete visible-corridor coverage,
scan aging, duplicate scan timestamps, empty/stale prediction fallback, and the content-to-ROBOT semantic adapter.
The earlier GCC 13 fixed-seed completion used a map-authored participant speed as a hard unseen-arrival bound. That
evidence is withdrawn: it demonstrated the scenario behavior but did not establish a deployable safety contract.

**Exit:** mapped and unmapped dynamic conflicts use the same perception-owned tracks, multi-model time occupancy,
braking-feasible trajectory validation and independent collision monitor. No release depends on authored actor
identity, path, turnaround point or physical capability. The reference scenario completes without weaker margins or
scenario-specific decision code.

**Status: reopened.** P1 supplies the general dynamic-interaction implementation required to close it.

Current corrected-contract evidence: `lost_signal --recommended --seed 47`
completes in 88.78 s with zero collisions and 0.392 m minimum measured dynamic
clearance after the GAME score target was removed from the ROBOT hard gate.
The run correctly completes without earning the optional 0.6 m clearance
badge. Its continuous result is 77.6/100: clearance quality 65.4, pace 100,
estimated efficiency 100, smoothness 48.1 and decisiveness 71.6. The 18.9 s
wait, 2.3 s minimum-risk recovery and 23 action changes remain explicit
optimization evidence rather than being hidden by binary completion.

`town_freight --recommended` also completes in 60.04 s with zero collisions.
Its former permanent `HorizonShort` hold was a route/geometry defect: the
crossing evaluator skipped the first route vertex whose full footprint had
already cleared the conflict envelope, lengthening a 3.5 s provable maneuver
to 4.75 s. The corrected first-clear-exit contract has a focused diagonal
regression. Its current 91.0 score includes seven strategy changes, no
minimum-risk recovery and 62.8 smoothness; neither the prediction horizon
nor the physical margin was weakened.

The `rain_descent` unload marker now lies on a continuous flat runout after
the wet 0.28-grade ramp. Previously the marker labelled “坡底卸货区” was still
on the incline, so a robot required to arrive below 0.18 m/s could reach the
correct position yet roll and tip while trying to park without a declared
parking brake. The ramp retains its grade and wet-contact experiment; a
content/terrain assertion pins flat goal geometry and continuous ramp-to-deck
height before the mission regression checks the recommended and low-grip
outcomes.

Global navigation now assigns a material soft cost across two free cells
beyond lethal inflation. The old `0.001` one-cell critic could only break an
exact path-length tie; `scrub_climb` therefore selected a shorter route along
the inflated shelf boundary and the local planner alternated between tracking
and clearance recovery. The new focused oracle proves that a modestly longer
wide corridor outranks a shorter boundary-hugging path while lethal occupancy
and physical collision geometry remain unchanged. The recommended tracked
run completes with zero collision; its 111 s runtime and low smoothness remain
visible score targets rather than being called complete optimization.

### P1 — Close foundation contracts before another release run

Finish the active DWA, trajectory-validation, dynamic-prediction, crossing-behavior and LiDAR-perception work at the
module where each claim originates. Normal drive acceleration, normal braking and the stability-aware emergency-stop
tail remain distinct; forward/reverse candidate reachability stays explicit; and zero safe braking authority is a hard
rejection rather than an implicit feature-disable value. Generic dynamic traffic uses a fresh, route-relevant finite
prediction window and compares ego and target at the same time. A longer-range or denser LiDAR must not freeze a robot
because it observes a real target that will occupy the same place at a different time. A mission goal may not lie in a
fatal actor's authored swept volume.

Maintain a [production coverage ledger](verification-coverage.md) for every algorithm and model. Each row names its contract, units/frames/time
semantics, independent oracle, invariant/property cases, boundaries and faults, deterministic replay/system evidence,
and remaining claim limit. Additions enter integration only after their focused gate exists. Tests must cover sensor
density and cadence changes, stale/out-of-order data, degenerate and no-solution inputs, actuator saturation, low
friction and grades where applicable. Tests may use simulator truth as an oracle, but production ROBOT inputs may not.

**Exit:** focused and component tests prove the corrected contracts; the standard 800-sample planar LiDAR and teaching
profiles preserve one stable moving-object track within declared tolerances; all dynamic-traffic reference missions
finish with zero collisions without longer time limits or weaker margins; and the coverage ledger has no unowned
production subsystem.

The latest GCC build passes the focused SLAM/localization set and all 16-line projection gates. A serial full-CTest
probe reached `mission_simulation` and exposed an existing P1 failure in the recommended `slope_launch` build: after
the mid-slope inspection stop, RO-1 rolls back under the physical plant and remains at zero command until timeout,
with no collision. This is tracked as a real differential controlled-stop/grade-hold defect; it must be fixed in the
ROBOT control/actuator contract or the level's physically declared capabilities, never by relaxing the mission timeout
or treating simulator pose as localization.

The active crossing implementation now treats the canonical path, a map-validated shortcut and a fully rechecked
circular-fillet path as separate candidates. Static feasibility cannot erase a dynamically safer route. Fillet
curvature is constructed in a shared ROBOT module, continuous curve samples carry explicit execution semantics, and
insufficient leg room falls back to stop-turn-go. The no-return boundary is the conflict approach safety envelope,
not physical overlap with the semantic box. Bounded predictor hypotheses are also used for short-occlusion track
association. P1 remains open until the full reference-mission and release gates are green.

The next P1 correction is a general dynamic-object contract, not a `lost_signal` forklift rule. Map conflict semantics
are optional context: without them, ROBOT derives temporary conflict regions from intersections between its planned
motion and tracked time occupancy. Track identity and
motion intent are separate: one physical object keeps its stable track ID while it stops, reverses, turns or changes
model, unless association is genuinely lost and the lifecycle retires that track. The tracker publishes timestamped
kinematics, shape, covariance/quality and lifecycle state. Global association uses uncertainty and geometry gates;
it must not use simulation actor IDs. Prediction maintains explicit stationary, constant-velocity,
constant-acceleration/turn and map-constrained hypotheses where evidence supports them. A learned shuttle or rail
cycle may be one hypothesis, but two observed endpoints never make periodic reflection a safety guarantee for an
ordinary vehicle.

Behavior uses hypothesis confidence and uncertainty to rank progress, while command authorization checks every
retained motion hypothesis against the complete executable ego trajectory and its braking tail. This is a
receding-horizon, risk-controlled decision, not an absolute proof about an arbitrarily fast unknown participant.
Hard guarantees require an independently enforced signal or physical interlock. Unsafe evidence changes the
pre-entry decision to STOP immediately; GO requires sustained, timestamp-distinct evidence. Unknown is not silently
renamed as reversing, safe or permanently blocked. The acceptance matrix includes constant motion,
threshold-adjacent noise, deceleration-to-stop, deceleration-to-reversal, early reversal, turning, cadence changes,
drop/out-of-order, partial occlusion, identity recovery, two-target crossing and an actually new target. Each case
checks ID continuity, state/covariance bounds, hypothesis continuity, first conflict time, minimum same-time gap and
safe-stop behavior before any mission outcome is considered.

Implemented in the current P1 slice: the map-independent entry-window assessor is ROBOT-owned, clips the executable
route to prediction coverage, detects continuous between-frame crossings and derives a temporary braking hold without
an authored crossing or stop point. Stationary temporary targets remain in prediction even before any non-zero
velocity is observed. Estimator readiness is now history-based and no longer means that a complete periodic shuttle
cycle has been learned. Mapped rules and online inferred constraints are now intersected before a no-return boundary;
the first valid wall along the current route wins, and the same swept stop-plane validator consumes map-independent
walls. After no-return, a new upstream wall cannot replace the independently validated exit maneuver.
The assessor also refuses to label the current pose as a safe waiting wall when retained target occupancy sweeps it;
the conflict remains active while the local planner compares validated motion against brake-and-hold.

The remaining gate is a typed stationary/CV/acceleration/turn model bank with uncertainty, a general interaction
candidate owner for proceed/slow/brake-and-hold/avoid actions, and successful reference missions under the corrected
contract. Brake-and-hold must itself pass time-occupancy validation. Avoidance belongs to normal behavior/local
planning and must satisfy actuator, static-space, terrain, stability and braking-tail constraints. The last-line
collision monitor may limit or stop motion but must not synthesize steering. A lower-risk escape is reported
separately when no collision-free action exists; it must never be labelled safe merely because stopping is worse.
Regression reports must include total waiting time, decision-mode transitions,
time from the first continuously valid departure window to commitment,
`T_intrusion`, `T_stop` and `T_escape`, plus the rejection reason for proceed,
hold and escape candidates. Add explicit `NO_COLLISION_FREE_ACTION` and
`VALIDATION_INDETERMINATE` outcomes instead of representing every unresolved
cycle as an ordinary wait.

Campaign results must also separate hard completion from continuous
optimization. Objectives, collision/rollover and irreversible mission
constraints decide success; raw minimum clearance, time, energy, smoothness
and localization error remain comparable metrics, with threshold badges as
secondary achievements. A score target may rank candidates or award a badge
but must never enter ROBOT trajectory authorization.

The GAME-side continuous score is now implemented for successful runs only.
It preserves the original values and reports safety margin, pace, estimated
efficiency, command smoothness and decisiveness plus a transparent weighted
overall score. Ideal clearance saturates at its authored target; a closer
collision-free run remains successful and receives a lower clearance score.
Failures are unranked with zero score, so speed or energy can never compensate
for a collision, rollover or incomplete objective. Campaign progress schema 3
stores the best overall result and its five axes; the score is presentation
and optimization data and has no dependency path back into ROBOT.

The first typed-contract slice is implemented: `PredictionBatch` preserves
track identity, hypothesis identity, model type and the shared prediction
clock, and `AutonomyPipeline` owns its generation. Existing consumers receive
an explicitly named deterministic flattening adapter. The remaining model-bank
work is acceleration/deceleration, turning, covariance/quality, lifecycle and
calibrated hypothesis ranking; the flattened adapter cannot be used as an
identity source.

### P2 — Add deployable known-map LiDAR localization

Replace the normal `fused` mode's simulator-truth absolute pose with a ROBOT-side known-map localization provider.
Keep the standard frame split used by REP-105: wheel/IMU odometry owns a smooth but drifting `odom -> base_footprint`
prediction, while accepted map observations update `map -> odom`. The provider consumes only timestamped LiDAR,
wheel/IMU estimates, calibrated sensor extrinsics and the immutable LSRM map snapshot; it publishes pose, covariance,
quality and explicit rejection reason through the same runtime observation boundary used by replay and future hardware.

#### Localization, LSRI and planning must remain separate

Localization is a replaceable ROBOT capability, not a planner variant. `KnownMapLocalizer`, AMCL/particle
relocalization, LiDAR-inertial odometry, and a future full SLAM backend must all publish the same timestamped pose
observation or `map -> odom` correction. The global planner, local planner, trajectory validator and controller must
consume the resulting `RobotState` and declared chassis capabilities without knowing which localization provider
produced it. Adding or replacing a localization provider therefore must not require changes to DWA, TEB, Hybrid A*,
or chassis-specific rollout code.

LSRI remains the transport and lifecycle contract between providers and ROBOT. It carries LiDAR, IMU, encoder,
localization and actuator observations/commands with units, frames, timestamps, sequence numbers, source identity and
expiry; it does not perform SLAM, planning or control. SIM may implement an LSRI provider, but it must not inject
world truth into the normal ROBOT localization path. A real LiDAR can be mixed with simulated IMU or actuators only
through the same provider boundary.

The only planner-facing consequence of localization is state validity. A rejected or stale match leaves odometry in
control and reports degraded confidence. An accepted correction within the estimator gate updates `map -> odom`.
A discontinuous relocalization, map revision change, or covariance/innovation failure invalidates the active route
and requests replanning; it does not silently alter a local planner's physics or safety thresholds. This is an
integration contract and a focused test, not a reason to place scan matching inside local planning.

The current working-tree changes in local planning belong to the independent orchard recovery and trajectory-safety
work package. They are not part of known-map localization and must be reviewed and committed separately so a future
localization change can be verified without changing planner behavior.

The first implementation is deliberately smaller than full SLAM: deskew/project a planar scan using its acquisition
times and the motion estimate, remove invalid/self/ground/dynamic returns, seed a deterministic coarse-to-fine
scan-to-map search from odometry, estimate local uncertainty, then gate score, overlap, innovation, timestamp and map
coverage before fusion. It must reject empty, repetitive, feature-poor or badly initialized scans instead of emitting a
confident pose. Simulator-truth pose remains available only as a test oracle and explicitly labelled instrumentation.

The first ROBOT-owned slice is now present as `KnownMapLocalizer`. It consumes a timestamped planar scan, an
odometric SE(2) prior and an immutable `MapSnapshot`; it performs a bounded deterministic occupancy endpoint search,
reports overlap/score/covariance and rejects stale, empty, invalid or weak matches. `AutonomyPipeline` exposes the
same provider-neutral operation, and the SIM harness accepts `localization_mode: known_map` to exercise it without
publishing a simulator absolute-pose correction. This is a component gate, not the P2 exit: deskew, dynamic/ground
filtering, covariance/innovation gates, held-out trajectory evidence and migration of normal recommended levels are
still required.

The correction policy is now explicit in ROBOT's `LocalizationSupervisor`. A known, low-uncertainty estimate runs
bounded local scan-to-map correction only when motion or an idle deadline makes it due; a rejected local match is
evidence and does not by itself launch a whole-map search. Unknown start, invalid/high estimator uncertainty, or a
lost estimate requests rate-limited global recovery. The local matcher receives an `IMapProvider::query_region()`
view, while the global provider alone receives the complete map. Estimation owns the propagated position/heading
uncertainty and fuses accepted measurement covariance; the supervisor does not maintain a second drift model.

Localization quality is a multi-signal ROBOT output: position/heading sigma, match score, overlap, separated-hypothesis
margin, innovation consistency, and observation age are retained separately. Sigma is the recovery gate, while quality
evidence is published to behavior and local planning. `RobotExecutive` is the task-level owner of a lost/relocalizing
decision (pause, safe return, or retry); local planning and control execute the resulting speed/stop constraint and
continue to apply independent swept-collision and braking validation. The executive publishes a mode, not an absolute
velocity or brake torque; the chassis-aware local planner converts quality into a speed envelope. A degraded but
usable estimate caps ordinary speed; confidence never relaxes a hard safety constraint. The local planner consumes localization health directly; the executive does not relay a speed or stop value through the planning request.

The authority ordering is explicit: task intent is advisory to motion, executable velocity is owned by chassis-aware
local planning and control, and collision/stability safety is the final gate. When a predicted dynamic intrusion makes
waiting unsafe, the planner may compare braking, retreat and other bounded escape candidates, but every candidate goes
through the same independent swept-geometry, actuator, terrain and stability validation. A task cannot force an
unvalidated escape; if no safe candidate exists, the result is an explicit no-safe-action diagnostic.

This migration is now applied to the shipped level catalog: every production level that previously selected `fused`
selects `known_map`, and the SIM session no longer calls `capture_pose()` or publishes an `absolute_pose` measurement.
The spawn marker is the declared initial-pose contract for a level; its first LiDAR scan still passes the local map
matcher. Explicit `odometry` levels remain only for teaching estimator drift. `fused` is no longer a valid level
localization mode.

Focused gates use analytically generated scans on asymmetric maps, known SE(2) offsets, empty and symmetric geometry,
occlusion/outliers, moving objects, scan motion distortion, slopes, latency, dropped scans and wrong initial guesses.
Trajectory gates compare odometry-only drift with LiDAR-corrected error over distance and turns, verify that rejected
matches do not jump the map frame, and replay the same observation trace deterministically under GCC and Clang. Normal
recommended missions must complete with truth correction disabled.

This stage is map localization, not map building. A later SLAM provider may produce LSRM occupancy/elevation plus
localization constraints through the same interfaces. A future 16-line LiDAR-inertial provider must use per-point time,
ring and IMU extrinsics for deskew; local odometry, global loop closure and map persistence remain separate contracts.

AMCL/particle relocalization is a separate P2 follow-up for cold start, kidnapped-robot recovery and large initial
pose error. It must publish the same pose-observation contract and confidence state as scan matching. It must not be
used to hide a weak local matcher, and it must not make the planner depend on particle-filter internals. Continuous
localization and global recovery may select different providers, but both must trigger the same route invalidation and
replanning policy when the pose jump exceeds the active estimator gate.

For Lost Signal's small authored maps, the deployable no-GNSS startup path is **known-map global localization**:
LiDAR observations and odometry/IMU motion are evaluated against the whole bounded map using AMCL/particle recovery or
a similarly global scan matcher. It must support an unknown initial pose, reject symmetric or feature-poor ambiguity,
and expose `UNLOCALIZED`, `LOCALIZED`, `DEGRADED` and `RELOCALIZING` states. The robot may perform a bounded in-place
observation sweep before accepting a pose; this is a localization behavior and must not be hidden inside DWA. The
`GlobalMapLocalizer` plus `StartupLocalization` now implement this bounded cold-start contract; the existing bounded
`KnownMapLocalizer` remains the local-correction component. Startup tests cover no-scan waiting, unknown-origin global
recovery, rejection without a fabricated pose, and the mapless fallback. AMCL/particle recovery and a physical
in-place sweep remain P2 extensions for larger initial error, kidnapped-robot recovery and symmetric maps. The SIM
`known_map` adapter now exercises the same startup/recovery path rather than silently relying on a fused pose.

The complementary **mapless online-SLAM path** starts with an arbitrary local origin, builds submaps while moving and
estimates its own pose from LiDAR plus the IMU/encoder prior. It does not require RTK or GNSS for a small self-contained map.
When the map is later saved, its local origin and provenance become the map anchor; a future run can use known-map
global localization to recover that pose. Both paths publish the same pose/state contract, while mapless SLAM also
publishes validated LSRM revisions and retains its pose graph/session data separately.

The delivery order is intentional: the known-map provider supplies the recovery contract, and the mapless provider now
reuses it through `OnlineSlamMapBuilder`. The online builder owns a bounded local submap and its local pose; it must
not create a second pose type or make Planner depend on a SLAM backend. In mapless mode there is no external
localization input, but the builder estimates its own pose internally from the first observation onward.

The current implementation is the first executable SLAM slice: deterministic occupancy integration, bounded
correlative scan alignment around the odometry/IMU prior, per-ray motion deskew from scan timing and body twist,
immutable map revisions, timestamp rejection, bounded 16-line point-cloud projection with height rejection and
per-return elevation publication, plus a ROBOT-only SE(2) pose graph with fixed-anchor sparse Gauss--Newton and robust
loop-closure edges. Pose-graph tests cover a drifted square loop, anchor preservation, invalid information, unknown
nodes, duplicate nodes, self edges and disconnected graphs. The bounded `localization_mapping_replay` gate exercises
fixed-timestamp scans with bounded wheel/IMU drift. This is component evidence: it does not claim dynamic-object
filtering, calibrated multi-view 2.5D elevation fusion or production large-map localization. Keyframe-triggered
rerasterization is now covered as a transactional component, but still needs long repeated-loop replay. This boundary
follows the established SLAM Toolbox/Cartographer pattern: serialized pose-graph/session data
is retained separately from an exported navigation map, loop closure updates poses, and the map is rebuilt from the
associated scans. The project keeps the first solver small and provider-neutral, but adopts those contracts before any
third-party optimizer is considered.

The next P2 gates are:

1. **Completed component gate:** `MappingSession` now selects bounded keyframes from motion, heading and time thresholds, retains
 timestamped session scans, creates the sequential odometry chain, and applies a separate loop-closure evidence gate
 for temporal separation, spatial candidate distance, overlap, score margin and information validity. Complete
 reconstruction of an optimized map revision now has a separate rebuilder that regenerates occupancy/elevation from
 retained keyframe scans and per-return motion. `SlamMapPublication` now optimizes a graph copy, rebuilds from corrected
 poses, validates the candidate, and only then atomically calls `MapManager::replace`; old immutable revisions remain
 available when optimization, reconstruction, or publication fails. The graph correction gate rejects non-finite,
 non-improving, or implausibly large corrections. The production integration gate still requires dynamic filtering and
 held-out replay evidence. `SlamSession` now composes this transaction end to end: timestamped frontend keyframes feed
 a bounded scan-based loop detector, accepted evidence enters the existing gate and pose graph, corrected keyframes
 rebuild an immutable map revision, and the frontend adopts the same rigid correction. The active `MapProvider` supplies
 the map identity to both frontend and rebuild configuration, so a caller cannot publish a revision under a different
 `map_id`. `test_slam_session` covers drift, repeated loop insertion, monotonic revisions, publication and identity
 preservation; this remains a deterministic short fixture, not evidence of long repeated-loop robustness;
2. **In progress:** calibrated ground model and dynamic-return filtering, with repeated point-cloud elevation fusion and
 uncertainty. The ROBOT `ElevationFusion` component now combines multi-view heights by inverse variance, includes pose
 uncertainty, bounds stale evidence with a forgetting factor, and exposes disagreement as uncertainty instead of
silently publishing a precise but contradictory terrain value. `OnlineSlamMapBuilder` and `KeyframeMapRebuilder` now use
this fusion path for published elevation cells. `test_online_slam` now replays a multi-frame 16-line point cloud: a
stable elevated return retains an occupied cell and a fused height with bounded uncertainty, while a return that stops
being observed is cleared by subsequent free-space evidence. This is end-to-end ROBOT-side replay, not calibrated field
terrain evidence; calibrated ground classification and held-out 16-line replay remain open;
 The `OccupancyLogOdds` component now provides bounded inverse-sensor-model evidence with hysteresis; it is used during
 keyframe map reconstruction and has an isolated transient-return regression. It is intentionally not yet enabled in the
live online submap until the evidence path below was validated. It now runs there as well: free-space carving clears a
transient return only after the configured evidence accumulates, a confirmed wall survives a stray clearing ray, and
elevation is fused only for endpoints that have earned occupied evidence. This remains sensor-only transient filtering;
it does not claim semantic actor classification or replace the perception tracker. The ROBOT point-cloud projection now
consumes estimated gravity-referenced roll/pitch and publishes an optional per-ray gravity height through LSRI schema
5. A sloped-platform component replay proves that a ground return is rejected while an elevated return remains occupied
and receives the transformed elevation. This closes the attitude-transform contract; calibrated sensor mounting/terrain
parameters and held-out 16-line long-duration replay remain open evidence gates.
3. a 200,000 m² map-localization benchmark using tiled/windowed map access and a coarse place index. It must measure
 repeated cold-start accuracy, ambiguous-pose rejection, deterministic tie handling, p95 latency and peak memory;
 exhaustive global raster search is retained only as a small-map reference;

 The global recovery matcher now follows that contract structurally: it builds a bounded coarse distance field,
 ranks map-wide place hypotheses with endpoint evidence, refines only the retained hypotheses, and validates the
 final poses with free-space ray evidence. It rejects when the coarse budget is exceeded instead of treating a
 partial sweep as a localization result. `test_large_map_localization` now also performs three deterministic
 no-prior recoveries on a 240,000 m² map (0.071 m position error, 0.009 rad heading error, roughly 1.0 s p95),
 including exact repeat agreement. This is an executable component gate; a held-out 200,000 m² cold-start timing and
 accuracy fixture on the deployment computer remains an exit gate.
4. **Component gate added:** `large_map_long_run` replays 64 repeated LiDAR corrections against a 200,000 m² map,
 with a persistent 0.35 m odometry prior error, quarterly error tracking, p95 latency, peak RSS and kidnapped-prior
 rejection. The current development result is 0.05 m maximum correction error for this synthetic fixture, p95 about
 116 ms and peak RSS about 61 MiB. The localizer now precomputes the map-grid transform per call, and this stability
 fixture uses a documented 32-return/10 cm bounded profile; the separate 180-return accuracy fixture remains intact.
 This is a repeatability gate, not field evidence; long loop closure replay and held-out terrain remain required before
 normal levels rely on mapless SLAM.

5. **Component gate added:** `localization_environments` runs one matrix over six environments instead of one fixture:
 shelved warehouse convergence, a translation-invariant long corridor, a quarter-turn symmetric hall, a featureless
 hangar, a warehouse scan with 20 % dropout plus outliers plus an unmodelled crossing object, and a ten-pose route
 through a doorway into a repeated-column hall. Each case asserts the correct outcome for its observability, not a
 uniform success. The matrix found a real defect: the endpoint score is quantised, so adjacent 5 cm samples of one
 solution tie exactly and the old margin reported a well-observed pose as geometrically ambiguous (six of ten facility
 poses were rejected). `KnownMapLocalizer` now retains separated hypothesis peaks (`ambiguity_distance_m`,
 `ambiguity_heading_rad`) and measures the margin between distinct hypotheses; the corridor still rejects with zero
 margin, while the facility route accepts 10/10 poses with zero wrong accepts and 0.25 m worst error. The reported
 large-map p95 latency is one continuous local `localize()` call in a bounded 16 m window, measured on the development
 host; it excludes cold-start global search, map publication, sensor transport and scheduling.

6. **Native provider gate added:** `ProviderConformanceMonitor` now runs on the ROBOT side of the LSRI boundary. It
 checks endpoint source/frame identity, payload structure, monotonic timestamps and sequence numbers, freshness and
 future skew, declared-versus-observed rate, and source silence; absolute pose input is rejected unless explicitly
 enabled. `test_provider_conformance` covers a healthy mixed LiDAR/encoder/IMU/power stream, duplicate and reordered
 samples, stale input, wrong frame, silence, rate mismatch and undeclared absolute pose. This makes the native guide's
 conformance step executable without coupling ROBOT to a vendor SDK. Device calibration and emergency-stop evidence
 remain deployment gates.

Editor-authored maps do not fabricate SLAM keyframes. Their localization index is a derived artifact of the authored
map revision. Continuing mapping adds keyframes in a new or extended mapping session; editing or importing map geometry
creates a new map revision and rebuilds the index rather than silently rewriting historical session constraints.

The SLAM audit keeps three gaps explicit. `OnlineSlamMapBuilder` is a bounded local mapping provider with deterministic
replay evidence, but it is not yet the normal provider for every mapless level. Pose-graph optimization and keyframe
rerasterization are transactional components, while loop-closure candidate generation and long repeated-loop replay are
not yet a production gate. Dynamic-return filtering currently provides sensor-level transient evidence and elevation
uncertainty; it does not claim semantic actor filtering or lidar-inertial observability. These are integration and
evidence gaps, not reasons to reintroduce simulator-truth pose.

Interfaces are adapted before either implementation is considered complete: LSRI carries raw timestamped sensor data;
`ILocalizationProvider` publishes pose, covariance and readiness/degradation state; `IMapProvider` serves immutable
static revisions; and a future `IMapBuilder` publishes complete candidate revisions plus session/checkpoint metadata.
Planner reads only `RobotState` and a pinned `MapView`. Map revision publication or a pose jump invalidates the
affected route and requests a fresh plan through the existing pipeline.

The same P2 package also freezes the `RobotExecutive` boundary. `TaskExecutive` remains the tested lifecycle kernel,
while the deployable coordinator above it accepts provider-neutral task requests and coordinates the autonomy providers. The
coordinator consumes state, perception summaries, map status and health; raw LSRI packets stay with the estimator,
mapping and perception providers that own them. A SIM adapter may submit the same request and expose simulated LSRI
endpoints, but it must not pass game objectives, actor truth or ScenarioSession state into ROBOT. The brain's output is
intent and lifecycle state; local planning, safety validation and LSRI remain responsible for executable commands.

**Brain exit:** a request submitted by a simulation adapter and by a provider-neutral test client produces the same
ROBOT task lifecycle and autonomy inputs; replacing the SIM LSRI provider with replay or native endpoints requires no
brain or planner code change. Pause, cancel, stale-observation and provider-health failures have explicit outcomes,
power telemetry has an explicit stale/missing policy for reserve enforcement, and no game-only type is reachable from
`lost_signal_robot_executive`. Battery/charger physics remains a provider concern; the executive only consumes the
typed LSRI power observation and publishes a mission-level resource directive.

The native deployment workflow is documented in [Native hardware integration](native-hardware-integration.md): native
providers, calibration and endpoint capabilities replace SIM providers at the composition root while ROBOT algorithms keep
their typed contracts. The guide is a bring-up contract, not evidence that a particular vendor driver or real robot has
already been validated.

The design follows the useful boundaries in [REP-105](https://www.ros.org/reps/rep-0105.html), Nav2
[AMCL](https://github.com/ros-navigation/navigation2/tree/main/nav2_amcl), Cartographer's
[local/global SLAM split](https://google-cartographer-ros.readthedocs.io/en/latest/algo_walkthrough.html), Autoware's
[NDT scan matcher](https://github.com/autowarefoundation/autoware_core/tree/main/localization/autoware_ndt_scan_matcher)
and [EKF localizer](https://github.com/autowarefoundation/autoware_core/tree/main/localization/autoware_ekf_localizer),
and LIO-SAM's [time/ring/extrinsic requirements](https://github.com/TixiaoShan/LIO-SAM). These are semantic and
validation references; Lost Signal keeps a compact project-owned provider interface and does not import ROS2.

**Exit:** a standard-LiDAR reference run localizes against an LSRM map without simulation pose truth, reports calibrated
quality limits and bounded held-out trajectory error, survives declared dropout/degeneracy cases safely, and produces
the same accepted/rejected observation sequence on replay. A provider-conformance test must run the same planner and
controller trace with odometry-only, known-map and a recorded-relocalization source; planner commands may differ only
when the supplied pose/state differs, never because the planner branches on provider identity. A rejected match,
accepted small correction and large relocalization jump each have explicit route/replan assertions.

The collision monitor and decision-evidence state machine have also moved to ROBOT-owned autonomy libraries. SIM keeps
compatibility type aliases and supplies world/physics observations; it no longer compiles those algorithms as mission
implementations. Remaining SIM-side autonomy audit items are tracked separately from content, actor behavior and plant
code, which remain simulation responsibilities.

The passive braking measurement state machine has now moved to `robot/autonomy/diagnostics/vehicle_measurement`; the
SIM header is only a compatibility alias and the SIM target no longer compiles an implementation. It observes command
transitions and measured motion for diagnostics and scoring evidence; it does not replace the physical brake actuator,
the trajectory validator or the safety stop limiter.


### P3 — Re-establish the release candidate evidence

After P1 and P2 are frozen, run the gates in [Validation](validation.md): GCC 13 Release and the complete CTest suite
with parallelism 8; ROBOT-only build and tests; schema, content, locale and generated-artifact checks; identical-seed
run artifact reproduction; graphical input, shutdown and recommended-experience smoke tests; Clang 18 compatibility;
and ASan/UBSan last. Record the exact revision, commands, counts and results. A historical green baseline does not cover
a new autonomy, localization or simulation revision.

**Exit:** the current clean revision has one complete release record. It may be published as an Alpha / Developer
Preview with the claim boundaries stated in the simulation audit.

### P4 — Validate Stormglass as the gameplay vertical slice

Stormglass already provides three player-selected signal cores, autonomous goal execution, payload and centre-of-
gravity changes, authored storm keyframes, changing grip and a moving service cart. Use the playtest protocol to find
where a new player cannot identify the danger, choose a goal, understand waiting, complete or diagnose the run, and
explain one causal effect. Revise the data-authored task loop and information rhythm only where evidence shows a
problem. Existing campaign content remains selectable.

**Exit:** three fresh players can complete or diagnose the loop without steering through the autonomy problem; at
least two target orders are safely completable on one seed and produce measurably different route, time, payload,
clearance or grip evidence.

### P5 — Give the approved loop a distinctive presentation

Refine Stormglass with authored models, readable landmarks, vehicle details, materials, weather, bounded particles,
lighting, camera staging and state-driven sound. Preparation, target choice, risk, pickup, waiting, release, damage and
debrief must remain readable without opening AI View. Every effect reads snapshot state and cannot manufacture a
physics or autonomy outcome. Headless execution remains independent of graphical assets.

Apply the responsive information-design gate below to real flows: compact symbols, measured localized text, hover
hints, responsive panels, immediate mouse input and clean shutdown. Broad screenshot work remains secondary to a
concrete blocked flow.

**Exit:** the complete experience is visually coherent, responsive and understandable in English and Simplified
Chinese; deterministic graphical smoke tests contain no clipping, missing assets or state/render disagreement.

### P6 — Complete the causal learning and comparison loop

Upgrade AI View and the debrief from internal-state display to a causal chain: observation, prediction, rejected
candidates, selected constraint, action and measured outcome. Let a player rerun the same seed after changing one
declared variable and compare complete run artifacts and key trajectory metrics. Keep simulator truth and onboard-
observable estimates visually distinct.

**Exit:** a player can explain why the robot waited, moved or stopped, change one variable, and see a reproducible
causal difference rather than only a different success boolean.

### P7 — Generalize experiments and deterministic fault injection

Extract only the needs demonstrated by the finished content into Scenario, Experiment, Baseline, Metric and Diff
contracts. Add seeded provider-level faults for delay, loss, reordering, stale timestamps, sensor noise, actuator lag,
saturation, slip and changing load. Preserve the in-process fixed-step runtime while keeping LSRI provider boundaries
suitable for future threads, processes and native devices.

**Exit:** a content-authored experiment can compare algorithm or configuration variants without UI-specific mission
logic, and faults traverse the same validation and safety paths as ordinary observations and commands.

### P8 — Expand capabilities only for a named question

Add further sensors, algorithms or execution modes only when a specific learning experience or measured platform
target supplies acceptance criteria. TEB remains a planned DWA comparison provider behind the shared
`LocalPlanningRequest` → `ExecutableTrajectory` → `TrajectoryValidator` contract. Camera, map-building SLAM,
solid-state LiDAR, multi-process LSRI Wire, HIL, VLA, native drivers, ABS, parking brakes, identified tire laws and
more complex manipulation remain candidate work rather than release dependencies. Higher-fidelity electric-drive
work is one named package: separate motor winding/back-EMF and torque constants, gearbox ratio/efficiency/reflected
inertia, inverter bus voltage/current/PWM limits, nested current/speed control, and calibrated energy/regeneration.
Until that package exists, `energy_scale` remains only a relative mission-energy estimate and never changes physical
actuator effort. Fidelity claims advance only for behavior that is both implemented and validated; adding fields or
describing a future model does not advance the claim.

**Exit:** each new capability ships with a named scenario, quantitative evidence, documented limits and a smaller
project-owned interface than any external system it may bridge to.

The [verification strategy](verification-strategy.md) is a continuous gate across P0--P8. Contract correctness,
mathematical invariants, independent oracles, deterministic system evidence, fault boundaries, and physical
validation accumulate in that order. Correct an unreleased algorithm or interface directly, update every caller and
test, and then update the content expectation; do not preserve an incorrect result through compatibility behavior.

### Responsive information-design gate

Player-facing layouts measure localized text instead of assuming Chinese-length copy. Chapter, garage, prediction,
running, and debrief panels use vertical flow or scrolling; configuration choices remain readable at the minimum
1100×720 window and at the reference 1440×900 capture size. English and Simplified Chinese screenshots must show no
clipping or overlap.

Persistent screen space is reserved for the current goal, active safety state, and decision evidence. Repeated
controls, component families, map layers, and diagnostic categories use a small shared symbol and color vocabulary.
Hover hints expose the full localized name, units, and consequence; the compact label still communicates the action
for screenshots, keyboard navigation, and users who never hover. A tooltip cannot be the only place that contains a
safety-critical value or the current autonomous action.

The AI view applies that vocabulary as a compact legend: cyan identifies the global route and LiDAR evidence,
neutral white records actual motion, violet shows local-planner samples, amber marks the selected local trajectory,
and red identifies predicted conflict. Hovering a legend row explains the producer and semantic meaning; hovering a
terrain-layer chip explains the active measurement. The current decision and safety clearance remain visible.

## Completed alpha baseline

The following stages record work already completed for the original `lost_signal` vertical slice. They are retained
as release evidence and are not the queue for upcoming work.

### Alpha Stage 1 — Design and implement the flagship encounter

Status: **complete**

`lost_signal` is the content target. Its learning question is:

> Why must an autonomous robot predict future motion instead of reacting only to an obstacle's current distance?

The mission uses Predict–Observe–Explain as its learning rhythm. The player commits to an idea before launch, watches a physical consequence, and then tests the explanation against recorded evidence. This follows the exploration and discovery principles documented by [PhET research](https://phet.colorado.edu/en/research) and the low-barrier model inspection described in [Explorable Explanations](https://worrydream.com/ExplorableExplanations/).

### Player contract

The player chooses the sensing/computing setup and driving policy, answers one concrete prediction question, and dispatches RO-1. RO-1 owns route selection, sensing, motion prediction, waiting, crossing, steering, braking, and actuator output. The authored mission never asks the player to steer around the difficult part.

The garage should emphasize variables that can explain this encounter. Unrelated component choices remain in the build but do not compete for attention on the flagship preparation screen.

### Three-beat experience

| Beat | Simulation trigger | What the player sees | Learning purpose |
|---|---|---|---|
| Carry the spark | Mission start through power-module pickup | A dark signal tower establishes the destination. RO-1 routes around the storage racks and retrieves a visible power module. | Establish intention, route, and the cause of the coming world change. |
| The blind crossing | Power pickup summons depot traffic; the planned route approaches the conflict region | The depot wakes up, a forklift emerges from behind an occluding rack, and RO-1 derives a safe stop pose from current route and vehicle state, predicts, brakes, and waits when needed. | Turn an apparent hesitation into a readable safety decision. |
| Send the signal | The predicted conflict clears and RO-1 reaches the tower | RO-1 commits to the crossing, docks at the relay, and the tower changes from dark to a valley-wide pulse. | Pay off the prediction with a visible outcome and a concise evidence chain. |

The route and timing come from the simulation. Presentation may amplify a real state transition, but it may not invent traffic, detection, clearance, or success.

### Information rhythm

The field view must answer “what is happening?” before a panel is opened. It uses short, event-driven cues:

1. Power module collected; depot traffic is now active.
2. Moving object observed; motion is being estimated.
3. Predicted path conflict; holding outside the crossing.
4. Conflict cleared; crossing committed.
5. Relay powered; signal restored.

AI View answers “what evidence caused it?” with the same event IDs and measurements. The debrief answers “was the prediction supported?” and offers one relevant variable for a retry. Repeated frame-by-frame chatter is not content.

### Failure and retry design

- A dynamic collision rewinds to the earliest useful observation, then shows the prediction and rejected or unsafe action.
- A timeout identifies the longest constrained interval and whether sensing, policy, or route progress caused it.
- A static collision identifies the occupied geometry and commanded local motion.
- A successful run still exposes the wait duration, closest predicted gap, and crossing decision so safety does not look like luck.
- Retry keeps mission and seed fixed and encourages one change among sensor rate/range, prediction horizon/computer, or driving policy.

### Stage 1 acceptance gate

- The preparation screen asks one question and foregrounds only relevant choices.
- The three beats are distinguishable without AI View or documentation.
- All player-facing English is concise and natural; Simplified Chinese covers the same keys.
- The recommended build completes deterministically with zero collisions.
- At least one controlled alternative produces a different, explainable observation or decision without relying on random timing.
- Success, dynamic collision, static collision, and timeout have specific debrief paths.
- Content, locale, headless, and mission regression checks pass.

The original clearance comparison was invalidated by the corrected nominal/conservative prediction split. The
current deterministic MuJoCo evidence keeps both builds safe: the performance computer preserves a larger dynamic
clearance, while the otherwise identical light-computer build finishes slightly sooner and consumes less energy.
This is now taught as a clearance/compute/energy tradeoff. A future fault-injection experiment will provide the unsafe
counterexample without depending on a known collision in the normal controller.

### Alpha Stage 2 — Make decision evidence a shared runtime contract

Status: **complete**

Introduce a compact, structured decision record produced by autonomy and mission systems rather than inferred by UI code. A record contains:

- simulation and observation timestamps;
- observation and tracked-object references;
- prediction horizon and measured/predicted separation;
- active constraint and stable reason code;
- selected action and relevant target values;
- outcome or superseding decision.

The field cue, AI View, debrief, headless JSON, and tests consume this same record. Localized prose maps from stable reason codes; algorithms do not emit presentation sentences. Replay seeking must reproduce the record visible at that simulation time.

The gate is a deterministic `lost_signal` run whose observation → prediction → constraint → action → outcome chain is identical in the graphical client, headless result, and regression assertions.

The implemented contract records compact decision transitions rather than every controller tick. The
recommended seed-47 run currently produces 72 records for 892 local-planning cycles. Replay snapshots carry
the record active at that simulation time; headless JSON emits the closed transition chain; live field copy,
AI View, debrief, and regression tests read the same fields and localize only stable reason/action codes.

### Alpha Stage 3 — Build the state-driven audiovisual vertical slice

Status: **complete**

Improve the research-base scene after the encounter and evidence timing are stable:

- create distinct silhouettes for the power cradle, conflict region, occluding cargo rack, and signal tower;
- improve RO-1 and forklift materials, readable fronts/rears, wheels, mast, work lights, and warning beacon;
- use worn metal, safety paint, damp concrete, screen emission, and controlled color contrast to separate navigable space from industrial clutter;
- bind tower power-up, forklift beacon, RO-1 waiting lights, tire marks, dust, and water effects to real snapshot state;
- add a small original sound set for power pickup, moving-vehicle warning, wait/clear transitions, collision, and tower restoration;
- keep meshes low-poly, particle pools bounded, audio optional, and all graphics absent from headless loading.

The gate is a deterministic screenshot sequence at the preparation, conflict, wait, crossing, and success moments, plus a graphical smoke test with no missing assets or shader warnings. Each visible or audible event must name the state that drives it.

The vertical slice now keeps its encounter landmarks physically present while their state changes. The power
module disappears from its cradle after `goals_reached` advances; the conflict overlay reads the live autonomy
phase; and the signal tower remains dark until `MissionStatus::Succeeded`,
then emits a cyan pulse and expanding signal rings. The forklift beacon reads actor velocity, detection, and
`waiting_for_clearance`; RO-1 lamps read that same wait state; tire marks and bounded dust, spray, and skid smoke
read replayed motion, surface, and traction state. The graphical-only audio director consumes `MissionEvent` and
`DecisionEvidence` transitions for pickup, traffic warning, wait, clear, collision, and restoration cues. `--mute`,
demo, screenshots, and every headless target avoid audio initialization.

The local gate passed with byte-identical 1440×900 images from two runs at 2.0, 23.7, 25.0, 26.5, and terminal
simulation time. The generated WAV set also passes byte-for-byte `--check` reproduction.

### Alpha Stage 4 — Add a minimum reproducible run artifact

Status: **complete**

Write one versioned artifact for a completed run. It records source revision, build identity, platform/backend, content hash, mission and condition, seed, hardware selection, algorithm IDs, configuration identity, result metrics, structured decisions, and causal events. Large sensor traces remain external and are referenced by identity when they exist.

Add a small comparison command that rejects incompatible comparisons and reports declared metric differences for like-for-like runs. Interactive UI replay, LSRI observation replay, and the run artifact remain separate formats with separate purposes.

The gate is reproduction of the recommended `lost_signal` result from its manifest plus a clear diff against one controlled alternative.

Run Artifact v3 now records source/build/platform identity, a SHA-256 JSON-content manifest, authored condition,
seed, runtime, physics engine and version, hardware, algorithm IDs, a canonical configuration digest, metrics, events,
the shared decision chain, and constructed-world plan/contact/traversal evidence. `--reproduce` rejects incompatible content and restores the declared run.
`tools/compare_runs.py` verifies both digests, forbids invariant changes, and requires each controlled variable to
be named. A manifest reproduction matched every reported metric. Changing only
`hardware.components.computer` remains the declared controlled variable for the comparison artifact. Exact metrics
are regenerated at release time because corrected autonomy contracts intentionally invalidate old baselines; the
comparison must report the current time, energy, and clearance values rather than preserve obsolete score claims.

### Alpha Stage 5 — Release closure

Status: **complete**

- finish English game copy and Simplified Chinese game localization;
- keep source, tests, headless output, and developer documentation in English;
- update learning, architecture, validation, asset-provenance, and sim-to-real documents to match shipped behavior;
- remove temporary screenshots, generated diagnostics, stale references, and unneeded workspace files;
- verify licenses and provenance for every shipped asset;
- run content and locale validation, the full headless suite, graphical smoke tests, and deterministic screenshots;
- run sanitizer checks for milestone and release-candidate tags;
- publish only after Linux, macOS, Windows, sanitizer, and graphical CI jobs pass.

The release claim remains deliberately narrow: Lost Signal demonstrates deterministic simulation, inspectable autonomy, reusable runtime semantics, and a path toward hardware integration. Real-world transfer requires calibrated hardware trials and recorded evidence.

The current Linux x86_64 Release build passes 90/90 tests, content and locale validation, canonical content
roundtrip, the five-chassis MuJoCo proving matrix, and a byte-identical Bridge screenshot repeat under Xvfb. The
most recent sanitizer baseline predates this Bridge increment; ASan/UBSan remains the final frozen-milestone gate as
planned. Portable Linux, macOS, and Windows CI remain publishing gates; local Linux evidence does not replace those
hosted jobs.

## RO-1 post-baseline optimization backlog

RO-1 now closes the reference control chain from body intent through deployable wheel PID and LSRI normalized-duty
commands to the simulated motor, articulated wheels, tire contact, rigid body, and measured joint feedback. The
items below strengthen timing, fault evidence, and physical fidelity; they do not reopen that ownership boundary.

### Release-candidate gates

1. **Sensor delivery time and fusion alignment.** Keep acquisition time immutable, delay simulated delivery by the
 configured device latency, and align encoder, IMU, pose, and LiDAR updates by sample time. Tests must cover delayed,
 jittered, stale, dropped, and out-of-order packets without using arrival time as the measurement timestamp. The
 current LSRI header carries latency, but shipped component profiles use zero delivery delay and therefore do not yet
 exercise this path end to end.
2. **Integrated command-loss safety.** Exercise the complete RO-1 loop with skipped and expired actuator commands,
 verify LSRI emits both channels' declared safe values, and measure stopping distance and residual wheel effort. The
 existing channel-level watchdog test remains necessary but is not the full plant-level proof.
3. **Long-run numerical stability.** Add bounded energy, pose, contact penetration, wheel-speed, and temperature-free
 motor-state drift checks over longer stationary and constant-speed runs. Compare at least two internal contact step
 sizes with tolerance envelopes rather than requiring bit identity between different solvers or platforms.
4. **Final release execution.** Run ASan/UBSan after the functional work is frozen, then graphical smoke,
 deterministic screenshot, packaging, and Linux/macOS/Windows CI gates. Record the exact commit, compiler, asset
 hashes, scenario, seed, configuration, and result artifacts.

### Optional higher-fidelity profiles

5. **Electrical and drive effects.** Add named profiles for duty deadband, supply-voltage sag, current/torque limits,
 regenerative or resistive braking, gearbox drag/backlash, command delay, and thermal derating. Each effect remains
 independently switchable and requires a measurable maneuver; it must not become an unexplained default constant.
6. **LiDAR fidelity families.** Extend the existing planar first-return profile with optional beam divergence,
 weather attenuation, multi-return behavior, packet timing, and later multi-line spinning and solid-state patterns.
 All variants publish the same LSRI time/frame/source contract and keep device-specific fields explicit.
7. **Payload mechanics.** Add payload collision geometry, attachment compliance, and load placement instead of
 changing only mass, centre height, and yaw inertia. Re-run static loads, braking, tip, slope, and rough-ground gates.
8. **Identified or HIL evidence.** A contributed physical target must record calibration sources and pass held-out
 wheel-speed, yaw, pose, stopping-distance, scan timing, range, and intensity error envelopes. Until then the project
 may state that its boundary supports transfer experiments, but must not claim proven sim-to-real performance.

RO-2 through RO-5 now have independent geometry, actuator, contact, and maneuver acceptance gates. Their typical
parameters remain engineering assumptions, and their completion does not weaken or silently generalize RO-1
evidence. Sensor evidence remains attached to the selected sensor profile rather than duplicated per chassis.

### Selected implementation sequence: 16-line LiDAR, then RO-2

Both expansions are approved, with this fixed order:

1. **16-line spinning LiDAR on RO-1.** First make configured sensor latency affect deterministic delivery while
 preserving acquisition timestamps. Then add data-authored vertical beam angles, ring and azimuth timing, 3D MuJoCo
 raycasts, LSRI point-cloud transport, ground/non-ground projection into the existing local obstacle layer, and a
 height/ring/intensity AI visualization. Analytic fixtures must cover flat ground, a vertical wall, a slope, a low
 obstacle, an overhanging obstacle, occlusion, intensity, ring identity, and motion-distorted per-point timing.
2. **RO-2 articulated Ackermann reference.** Replace the ideal single-body proxy with authored chassis, steering
 knuckles, four rolling wheels, explicit drive and steering actuator channels, physical joint feedback, tire/contact
 response, and deployable controllers across LSRI. The acceptance matrix must separately cover static wheel loads,
 straight/reverse motion, steering step/rate, common instantaneous turn centre, minimum-radius circles, braking,
 low-friction understeer/slip, cross-slope stability, command expiry, and long-run rest stability.

The 16-line profile must not be implemented as sixteen independent planar scans, and RO-2 must not be promoted by
retaining an analytic yaw-torque shortcut. Both features stay data-authored and use the same ROBOT/LSRI/SIM boundary
as RO-1. After their focused gates pass, existing missions are re-run; mission success itself is not the physical
oracle.

The 16-line implementation is functionally complete: timed 3D point clouds (introduced in schema 3 and extended by
schema 5 gravity-referenced ray heights), per-beam authored calibration,
three-dimensional firing-time raycasts, estimated-pose static reference, deterministic delayed delivery,
ground/non-ground projection, bounded 3D presentation, shared replay storage, focused geometry tests, and a repeatable
headless performance benchmark are implemented. A dedicated authored learning encounter and broader campaign
playtest remain content work.

The RO-2 articulated physical gate is also complete at the documented `typical` level. The data-authored assembly,
bounded front knuckles, four rolling wheels, rear drive, four-channel LSRI duty path, deployable Ackermann controller,
physical feedback, and MuJoCo contact run without an analytic yaw-torque shortcut. Independent tests cover the matrix
above plus motor-limited gradeability, zero-command rollback, public-step convergence, and 25 mm rough terrain.
Mechanical tie rods, identified parameters, HIL, and real-car evidence remain optional later fidelity work rather
than implied results of this gate.


### Deployed data ownership: ROBOT data versus game content

Tuned robot behaviour must not be compiled into the game. The repository now separates two data roots:

- `robot_data/` is deployable robot data: platform policy (currently
 `policy/motion_policy.json`), and the future home of calibration, map packages and per-device provider
 configuration. A native bring-up reads this directory unchanged.
- `assets/` is game content: levels, presentations, scenes, story, UI strings and the component catalog.

`MotionPolicy` fails loudly on a missing file, an unknown key, or an out-of-range value. This is deliberate: the
class of bug behind "sim-to-real will explode" is a tuned threshold that exists only in an executable.

### Remaining SIM to ROBOT migration

The algorithm bodies already live in ROBOT; what still sits in `src/sim/game/scenario/scenario_autonomy.cpp` and
`scenario_session.cpp` is the per-tick composition of those algorithms. It is the largest remaining piece of
robot-facing decision code inside the game. The migration plan is:

1. Extract `AutonomyCoordinator` into ROBOT with a typed request (task intent, estimated state with localization
 health, goal set, map provider, predicted dynamic frames, chassis capabilities, operator override) and a typed
 result (accepted command, motion-capability status, decision evidence, recovery reason). The function currently
 spans roughly 1,600 lines of one scenario method; it will move in stages, one behaviour group at a time, with the
 simulator calling the coordinator through LSRI-typed inputs only.
2. Move platform profile parsing (`assets/robots/*.json` drive, motor, collision, slot and mount definitions) into
 ROBOT, leaving the catalog responsible only for locating and handing over JSON. Render-only fields stay in a
 presentation struct.
3. Keep the mission and scenario layer responsible for authored goals, scoring, story and actor truth only. Any
 decision that would also run on a real robot must live behind a ROBOT interface.

Each stage keeps the deployed boundary test green: a ROBOT target that links SIM fails the build.

Progress on stage 1: the stop-authority latch, its release contract and its settle window now live in ROBOT
(`src/robot/autonomy/control/controlled_stop_latch.*`) and the scenario only assembles inputs. The upper-policy
transaction is now also ROBOT-owned by `src/robot/autonomy/pipeline/autonomy_coordinator.*`: policy selection and
the shared safety shield are applied together from estimated-state inputs, while SIM supplies only LSRI observations,
task targets and a provider-neutral map feasibility callback. The stateful route tracker/PID is now also owned by the
same ROBOT coordinator; SIM can configure it from the selected chassis profile and consume typed tracking results, but
does not own its state. The estimator instance is now owned by the coordinator as well; SIM feeds it LSRI observation
frames and reads the resulting estimated state, without retaining a parallel estimator. The scenario no longer calls
a policy or safety shield directly. The runtime thresholds that
the decision loop previously hard-coded — gravity, physical stand-off, decision
step, rollover risk limit, planning deadbands, committed progress floor, creep speed and the latch window — load from
`robot_data/policy/motion_policy.json` under `execution`, with strict validation and a unit test that fails if the
deployed file drifts from the compiled defaults. Remaining literals in the decision loop are catalogued below.

The coordinator now also owns the localization supervisor and global planning system. Chassis-specific global planner
selection remains a composition input, while planner instances and localization scheduling are ROBOT state. A SIM
scenario may provide a map and task waypoints, but it cannot retain a second planner or localization scheduler.

The coordinator extraction is a typed composition step, not the completion of SIM→ROBOT migration. Route-following,
dynamic interaction and local candidate assembly still need the same extraction treatment before the gate can close.

### Magic-number audit (decision loop)

Values still inline in `src/sim/game/scenario/scenario_autonomy.cpp` and their intended owner. Nothing here is a
silent default: each entry must either move to `robot_data/` or be justified as a fixed physical/algorithmic constant
before the next migration slice.

| Inline value | Meaning | Target owner |
| --- | --- | --- |
| `0.25` | Docking arrival radius and subgoal sampling stride | ROBOT docking policy |
| `3.0`, `1.4`, `0.7` | Docking range window and reverse hysteresis | ROBOT docking policy |
| `0.3` | Docking approach speed | ROBOT docking policy (platform-limited) |
| `0.20` | Minimum schedule speed while tracking a valid heading | ROBOT speed schedule |
| `0.5` | Ride excitation fraction of gravity | ROBOT ride-comfort policy |
| `0.35` | Crossing lane half-width used for admission geometry | ROBOT crossing policy |
| `1.2` | Fallback prediction horizon when a compute profile is absent | ROBOT compute profile |
| `1.0`, `1.3`, `0.25` | DWA critic weights (path, clearance, heading) | ROBOT local-planner profile |
| `0.18` | Inspect-goal terminal speed | ROBOT terminal-pose policy |
| `0.05`, `0.10`, `0.5`, `1.0` | Local lookahead bounds on committed crossing legs | ROBOT crossing policy |
| `2.0`, `0.01` | Local stop-plane activation distance and non-worsening clearance epsilon | ROBOT safety policy |
| `0.65` | Legacy research-map conflict reserve | Game content, mapped stop-line placement |

Fixed algorithmic constants that stay in code: numerical epsilons (`1e-9`, `1e-6`), rate expressions that only
convert an interval to a per-cycle delta, and the `0.5` factor that converts two tread speeds to a common-mode
longitudinal speed.

### Progress: platform profile is now ROBOT-owned

Stage 2 has started. The drive-train half of `assets/robots/*.json` — drive type, wheel geometry, mass and yaw
inertia, motor capability, steering servo, encoder, service brake and energy coefficients — is defined and parsed by
ROBOT (`src/robot/platform/robot_drive_profile.*`). The parser rejects unknown keys and range-checks every capability,
so a native bring-up reads the same contract the simulator validates. `mission::RobotDefinition` now derives from
`sim::RobotDriveProfile` and keeps only content metadata: title, presentation profile, garage label, unlock gate,
slots and parts. The simulator no longer owns the semantics of motor or brake capability.

Build layout is now one directory. MuJoCo is a required dependency of the simulation, so there is no separate
"MuJoCo build": `build/` contains the client, the simulation core and every test, and `lost_signal` plus
`lost_signal_headless` copy `robot_data/` next to their binaries so a packaged runtime resolves the same policy and
hardware files as the source tree.

### Known failing tests at this checkpoint

- `mission_simulation`: `slope_launch` still times out after `stability_hold` with a diverged odometry estimate.
- `orchard_scramble`: the balanced and constrained policies can still time out after a valid, collision-free run.
 The latest evidence is no longer a soft-inflation deadlock: in sparse orchard geometry the known-map matcher often
 has fewer than eight occupied returns, the correction age grows, and the estimator eventually loses its covariance
 validity or limits the speed envelope before the task can finish. The ROBOT-side change now separates stale map
 evidence from wheel/IMU covariance, treats valid sparse scans as a cautious envelope, and lowers the default local
 geometric minimum to four returns. Sparse scan matching now also scores free-space rays at a lower weight than
 occupied endpoints, preserving hit-supported pose estimates under partial occlusion. This removes the unconditional
 stale-age stop, but the mission still needs a held-out orchard replay with route-time budgeting and a measured
 observability strategy before it can be marked green.
- `stop_whatif`, `entry_shadow_town`, `ff_sensitivity` and `run_artifact` remain to be re-run after the planner and
 localization changes; no failure is waived by this checkpoint.

### Slope-lab localization mode

The three slope lessons (`slope_launch`, `cross_slope`, `rain_descent`) run on the robot's own wheel/IMU odometry.
The slope lab is a 20 by 14 m track whose only LiDAR features are the perimeter and four props, and its 20-degree ramp
makes a horizontally mounted planar LiDAR see the ramp surface as a wall. A planar endpoint matcher cannot separate
that ground return from an obstacle, so these lessons would stop on a localization hold for reasons that have nothing
to do with their teaching goal (traction, cross-slope stability, wet braking). The estimator still runs; it simply does
not receive a map correction. Levels with feature-rich maps keep `known_map` localization, and the boundary-wall
geometry is now part of every deployable map, so a LiDAR can never see a wall that the map does not contain.
