# Architecture

Lost Signal separates authored content, autonomous behavior, physical execution, and presentation so the same mission can support a game, a headless experiment, and future hardware adapters.

## System shape

```mermaid
graph TD
 A[Operator or external task request] --> X[RobotExecutive / TaskExecutive]
 X --> B[Behavior and goal management]
 B --> C[Global planner: A* or Hybrid A*]
 C --> D[Local planner and dynamic prediction]
 D --> E[Chassis-feasible command]
 E --> F[Kinematics and actuator targets]
 F --> G[Motor control and PID]
 G --> R[LSRI motion port]
 R --> H[Platform provider: SIM or real hardware]
 H --> I[LiDAR, IMU, encoders, contacts]
 I --> O[LSRI observation port]
 O --> J[State estimation, mapping and perception]
 J --> X
 J --> C
 J --> K[Immutable simulation snapshot]
 K --> L[3D field view]
 K --> M[2D AI view]
 K --> N[Headless JSON and replay]
```

The player, a real operator, or another application supplies `A` as a provider-neutral task request. The deployable
**RobotExecutive** (the robot brain) owns request validation, task lifecycle, mission-level priorities, recovery mode,
resource policy (power reserve and charging/return decisions), platform-health degradation and the coordination
of localization, mapping, perception, planning and control. It does not read a simulator scene or emit wheel commands
itself. `TaskExecutive` is the first implementation slice of this brain; the surrounding coordinator remains a ROBOT
composition root.

The brain normally consumes an estimated state, perception summaries, map status, platform capabilities and health
events. It does not need every raw sensor sample. The lower ROBOT modules may read the typed LSRI observations they
own (LiDAR, IMU, encoders, actuator feedback and point clouds) through the same observation port. LSRI is the data
boundary, not a rule that every algorithm must subscribe to every sensor. This keeps the brain above algorithms while
still allowing a new sensor provider to feed the appropriate estimator or perception module directly.

The command authority is ordered by consequence. RobotExecutive publishes task intent and operating mode; it never
chooses a wheel speed or brake torque. Behavior and local planning turn that intent, chassis capability, localization
quality and scene constraints into an executable velocity envelope. The controller tracks that envelope. The
independent safety validator has final authority over the candidate trajectory: it may reject a commanded transit,
require a reachable braking tail, or select a separately generated and validated escape when remaining stationary is
predicted to be unsafe. An escape is still subject to collision, terrain, actuator and stability checks. If no
collision-free action can be proven, the system reports that state instead of overriding safety to satisfy the task.

Autonomous code owns `B` through `G`. LSRI carries commands and observations across the platform boundary. `H` is
either the simulated platform provider or a native hardware provider; both implement the same contract. Sensors
observe `H`, and presentation reads immutable snapshots without changing the runtime.

## Layer responsibilities

### Runtime boundary

The [Lost Signal Robot Interface (LSRI)](runtime-interface.md) is the project-owned robot boundary. Its ROBOT port reads validated observations and writes expiring body-motion or actuator commands. Its platform side can be composed from simulated, replay, remote, and native providers one endpoint at a time.

LSRI is intentionally below the ROBOT algorithm graph. Localization, mapping, perception, planning, validation, control
and executive modules use typed ROBOT interfaces that preserve the same units, frames, time, quality, lifecycle and failure
semantics; they do not exchange opaque LSRI packets for every in-process call. This keeps high-rate modules replaceable
without coupling them to a sensor driver or simulator transport. An out-of-process module is adapted to a versioned LSRI
representation only at that process boundary, and the deployable boundary test keeps this graph ROBOT-owned.

LSRI remains a small project standard with no external middleware, executor, discovery, or device-SDK dependency.

### Content

`assets/` contains missions, maps, scenes, surfaces, robots, components, policies, presentations, story, UI strings, locales, and JSON Schemas. `ContentCatalog` loads and validates these resources into typed runtime definitions.

Locale files are sparse string-only overlays. They cannot change IDs, paths, seeds, physics, scores, or algorithms. English is the default runtime locale; Simplified Chinese remains available as `zh-CN`.

### Platform contracts and simulated devices

`src/robot/platform/`, `src/robot/control/`, and `src/robot/runtime/` contain deployable capabilities, kinematics, control algorithms, and LSRI contracts. `src/sim/hardware/vehicle/`, `src/sim/hardware/sensors/`, and `src/sim/hardware/physics/` contain virtual devices and physical plants. A chassis declares both its body command space and the lowest command level accepted by its provider. Algorithms consume these contracts without depending on MuJoCo or rendering.

Sensors carry sequence, sample time, declared latency, source, and frame ID. Noise affects the generated measurements.
The current in-process simulator queues LiDAR observations until their deterministic availability time while preserving
their acquisition timestamp. IMU, encoder, actuator-feedback, and localization paths currently carry their declared
timing in the observation contract without a general delayed-delivery queue. World truth remains inside physics and
scenario ownership.

Authored scenario maps carry 2.5D terrain patches and surface materials, while `SimulatedPlatform` turns that
content into 3D collision geometry and contact state. The deployable [Lost Signal Robot Map](robot-map.md) is a
separate provider-neutral contract: occupancy, elevation, uncertainty, surface classes, vector semantics, and
topology are composable layers. A snapshot requires spatial grid data or semantic content, but a large vector/HD map
does not allocate a mandatory global raster. A bounded static view is combined with ray-cleared sensor observations,
tracking, prediction, footprint, and inflation to produce the rolling local costmap. The scenario adapter, the
ROBOT-owned `OnlineSlamMapBuilder`, a future survey conversion, or an HD-map bridge can publish the same immutable
revision. The editor exports the
checksummed `.lsmap` package as well as its complete scenario source. Actors, rendering materials, scripts, and
physics truth never enter the Robot Map.

Static scene geometry follows one entity contract. `MapDefinition::world_objects` owns a stable ID, prefab transform,
one or more local collision boxes, active-level filter, and material. The same resolved transforms drive the renderer,
MuJoCo bodies, planar and multi-line LiDAR ray hits, material reflectivity, oriented occupancy rasterization, and the
continuous collision monitor. A scene instance outside that contract must declare `role: visual_only`; it is suitable
for paint, warning strips, and presentation cues and cannot masquerade as a wall, railing, tree, or crate. This mirrors
the visual/collision separation in SDFormat while preventing independently placed visual and physical copies.

The current collision proxies are authored boxes rather than triangle meshes, cylinders, or convex decomposition.
They are deliberately inspectable and deterministic, but must be checked against each visible asset. The repository
now ships a bounded `OnlineSlamMapBuilder` for small mapless sessions. It consumes only LSRI LiDAR and the estimated
wheel/IMU prior, publishes immutable occupancy revisions through `IMapBuilder`, and keeps its local origin in the
ROBOT map frame. It is a deterministic submap frontend. The separate `PoseGraphBackend` stores selected keyframes,
odometry/scan-match edges and validated loop closures, optimizes SE(2) poses with a fixed anchor and sparse Gauss--Newton,
and returns corrected poses for the builder to rerasterize. It does not mutate a published map or leak Eigen/solver types
into LSRI, Planner or map APIs. Dynamic filtering, calibrated ground modeling, multi-view elevation fusion and long-run
loop-closed reconstruction remain explicit validation work. Point-cloud returns may publish a basic height/elevation layer,
while planar scans leave that layer absent. The authored LSRM map remains a separate reference/provider output rather than
hidden SLAM input.

The shipped planar and 16-line LiDAR profiles are mounted in named 3D frames. In the
production path they raycast the 3D physics world against terrain, static world objects, legacy authored boxes, and dynamic actors, include
scan-time robot motion, and synthesize intensity from hit normal, body class, and distance before applying the
declared bias, noise, quantization, dropout, and delivery-latency model. Static world objects resolve reflectivity from
their surface material. The values remain normalized teaching responses rather than calibrated wavelength-dependent
material optics.

### Algorithms

`src/robot/autonomy/` follows the data flow used by compact autonomous systems: localization, mapping, perception, behavior, global and local planning, validation, then control. DWA and chassis-specific rollout samplers live under local planning; path following and PID live under control. The stack uses SI units and explicit configuration.

`PerceptionPipeline` is the observation boundary for planar LiDAR. It consumes timestamped raw and static-reference
scans after LSRI delivery, then owns projection and deskew, foreground extraction, transient ray-cleared occupancy,
anonymous tracking, and prediction state. Neither mission code nor simulated hardware classifies obstacles. A replay
or hardware source can feed the same scan contract without changing mapping or planning.

Planar returns are projected through `map → base_link → lidar_link` with the estimator's roll, pitch, yaw and the
calibrated sensor mount. The local map retains LSRM elevation, so a beam that intersects mapped ground is rejected as
ground rather than promoted to an obstacle merely because the robot is pitched on a slope. Neighboring reference
azimuths are searched within the localization uncertainty envelope instead of requiring an exact same-ray match.
The ROBOT `KnownMapLocalizer` additionally provides a bounded deterministic occupancy scan-to-map correction around
the wheel/IMU prior, with explicit overlap, timestamp and degeneracy rejection and a covariance-bearing pose result.
`GlobalMapLocalizer` and `StartupLocalization` provide cold start: when a known map exists, an unknown initial pose is
searched against the complete bounded map and rejected matches remain `RELOCALIZING`; no simulator absolute pose is
substituted. With no map, the same startup contract delegates to `OnlineSlamMapBuilder` and publishes a local-origin
pose. `LocalizationSupervisor` schedules local correction by motion/idle cadence and estimator uncertainty; global
matching is a rate-limited recovery action only. It requests bounded map views for local matching and complete maps
only for unknown-start or lost-pose recovery. The estimator publishes sigma, match evidence, innovation/age and a
separate localization confidence to behavior and planning. Full deskew, calibrated innovation gates and normal-level
deployment evidence remain the P2 gate. RobotExecutive consumes localization health for task-level responses; it does
not calculate a chassis speed or brake torque. The local planner maps the response and quality into chassis-specific
velocity windows from the localization health it already consumes, and the controller tracks those windows under the independent safety validator.

A future camera provider follows the same boundary: LSRI carries timestamped image or depth observations with frame,
calibration and exposure timing; ROBOT perception may publish class probability, extent, orientation and optional
intent cues. Those semantics may enrich an anonymous LiDAR track, but collision safety continues to consume observed
geometry when a camera is absent, stale or uncertain. Simulator actor IDs and authored class labels never enter the
ROBOT observation stream.

Global planners share a canonical `RoutePoint` result rather than a geometry-only polyline. Position, nominal body heading,
and arrival direction remain one value from planning through control. Grid A* still performs an eight-connected 2D
search and emits forward segments; its heading is the geometric route tangent and its final point inherits the
incoming edge heading. Hybrid A* remains the Ackermann default and preserves its searched heading and gear, including
at its collision-checked terminal state inside the requested goal tolerance. Multi-waypoint composition continues
from that actual terminal pose, so it cannot create an unchecked gap to an authored waypoint. Ackermann lookahead
stops at a forward/reverse cusp instead of inferring gear from point geometry.

A route lookahead point and a terminal pose are separate local-planning inputs. A pass-through route point may be
crossed at speed. A terminal pose is accepted only from estimated state: position must be inside its tolerance,
planar speed and yaw rate must be below their stop tolerances, and an authored heading must be inside its yaw
tolerance. The holonomic sampler adds signed fine forward/lateral corrections, a zero-motion candidate, and an
explicit terminal-heading critic; it does not keep rewarding cruise speed near the destination. A point-and-go
click has no authored yaw, so its default terminal heading is the final global-route heading. Scenario code observes
the result and advances presentation state; it never writes body velocity to zero or declares arrival from forward
speed alone.

The source layout is also a dependency rule. Mission state and simulated actor behavior cannot enter the autonomy
layer. Route following, waypoint progress, heading PID state, crossing behavior, prediction, and trajectory
validation all live in `src/robot/autonomy`; simulation mission code only supplies task intent and world semantics.
The `deployable_boundary` test scans all of `src/robot`, rejects every `sim/` include, rejects SIM sources and SIM
libraries from every `lost_signal_robot_*` target, and checks for the robot-only build gate.
`lost_signal_robot_autonomy`, `lost_signal_robot_executive`, `lost_signal_robot_runtime`, and
`lost_signal_robot_map_io` remain separate deployable targets. `lost_signal_robot_stack` aggregates only deployable
targets; `lost_signal_sim_hardware` depends on it and adds virtual vehicles, sensors, geometry, and physics.

Each local planner produces one `ExecutableTrajectory`: an explicit command space, the body command, and strictly
time-ordered pose, velocity, yaw-rate, and steering samples. The runtime validates the exact selected samples
against the inflated map, terrain feasibility, prediction coverage, and swept same-time dynamic clearance before
publishing the command. For RO-1 it also follows the candidate through the declared decision/actuator response
delay and validates a tire-limited emergency-braking tail. Once a conflict-zone transit is committed, a nominal
trajectory that still satisfies full static and dynamic clearance may continue when braking would strand the body
inside the conflict; this is reported separately as `CommittedTransit` and never waives nominal collision
validation. The selected chassis then converts that command into wheel speed and steering targets.

The prediction horizon never enlarges the command that may be issued now. Differential, Ackermann and holonomic
samplers restrict the published velocity to what the next decision interval can reach. If a route transition reduces
a cap below the measured speed, the planner publishes the nearest rate-limited braking command and stages the stop
over subsequent decisions. Differential candidates additionally satisfy the acceleration limit in left/right wheel
space, preventing simultaneous braking and yaw changes from asking either wheel for an unreachable speed step. The
rollout horizon includes both translational and rotational braking time, so a fast spin cannot be scored from a view
that ends before the chassis can reduce yaw rate. Candidate generation removes physical overlap; the independent
validator owns uncertainty margins and may name a physically clear, separation-increasing maneuver as a
`MinimumRiskEscape`. The rollout starts from measured motion and predicts the consequence of the next command; it
does not send the rollout's terminal velocity directly to the controller.
If validation finds no executable trajectory, the safety state also respects the platform contract: it stages the
stop from measured motion instead of publishing an impossible instantaneous zero. Differential and tracked
platforms perform that transition in left/right wheel space. The longitudinal deceleration ceiling is derived from
the dynamically loaded front/rear support bounds, combined load COM, mapped grade, and tire-limited brake ability;
MuJoCo remains responsible for the resulting contact, slip, pose, and stopping distance.
Motor controllers compare targets with encoder feedback. LSRI supports two explicit placements: a smart platform may
accept `BodyMotionCommand` and own its low-level loops, while a ROBOT controller may publish typed per-channel
actuator commands for a passive simulated plant or native driver. Command placement is a declared capability, never
an implicit reinterpretation of the same number. The default RO-1 MuJoCo composition uses the second placement:
physical joint position/velocity travels from SIM to the ROBOT wheel controller through LSRI, and normalized left/
right duty travels back through LSRI. SIM owns only the motor curve, joint effort, contact, and rigid-body plant.

For Ackermann platforms, the command is an equivalent front-wheel steering angle from the local planner through
LSRI to the steering position loop; no layer reinterprets it as yaw rate. Physical motor type, native drive command
mode, controller placement, and mechanical load are separate platform concerns. The
[platform-fidelity design](platform-fidelity.md) defines how a future native or HIL provider declares those
capabilities without exposing PWM or device protocols to autonomy.

### RobotExecutive and external commands

The deployable upper layer is a `RobotExecutive` service, not a game system. Its external API accepts generic requests such as
`navigate`, `inspect`, `collect`, `return`, `pause`, `resume` and `cancel`, together with map-frame goals, constraints,
priority and a request ID. The API does not contain a level ID, story event, actor ID, scripted route, renderer state or
simulator time. A real operator, a remote application and a simulation adapter all submit the same request type.

The brain coordinates these stages:

1. validate and queue the external request;
2. resolve named places through the ROBOT-owned `IMapProvider` when a request uses a semantic place;
3. select the active behavior and route goal;
4. provide the current intent and constraints to localization, perception, global planning, local planning, safety
   validation and control;
5. publish lifecycle, health, completion and failure evidence to the caller.

It may request a safe stop or cancel a plan, but the safety supervisor and LSRI watchdog retain final authority over
whether a command reaches an actuator. Localization, mapping, perception, PNC and control remain replaceable
providers below the brain; the brain does not branch on whether a provider is simulated, replayed or native.

`ScenarioSession` and game systems are adapters. They translate authored objectives and player choices into the
generic request, provide a SIM LSRI provider and display the brain's read-only status. They cannot call a planner,
write a stop pose, classify an actor, or inject a simulated absolute pose. In a real deployment these adapters are
omitted and a native/operator API submits the same requests over the ROBOT service boundary.

### Mission systems

The word *mission* has two distinct meanings and the codebase must not merge them:

- a **Mission Request** is deployable operator intent such as navigate, inspect, collect, return, cancel, pause, or
 resume. A **Task Executive** validates that request, expands it into ordered goals, tracks lifecycle and outcomes,
 and supplies intent to autonomy. It never reads simulator truth and never emits wheel or steering commands;
 planning, safety and control remain in autonomy;
- a **Scenario Definition / Game Session** is simulation-only content: world actors, faults, scripted events,
 scoring, story progression and presentation. It may observe mission progress, but it cannot choose a stop pose,
 rewrite a trajectory, or grant a collision exception.

The target deployable set includes the implemented `lost_signal_robot_executive` library above `lost_signal_robot_autonomy`,
with operator/API adapters above it and LSRI platform adapters below autonomy. Its first `TaskExecutive` slice owns
validated goal ordering, arrival, dwell, pause, resume, cancel, failure, and completion lifecycle. The implemented
**Lost Signal Map Interface** consists of `IMapProvider`, immutable `MapSnapshot` revisions and bounded `MapView`
queries. `MapManager` is its in-process lifecycle implementation and semantic-place resolver, while
`lost_signal_robot_map_io` loads and saves the checksummed LSRM v1 package. Simulation compilation, SLAM, survey/HD
map import and remote map services are provider adapters behind this ROBOT-owned boundary. These targets depend only
on deployable contracts.

`GoalSystem` is a thin adapter from authored game actions to `TaskExecutive` and resolves positions through
`IMapProvider`; it no longer owns generic task lifecycle or reads map markers directly. The simulation composition
root now lives in `src/sim/game/scenario/` as `ScenarioSession`, and its CMake target is explicitly
`lost_signal_sim_game`. Authored game content and scenario adapters remain under `src/sim/game/mission/`; both directories
are simulation-only and are rejected from deployable dependencies.

`src/sim/game/mission/` owns authored goals, progression, field dispatch, world activity, scoring, and presentation data.
`src/sim/game/scenario/` owns fixed-step composition, telemetry, and replay. `ScenarioSession` composes the focused systems
and publishes an immutable snapshot for consumers.
`CrossingMapSemantics` in `robot/autonomy/behavior` decodes deployable `conflict_zone` and `conflict_approach` features
from the current Lost Signal Map Interface snapshot. Scenario content may author those map features, but SIM/game does
not own a traffic-decision adapter or crossing state. `CrossingBehaviorSystem` in `robot/autonomy/behavior` owns
the per-run crossing state, derives stop and exit poses from the current route and vehicle envelope, evaluates
tracked-object predictions and current LiDAR free-space evidence, and controls the hold/commit/clear transitions.
Map semantics may declare traffic-approach centerlines, corridor width and maximum participant speed. A current scan
must cover the complete approach before it can supplement a missing track; physical returns terminate visibility,
and participant radius plus scan age reduce the remaining arrival-time bound. Duplicate sensor timestamps cannot
advance the confirmation count. Content never supplies a parking
coordinate: the stop is a continuous pose derived from route and conflict geometry rather than a selected grid
cell. Actor truth and physics objects never cross the autonomy boundary. `TrajectoryTrackingSystem` owns the last
accepted local trajectory between decision ticks and reports position, heading, and speed errors against its
interpolated expected state. Tracking is currently evidence only; it does not silently modify control authority.
Dynamic-object perception publishes timestamped constant-velocity track estimates rather than raw LiDAR cluster
centroids. Prediction may retain two motion modes while a shuttle pauses at a learned endpoint. Behavior consumes
that conservative view. Motion regression uses measurement-time samples so a smooth position filter cannot turn an
endpoint dwell into false outbound motion. An empty or stale prediction is explicit unknown evidence; it can release
a controlled crossing only when an independent, current and complete free-space observation proves every declared
traffic approach clear for the executable interval. A separate physical-collision validator retains final veto
authority.

Task orchestration is moving toward a small data-driven Behavior Graph, while continuous commitments use focused
deterministic state machines. The project does not embed a general-purpose behavior-tree runtime. The detailed
contract and the incremental decomposition of the current composition root are recorded in
[Behavior architecture](behavior-architecture.md).

Conflict-zone content carries `shadow` or `control` authority. Shadow evaluation can compare a route-level
hypothesis without changing commands. Control authority supplies a stop, hold, or commit constraint. The
route-level crossing schedule is admission evidence, while the local planner independently validates the exact DWA
trajectory at every decision tick. Admission never acts as a collision waiver and cannot lower static clearance,
dynamic clearance, prediction coverage, or braking requirements for an uncommitted motion.

The field operator, autonomous robot, dynamic actors, weather, and events remain part of one simulation world. Expedition mode reuses the same sensing, planning, control, and physics path as authored missions.

`DecisionEvidence` is the shared explanation boundary. The mission coordinator assembles it only from the
estimator timestamp, anonymous tracker references, prediction horizon, measured and projected separation,
active constraint, selected command, and planner outcome. A new record opens when the constraint, reason,
or action changes; the preceding record closes as `superseded` and names the next sequence. Mission
termination closes the final record. The current record is copied into every immutable replay snapshot,
while the compact transition history is emitted by headless runs. Player prose is a locale mapping over
stable codes, so the field view, AI view, debrief, replay, and tests cannot invent separate explanations.

### Physics

MuJoCo 3.16.0 is the single production physics engine. There is no runtime backend selector, compatibility layer, or
second production world. The complete decision, native model/data design, limits, and completed atomic cutover gate
are defined in [Physics engine decision and migration](physics-engine-migration.md).

The simulation model compiler maps content geometry, native 2.5D terrain, complete robot assemblies, actors,
contacts, and sensor sites into one immutable MuJoCo model. A simulation instance owns its corresponding mutable
state and is the single physics writer. Focused analytic kinematics remain test oracles, never a second production
world. The rendering layer never advances physics.

`sim::platform::SimulatedPlatform` in `src/sim/game/platform/` is the in-process simulation-platform adapter; mission code only
schedules it and combines its published result with goals and autonomy diagnostics. Wheel motors, encoders, IMU and LiDAR device behavior belong
to `src/sim/hardware/vehicle/` and `src/sim/hardware/sensors/`; rigid bodies, joints, contacts and raycasts belong to
`src/sim/hardware/physics/`; binding authored devices to physics joints and links belongs to `src/sim/game/platform/`. No wheel,
caster or sensor model belongs in a renderer.

The deployable set is `lost_signal_robot_stack` plus one or more native platform providers. Its targets do not
include or link any `src/sim` source or `lost_signal_sim_*` library. Neutral observation and command
messages live under `robot/runtime/`; simulator truth remains under `sim/`. A real deployment therefore replaces the
LSRI provider and platform adapter and omits all game, world, MuJoCo, and rendering targets.

`scenario_autonomy.cpp` is the current simulation composition adapter: it translates loaded task/configuration data
into neutral autonomy inputs and copies diagnostics into the game snapshot. It is not an algorithm provider. A
native product uses the `lost_signal_robot` composition root, which links only `lost_signal_robot_stack` and owns the
LSRI lifecycle in a standalone process. It currently provides the provider/startup gate; native hardware registration
and the remaining full autonomy-cycle extraction are separate deployment work. SIM and ROBOT need not use IPC: both
composition roots may call the same typed in-process interfaces, while an optional transport adapter can be added at
the provider boundary later.

Robot geometry has three independent meanings:

- the circular planning footprint is an orientation-independent conservative envelope for grid and local planners;
- the rigid hull is authored collision geometry compiled into the simulation model;
- the support polygon comes from wheel, track, and caster contact mounts and governs stability geometry.

These values are assembled at runtime rather than selected by robot ID. Robot/component JSON produces a
`ChassisCaps` record containing command space, support bounds, steering geometry, speed/rate limits and provider
capabilities. Global and local planner registries select implementations from that record. Runtime payload updates
change mass and combined COM without changing the planner API. Replacing the chassis during an active physics
session is not supported; selecting a different chassis or component loadout before a run is data driven.

Robot schema v4 forbids deriving one of these from another. In particular, a support triangle is not a hull and a
planning circle is not a vehicle contact shape. This follows the link semantics used by
[URDF](https://docs.ros.org/en/rolling/Tutorials/Intermediate/URDF/Adding-Physical-and-Collision-Properties-to-a-URDF-Model.html)
and [SDFormat](http://sdformat.org/spec?ver=1.12&elem=link): visual, collision, and inertial properties have separate
representations and origins. Lost Signal keeps a smaller native format rather than importing either middleware
format into the runtime.

RO-1 uses an elevated chassis, two powered wheel links, and a passive caster fork and wheel. RO-2 uses bounded
Ackermann knuckles, four rolling wheels, rear drive, and an independent service brake. RO-3 distributes two track
commands over six driven road wheels, RO-4 uses four independently steered and driven modules, and RO-5 combines
four driven wheels with a load-limited mecanum roller-contact provider. The single production physics engine solves every articulated assembly;
encoder truth comes from physical joint state and the motor loops apply joint effort. Each platform has its own
physical acceptance gate. Optional suspension or identified tire, track, and roller models remain platform-provider
concerns and do not change planner commands.

### Presentation

`src/sim/game/client/` renders the field and AI views from the same immutable, timestamped presentation snapshot. The
snapshot combines platform state (body/link poses, contacts and device telemetry), mission state (goals, actors and
environment), and autonomy diagnostics (routes, candidates and decisions). A hardware provider alone cannot
publish the complete game view because goals and explanatory overlays are not hardware state.

In-process code passes a `PresentationFrameView` with read-only access for the duration of one main-loop frame. A
future threaded or multi-process provider may materialize and deliver
the same value through a bounded queue or shared memory. This is subscription semantics without requiring a
middleware today. The renderer cannot step the runtime and never reads MuJoCo bodies, motor objects or sensor
internals; it also never derives physical wheel or caster motion. UI state may pause or submit player intent, but
changing camera or visualization layers cannot affect autonomous output. The headless executable consumes the same
`src/sim/game/` results and reports English, machine-readable output.

The update order follows the read/write split used by mature simulators:

```text
command snapshot
 -> hardware controller/device commands
 -> simulation-platform binding
 -> one authoritative physics step
 -> platform snapshot + ideal sensor truth
 -> hardware sensor/encoder models
 -> game/scenario autonomy presentation snapshot
 -> zero or more read-only consumers (3D, AI view, headless, replay)
```

[Gazebo's system lifecycle](https://gazebosim.org/api/sim/9/createsystemplugins.html) applies commands before the
physics update and gives sensors, graphics and rendering read-only post-update access. CARLA similarly sends a
timestamped [`WorldSnapshot`](https://carla.readthedocs.io/en/latest/python_api/#carla.WorldSnapshot) to clients on
each tick. Lost Signal uses this ownership pattern while keeping its smaller native LSRI and snapshot contracts.

### Experiment evidence

Run Artifact v3 records build and source identity, a SHA-256 content manifest, comparable run inputs, hardware and
algorithm IDs, constructed-world identity and traversal evidence, metrics, events, and the structured decision chain. `--reproduce` restores the declared authored
mission configuration, while `tools/compare_runs.py` rejects undeclared input changes. This compact manifest is
separate from UI snapshot replay and LSRI observation replay; large sensor traces remain external references.

## Stable contracts

The most important extension boundaries are:

- `ContentCatalog` for validated, localized data;
- the authored simulation-model contract and its MuJoCo compiler bindings for bodies, joints, contacts, sites, and rays;
- `IGlobalPlanner` for global planning;
- `IHighLevelPolicy` for behavior selection;
- `ChassisModel` plus `ChassisCaps` for feasible motion and actuator telemetry;
- LSRI v2 ports, sensor packets, transform frames, lifecycle, and watchdog semantics;
- `DecisionEvidence` for observation-to-action explanations and deterministic decision transitions;
- immutable snapshots for rendering, replay, and external observation.

The [autonomy extension guide](extending-autonomy.md) documents the global-planner v1 request/result contract, registry, directory boundaries, and a complete provider example.

These are useful sim-to-real seams because a hardware adapter can replace actuation and observation while keeping high-level logic testable. The seams do not eliminate the need for calibration, timing, safety, and hardware evidence.

## Coordinate and time model

Frames follow the familiar `map → odom → base_footprint → base_link → sensor/wheel link` structure. This aligns with [ROS REP-105](https://www.ros.org/reps/rep-0105.html). Geometry, time, angles, velocity, and acceleration use SI units and conventions consistent with [ROS REP-103](https://www.ros.org/reps/rep-0103.html).

`odom → base_footprint` is the continuous dead-reckoned motion and is never reset by a global correction.
Localization updates change `map → odom`; the composed state in `map` is the corrected map pose consumed by planning.
Headless artifacts report raw odometry drift and map-pose error separately. Safety scoring uses the estimate that
actually drives navigation, while the raw metric remains available for wheel-slip and odometry lessons. No simulator
absolute-pose truth is a ROBOT observation.

`base_footprint` is the terrain projection; `base_link` is the physical root-body origin. Robot `mounts` are always
calibrated in `base_link`. Physics owns centre-of-mass motion, then the shared transform tree evaluates each sensor
origin. In particular, the simulated IMU lives in `src/sim/hardware/sensors/`: it receives ideal six-axis truth at `imu_link`
and owns bias, random walk, white noise, timestamps, latency, and sequence numbers. Mission code only schedules the
device and publishes its observation.

The ROBOT side owns one versioned `RobotDescription/TransformTree` per physical configuration. It contains the rooted
link tree, sensor mounts, calibration and frame conventions (the same kind of structure commonly expressed by URDF or
SDFormat), but it is loaded as configuration rather than embedded in every LSRI packet. LSRI carries only the
measurement, acquisition time, source and `frame_id`; a transform service resolves that frame into `base_link`,
`odom` or `map` using the active description version. SIM and a native hardware provider implement the same seam, so
replacing the provider does not change localization or perception code. A calibration change creates a new version and
invalidates dependent estimates; it is never silently mixed into an existing session.

The simulation advances at a fixed step. Sensors have independent sample rates and declared latency. LiDAR uses a
bounded deterministic delivery queue; the other current observation paths publish at their scheduler tick and retain
timing metadata for downstream evidence. A real-time or transport provider may apply the same contracts to every
device. Determinism requires the same content, configuration, seed, backend, and build assumptions; cross-backend
numeric identity is not promised.

## Scheduling and concurrency

The current simulator uses one fixed-step, single-threaded scheduler. This is deliberate: identical inputs produce an inspectable execution order, and tests can advance simulated time without depending on host CPU timing. A 20 ms tick does not run every algorithm once. Each subsystem has its own cadence or trigger:

| Work | Current trigger |
|---|---|
| Dynamic world and physics | every 20 ms tick (50 Hz) |
| Encoder and IMU simulation, state-estimator prediction | every 20 ms tick (50 Hz) |
| LiDAR simulation, LiDAR perception, track update | configured LiDAR `scan_hz` |
| High-level policy, local planning, and control | compute component `decision_hz` |
| Global planning | mission start, goal change, or expedition-route change |
| Replay sample | every 40 ms |

Within one due tick, the scheduler updates dynamic actors, captures and processes a due LiDAR scan, runs a due autonomy decision, applies the held command to the chassis and physics, then generates encoder and IMU observations for the resulting motion. A command is therefore held between decision events. Global planning is event driven rather than part of the frame loop.

The in-process LSRI provider publishes and reads through a mutex-protected bounded queue on that same thread. The provider also accepts concurrent platform producers, so later sensor I/O threads do not require a new algorithm API. Sensor packets still carry independent sample times, sequence numbers, frames, and latency metadata; independent metadata does not imply one operating-system thread per sensor.

LSRI does not require this execution model. A hardware provider or out-of-process bridge may receive LiDAR, IMU, and encoder data concurrently and place validated packets in the bounded observation queue. Autonomy must consume timestamped observations and publish expiring commands without sharing mutable mission or simulator state.

If parallel execution is added, the deterministic scheduler remains the reference model. Parallel workers should operate on immutable, timestamped input snapshots and commit outputs at explicit scheduler boundaries. Queue limits, deadlines, stale-data policy, and command expiry belong in the runtime contract; thread ownership stays in the provider or executor. This keeps algorithms portable between deterministic simulation and asynchronous hardware without making their interfaces depend on a particular thread model.

Three execution modes can therefore share the same module contracts:

| Mode | Execution | Transport | Intended use |
|---|---|---|---|
| Deterministic | one fixed-step thread | in-process LSRI | tests, replay, teaching, reproducible experiments |
| Concurrent | sensor I/O threads plus an autonomy worker pool | in-process bounded mailboxes | real-time desktop simulation and embedded computers |
| Distributed | supervised platform, autonomy, and presentation processes | an IPC, shared-memory, or network LSRI provider | hardware deployment and fault isolation |

The concurrent mode should not create one permanent thread for every algorithm. Sensor drivers are naturally asynchronous; estimation and perception can run when observations arrive; global planning belongs on a cancellable low-frequency worker; and control needs a periodic deadline with the newest valid state snapshot. A small executor can map those logical tasks to a bounded worker pool.

The distributed mode needs more than a socket. Before it is considered implemented, LSRI needs a canonical wire encoding, schema compatibility tests, clock-domain conversion, bounded transport queues, peer health and restart behavior, and end-to-end watchdog tests. High-bandwidth scans may use a shared-memory ring while commands use a small reliable channel, but both must preserve the same message semantics.

To move algorithms safely between these modes, each module should own its mutable state and exchange immutable messages. Planned extension contracts are `ISensorSource`, `IEstimator`, `IPerceptionStage`, `ILocalPlanner`, and `IController`, following the existing `IGlobalPlanner` pattern only when a second real implementation justifies the boundary. Each task declaration will name its trigger, input age limit, deadline, queue policy, and failure fallback. Physics remains single-writer authority and publishes timestamped world snapshots to simulated sensor jobs; algorithm workers never read mutable physics or mission state directly.

## Content and algorithm integrity

A new data-driven component may tune declared physical behavior, cost, visuals, and unlock rules. It cannot inject code. A new chassis type or algorithm requires a C++ implementation, registration, build integration, and tests.

Validation checks references, budgets, schema shape, presentation coverage, locale completeness, placeholder preservation, canonical generation, and important gameplay contracts. Content generation scripts are the source of truth where a generated asset has a corresponding generator.

## Current strengths

The architecture already provides several uncommon combinations in a small open project:

- autonomous decisions and wheel-level control in one inspectable loop;
- five motion families behind explicit capability contracts;
- shared physics and sensor state across game, AI visualization, replay, and headless tests;
- observation noise, sampling, frames, and latency separated from world truth;
- deterministic scenario seeds and machine-readable results;
- data-driven educational content with string-only localization overlays;
- a clean path for log comparison, native hardware providers, and measured HIL work.

The main technical risk is concentration in `ScenarioSession`, which still coordinates many subsystems. Recent
extraction of planning, control, goal, dispatch, operator, world activity, dynamic traffic, and trajectory tracking
reduces that pressure. Sensor scheduling, decision evidence, mission progress, and snapshot assembly remain the
next ownership boundaries.
