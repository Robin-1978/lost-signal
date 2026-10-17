# RO-1 reference platform

## Decision

RO-1 is the correctness reference for Lost Signal. New chassis families, sensors, gameplay mechanics, and
sim-to-real claims wait until this platform passes its physical, sensor, timing, and determinism gates. The goal is
one small system whose meanings are unambiguous, not a large catalog of approximate models.

The fidelity sequence has two distinct kinds of compliance. Finite wheel-ground contact stiffness and damping are
part of the baseline from the first rigid multibody model because infinite-rigidity contact is neither physical nor
numerically useful. Suspension travel, bushings, flexible mounts, tire carcass models, and structural flex are
optional higher-fidelity assemblies. RO-1 must first close its baseline gate. When another chassis becomes a product
priority, it receives its own rigid articulated assembly, support/contact model, and independent acceptance suite;
RO-1 parameters are not copied into it. Optional structural compliance is introduced only for a named experiment
with observable acceptance evidence. Product work can continue between those chassis gates, so this rule does not
require implementing all chassis before content work resumes.

This does not make ROS 2 a runtime dependency. Lost Signal keeps its native content and LSRI contracts. Established
robot description and simulator designs are used as semantic references, and an external bridge may translate them
later.

## Industry evidence

- [URDF physical and collision properties](https://docs.ros.org/en/rolling/Tutorials/Intermediate/URDF/Adding-Physical-and-Collision-Properties-to-a-URDF-Model.html)
 places visual, collision, and inertial data beside one another on a link. It explicitly allows collision geometry
 to be simpler than visual geometry and gives collision and inertia their own origins.
- [SDFormat link](http://sdformat.org/spec?ver=1.12&elem=link),
 [collision](http://sdformat.org/spec?ver=1.12&elem=collision), and
 [joint](http://sdformat.org/spec?ver=1.12&elem=joint) separate physical links, one or more collision shapes,
 inertia, visuals, sensors, and typed parent/child joints.
- [Gazebo DiffDrive](https://gazebosim.org/api/sim/9/classgz_1_1sim_1_1systems_1_1DiffDrive.html) addresses left and
 right wheel joints. Wheel geometry and contact belong to the model; the controller commands joints and publishes
 feedback rather than moving the chassis pose directly.
- Bullet's `btMultiBody` represents an articulated base and links. Its wheel vehicle API independently names a
 chassis connection point, wheel direction, axle, radius, suspension travel, stiffness, compression/relaxation
 damping, friction slip, and maximum suspension force. `btRaycastVehicle` is useful evidence for those physical
 roles, but its car-oriented suspension model is not the reference implementation for a two-wheel differential
 base with a freely swivelling caster.
- [REP-103](https://www.ros.org/reps/rep-0103.html) and
 [REP-105](https://www.ros.org/reps/rep-0105.html) provide the coordinate, unit, and frame conventions already used
 by LSRI. They are conventions, not a ROS 2 dependency.
- [SDFormat LiDAR](https://sdformat.org/spec/1.12/sensor/#lidar) separates horizontal/vertical samples and
 resolution, range limits and resolution, update rate, and measurement noise. Lost Signal adds explicit acquisition
 timestamps and material-return semantics to that minimum.
- [Gazebo physics parameters](https://classic.gazebosim.org/tutorials?tut=physics_params) treats contact `kp` and
 `kd` as stiffness- and damping-equivalent coefficients and documents ERP/CFM as the equivalent soft-constraint
 form. Its example uses compliance to stabilize intermittent rigid contact rather than assuming infinite stiffness
 is automatically more accurate.
- [MuJoCo solver parameters](https://mujoco.readthedocs.io/en/stable/modeling.html#solver-parameters) exposes
 impedance, stiffness, and damping per soft constraint. [Drake hydroelastic contact](https://drake.mit.edu/doxygen_cxx/group__hydroelastic__user__guide.html)
 likewise represents nominally rigid bodies with a slightly deformable layer and a distributed pressure patch.
- [Project Chrono tire models](https://api.projectchrono.org/wheeled_tire.html) deliberately separates rigid,
 semi-empirical handling, and finite-element tires. Its contact-material examples name normal and tangential
 stiffness and damping independently. Lost Signal follows the same fidelity-tier principle while keeping the RO-1
 reference small.

## Canonical RO-1 assembly

The reference model is a kinematic tree:

```text
base_footprint
  -> base_link (chassis collision + inertial body)
      -> left_wheel_link  (continuous axle joint, powered)
      -> right_wheel_link (continuous axle joint, powered)
      -> caster_fork_link (continuous vertical swivel joint, passive)
          -> caster_wheel_link (continuous axle joint, passive)
      -> imu_link (fixed)
      -> lidar_link (fixed)
```

The two drive-wheel contacts and caster contact form the support triangle. The chassis collision body is elevated
above the ground and never acts as a skid plate during nominal driving. The caster wheel axle trails its swivel axis;
fork yaw and wheel roll are separate state variables. Visual meshes attach to links but never define collision or
inertia implicitly.

The planning footprint, collision shapes, inertial properties, support contacts, and safety clearance are separate
data. Each value has one owner and one unit. A loader rejects a planning footprint that does not contain the rigid
hull projection or an assembly whose centre of mass lies outside its support polygon at rest.

## Contact compliance and parameter evidence

The rigid links remain rigid bodies. A wheel-ground contact is compliant because real rubber, wheel mounts, ground
surfaces, bearings, and structures deform even when the visible meshes do not. The dynamic reference begins with a
linearized normal law around its operating load:

```text
F_normal = max(0, stiffness * compression + damping * compression_rate)
```

This is an effective contact-pair model, not a claim that rubber is globally linear. The friction limit consumes the
resulting normal force; longitudinal and lateral tire forces therefore change with payload, slope, and load transfer.
An actual suspension is a separate spring/damper joint and is authored only for hardware that has one.
The stock RO-1 caster is therefore rigidly mounted at its swivel joint. Tire/contact compliance, caster trail,
fork and wheel inertia, and Bullet's joint/contact solve provide its vertical response. A future sprung caster must
declare suspension travel, axis, stiffness, damping, preload, and limits in robot data; it must not be added merely
to hide penetration, an unstable support polygon, or unsuitable solver parameters.

In the RO-1 Bullet provider, this law is not evaluated as a 50 Hz force in `SimulatedPlatform`. Bullet converts the
effective stiffness and damping to its compliant normal constraint and solves normal load, non-penetration, joint
constraints, and friction together in four 5 ms internal substeps. `BulletTireContact` projects each wheel's physical
axle into the contact plane and supplies a stable axle/rolling tangent basis. It refreshes cached manifold material
values when surface contact changes; changing only the rigid body's friction value would leave an existing contact on
its old material. The positional friction anchor then resists low-speed slip without an outer-loop tire force.

Content expresses longitudinal rolling resistance as dimensionless `Crr`. Bullet's torsional limit is a torque-per-
normal-load length, so the provider uses `Crr * wheel_radius` about the axle. The articulated wheel separately declares
`rolling_support_moment_arm_m`, which represents finite-patch resistance about the tire's non-rolling axes and stops a
point-contact cylinder from behaving like a barrel. Spinning resistance is converted by wheel radius as well. These
values configure anisotropic Bullet constraints; they do not replace the two Coulomb tangent constraints.

Static tire friction is not a parking brake. With both drive joints freely backdrivable, the passive caster can turn
the vehicle toward the fall line and let it roll downhill without tire slip. The reference cross-slope hold test uses
encoder feedback plus a zero-speed pose-hold command, matching the powered robot's closed loop. A coast-mode test must
expect downhill motion unless an identified gearbox drag or mechanical brake is explicitly authored.

Dynamic-profile values are selected from a declared static deflection and damping ratio. For example, 100 N on a
40,000 N/m contact gives 2.5 mm deflection. For an effective supported mass `m`, a target damping ratio gives
`damping = 2 * ratio * sqrt(stiffness * m)`. An identified profile must instead record incremental load/deflection
measurements and a release, drop, or step-response trace. It fits on a calibration set and reports chassis-height,
rebound, IMU-peak, and contact-load error on held-out runs.

Wheel and ground compliance combine as a pair. For the linearized case, two normal compliances in series use
`1 / effective_stiffness = 1 / wheel_stiffness + 1 / ground_stiffness`; damping is fitted for the pair rather than
blindly added. A future Hertz or Hunt-Crossley profile may replace the linear law without changing the platform
contract.

Steel wheel on steel deck is a useful named experiment, but it is not infinite-stiffness contact. Hertz deformation,
bearing clearance, wheel-carrier flex, roughness, and deck seams remain. Its profile uses much higher stiffness and
lower damping than the rubber reference and may require a smaller internal contact step. The lesson compares rubber
on concrete, steel on dry steel, and steel on wet steel using normal-load balance, IMU shock, slip, stopping distance,
and rolling loss. Solver chatter is never presented as physical vibration.

`btRaycastVehicle` remains a possible coarse Ackermann provider: its ray wheel supplies suspension travel, spring,
damping, engine, brake, and tire-slip roles around one chassis body. It is not the RO-1 reference because its wheels
are ray/suspension elements rather than collision bodies, and it cannot naturally represent the caster's independent
vertical swivel joint, axle roll joint, step contact, and encoder state. A later high-fidelity tire provider may use
Chrono or another validated contact model behind the same platform contract; it must earn that role with measured
normal-load, slip, and trajectory error rather than by replacing Bullet with an unvalidated outer-loop spring.

`base_footprint` is the terrain projection of the root link. `base_link` is the physical root-body origin, which is
the RO-1 chassis centre of mass in the reference profile. Every entry in `mounts` is expressed in `base_link`; link
initial poses are expressed in the model/`base_footprint` datum, while joint pivots and axes are expressed in their
named parent and child link frames. Renderers, sensors, and physics must resolve these values through the shared
`FrameTransformTree`; none may infer a frame from the planning collision height.

## Execution boundary

The correct closed loop is:

```text
body command
 -> differential inverse kinematics
 -> left/right wheel-speed targets
 -> PID from joint encoder feedback
 -> voltage/PWM or torque effort with saturation
 -> wheel joints and wheel-ground contact
 -> articulated rigid-body state
 -> encoders, IMU, and LiDAR observations
 -> state estimation and autonomy
```

No layer may integrate a second hidden wheel plant. In Bullet authority mode, joint positions and velocities are the
source of encoder truth and the articulated base is the source of pose and IMU truth. The deterministic planar
backend may use analytic differential kinematics, but it must publish the same observation and actuator contracts.

The IMU is an independent hardware-layer device. Physics supplies ideal angular velocity and specific force at
`imu_link`; the device owns sample sequence, sample/delivery timing, three-axis fixed bias, bias random walk, and
white noise. For an IMU offset `r` from the chassis centre of mass, the ideal acceleration is
`a_com + alpha × r + omega × (omega × r)`, then gravity is removed and the vector is rotated into `imu_link`.
RO-1 currently mounts the IMU at the chassis centre of mass, so `r = 0`, but the contract and implementation retain
the complete lever-arm transform.

## Single-line LiDAR reference

The first sensor is a rotating single-plane scanner mounted at `lidar_link`. A scan is a sequence of rays, not an
instantaneous circle. Every return records acquisition time, angle, range, intensity, hit status, and source frame.
The reference profile defines:

- scan rate, sample count, angular limits and direction;
- minimum/maximum range and range resolution;
- per-ray pose during the scan, including chassis motion;
- first-surface occlusion in the same 3D collision world used by physics;
- deterministic range bias, random noise, quantization, dropout, and out-of-range behavior;
- material reflectivity, incidence-angle response, and distance attenuation for intensity;
- seeded replay with sample time preserved independently of delivery time.

Weather attenuation and multi-return behavior remain later named profiles. They are not silently enabled in the
reference sensor.

Ground materials own normalized LiDAR reflectivity independently of friction, rolling loss, and contact compliance.
The sensor profile supplies fallback terrain, obstacle, and actor returns plus the incidence and distance response.
This separation lets a steel deck be mechanically slippery yet optically strong without a code branch for its name.

## Acceptance gates

### Assembly and contact

- Only the two drive wheels and rear caster touch a flat floor at rest; chassis clearance stays positive.
- The sum of support-contact normal load agrees with the analytic gravity component within a declared tolerance;
 every reference support retains non-zero time-averaged load on a planar surface.
- Contact stiffness and damping identify their source (`dynamic assumption` or measured profile), units, operating
 load, and validation envelope. Infinite stiffness is not an acceptance shortcut.
- Wheel radius, axle separation, axle offset, caster trail, link poses, mass, centre of mass, and inertia are data and
 are present in run identity.
- The stationary model settles without visible penetration; over 100 fixed 20 ms steps, chassis height changes by
 less than 15 mm per step and roll/pitch by less than 0.025 rad per step.
- A deterministic 15--30 mm rough-strip fixture excites the IMU while the chassis and caster fork remain clear,
 contact support persists, pitch and roll stay bounded, and two fresh worlds produce identical peak response and
 final pose. This is a plant fixture and never changes mission scoring.
- Wheel and caster render transforms come from joint state. No wall-clock animation or chassis-specific renderer
 branch is allowed.
- A passive caster has no control or visual steering algorithm. Its swivel and roll emerge from authored trail,
 mass, inertia, joint constraints, friction and contact; the platform snapshot publishes the resulting link poses.

### Differential motion and actuators

- Equal signed wheel speeds produce straight motion; equal opposite speeds rotate around the driven axle; unequal
 speeds match the analytic instantaneous-centre-of-curvature solution within a declared tolerance.
- Forward and inverse kinematics are checked by independent geometric or finite-difference oracles.
- Encoder counts come from joint angle and preserve sign through forward, reverse, and direction changes.
- In the default Bullet composition, named joint position/velocity feedback crosses LSRI to the deployable ROBOT
 wheel controller. Its normalized left/right duty returns through typed LSRI actuator commands before the SIM
 motor curve produces joint effort. No simulator state enters the controller through a side channel.
- Physical longitudinal slip compares each joint's `radius * angular_velocity` with the rigid-body velocity at that
 wheel contact. The reported ratio is `abs(tread - contact) / max(abs(tread), abs(contact), 0.1 m/s)` and requires
 both 0.08 m/s absolute mismatch and ratio above 0.25 for three steps; release takes five clean steps. This avoids
 an unstable near-zero ratio and one-step contact chatter.
- Saturation, deadband, acceleration, braking, low friction, and payload mass have measurable effects and conserve
 units from command to effort.
- Swappable motor torque is limited first by that motor's speed/torque curve and then by a separately named
 joint/transmission ceiling; the stock motor value is never reused as the articulation ceiling.
- With no joint effort and no authored brake, the backdrivable platform coasts downhill. A zero-speed wheel loop and
 an optional pose-hold controller are separate plugins above the plant. Hold succeeds only while motor torque,
 tire-ground friction, supply voltage, and any declared continuous/thermal envelope can support the grade.
- A level with known dead-reckoning uncertainty declares `planning.localization_margin_m`; global obstacle inflation
 adds it to the physical envelope and chassis margin. Score tolerances never silently double as planner parameters.

### Dynamics maneuver matrix

`test_ro1_dynamics_acceptance` is the focused plant gate. Its assertions use static equilibrium, kinematic,
force/energy, friction-envelope, and inertial identities that are separate from mission scoring and planner code.
The public integration step is 20 ms with Bullet contact substeps; a 10 ms comparison checks numerical convergence
rather than requiring bit identity across different step sizes.

| Maneuver | Independent acceptance evidence | Current status |
|---|---|---|
| Flat-floor initialization and rest | `sum(N) = m*g`; force and pitch-moment balance predict drive/caster loads from the authored assembly centre of mass; the first public step has bounded height, speed and IMU transients; residual pose and velocity remain bounded for six seconds | executable; longer solver/energy-drift sweeps remain open |
| Ground clearance and rough strip | only wheel/caster supports touch; a deterministic 15--30 mm strip excites IMU truth without base/fork strikes, contact loss, penetration growth, or unbounded roll/pitch | executable within the authored roughness envelope |
| Zero-speed peak-torque launch | the first 300 ms remains below the smaller of motor-force and friction acceleration bounds; symmetric motion, three-point support, chassis height, and pitch remain bounded | executable; short-duration stall torque only |
| Straight-line step-size convergence | identical one-second shaft effort at 20 ms and 10 ms produces bounded pose, speed, height, and attitude differences | executable |
| Steady cruise stability | after the speed loop settles at 1.0 m/s, physical pitch peak-to-peak stays below 0.002 rad, chassis-height ripple below 1 mm, and speed ripple below 0.02 m/s; these plant values distinguish contact instability from display cadence | executable; current measured ripple is about 0.00022 rad, 0.06 mm, and 0.002 m/s |
| Maximum-speed straight line | the closed wheel-speed loop reaches the declared speed envelope with bounded lateral/yaw drift, roll and support loss; this exposes caster shimmy and left/right bias that a circular maneuver can hide | executable inside the 2.4 m/s profile |
| In-place turn | opposite closed-loop wheel targets preserve wheel signs, rotate around the driven axle within drift tolerance, retain all supports, and do not vertically launch the chassis | executable |
| Longitudinal and lateral adhesion | low longitudinal friction produces less body travel, more tread travel and observed slip; the tangent resultant stays inside Bullet's two-axis friction-pyramid bound `sqrt(2)*mu*N`; a short sub-tip cross-slope probe distinguishes high-grip hold from low-grip lateral slide | executable for uniform surfaces; split-friction and identified combined-slip behavior remain open |
| Emergency braking | measured stop distance cannot beat the motor/traction energy bound; the rear caster unloads under braking and the hull remains clear | executable; no thermal or ABS claim |
| High-speed steady circle | after transients, IMU lateral acceleration agrees with `v*yaw_rate`; encoder-derived and body yaw rates agree within the declared slip tolerance; all supports remain loaded | executable inside the tested RO-1 speed/curvature envelope |
| Longitudinal grade boundary | high-grip probes immediately below and above the analytic motor-torque boundary respectively climb and roll back; default-grip probes bracket the independently derived driven-wheel normal-load/adhesion boundary; zero effort backdrives downhill | executable around peak-torque and uniform-surface traction boundaries; continuous/thermal gradeability remains unclaimed |
| Cross-slope/tip boundary | the triangular support polygon gives the analytic static limit; a high-grip under/over-limit fixture checks uphill-wheel unloading and tip onset independently of sliding; 20-degree coast and closed-loop hold remain load cases | executable boundary; post-tip recovery is outside the current controller contract |
| Curb, drop, and wheel lift | a separate 30 mm vertical curb/deck/drop fixture checks traversal, transient support, base/fork clearance, IMU peak, attitude, landing height, and per-step vertical motion | executable at 30 mm; larger-obstacle capability remains unclaimed |
| Payload sweep | one authored slope payload and dynamic payload mass/inertia updates are covered | partial: systematic mass, longitudinal/vertical centre-of-mass, braking, and tip sweeps remain open |

These rows are plant tests, not level-completion tests. A level can fail because of perception, planning, control or
content and still use a correct plant; it can also pass while compensating for an incorrect plant. Mission regression
therefore starts only after the relevant plant rows pass and never supplies the expected physics values.

The next physics claims remain deliberately open instead of being inferred from the rows above:

- Per-wheel ground material lookup, split-friction braking/launch, and left/right surface transitions. The present
  `GroundContact` input is uniform for the complete platform during a step, so it cannot produce valid split-surface
  evidence.
- Combined braking-and-cornering saturation against an identified friction circle or ellipse. Bullet's two-direction
  friction pyramid is bounded and tested, but it is not an identified tire model.
- Dynamic rollover under transient steering, curb impact, or payload shift. The current tests cover static geometric
  tip onset and one supported steady circle only.
- Systematic payload mass and longitudinal/vertical centre-of-mass sweeps. The current payload adapter changes mass,
  centre height and yaw inertia, but it does not yet author a longitudinal payload location or collision body.
- Long-duration resting-energy drift and solver sweeps across iteration count, contact stiffness/damping and public
  time step. The existing 10/20 ms comparison closes only the named one-second straight maneuver.

The steady-circle structure follows the purpose of
[ISO 4138 steady-state circular driving](https://www.iso.org/obp/ui/#iso:std:iso:4138:ed-4:v1:en): expose the
relation between steady speed, curvature, yaw response, and lateral acceleration. Lost Signal applies the invariant to a differential robot rather than
claiming that the complete passenger-car procedure applies.
[ISO 18646-1 locomotion tests for wheeled service robots](https://www.iso.org/obp/ui/#iso:std:iso:18646:-1:ed-1:v1:en)
motivate keeping translational speed, rotational speed and stopping behavior as separately measured capabilities;
the project adopts the decomposition without claiming certification to the full standard. Bullet's solver contract and the
[Gazebo physics-parameter guide](https://classic.gazebosim.org/tutorials?tut=physics_params) motivate the separate
fixed-step, solver, contact-compliance, friction-direction, and penetration-stability checks. The
[Project Chrono vehicle overview](https://api.projectchrono.org/vehicle_overview.html) likewise separates chassis,
suspension, steering, driveline, tire, terrain, and powertrain subsystems; the matrix keeps those claims separable
instead of treating one successful mission as whole-vehicle validation. NIST's
[standard response-robot test methods](https://www.nist.gov/el/intelligent-systems-division-73500/standard-test-methods-response-robots)
use repeatable apparatus, controlled variables, quantitative metrics and progressively harder mobility tasks. The
RO-1 fixtures use that same decomposition: one physical question per apparatus and an explicit tested boundary.

### Sensor and frames

- `map -> odom -> base_footprint -> base_link -> {wheel, caster, imu, lidar}` is connected and time-consistent.
- Analytic wall, corner, thin-pole, occlusion, moving-platform, min/max-range, intensity, and dropout fixtures pass.
- Network or queue arrival time is never substituted for acquisition time.

### System evidence

- Same build, scenario, seed, fixed step, and input trace produce identical result and critical trajectories.
- Headless and graphical clients consume the same snapshots and do not change physics.
- Rendering consumes a composed read-only presentation snapshot. It never accesses Bullet, motor/controller
 instances or mutable sensor state, and no physical result depends on whether a frame was rendered.
- The client renders between the previous and current 50 Hz authoritative snapshots using the remaining fixed-step
 accumulator. Body pose, actor pose, wheel/caster angles, and articulated link transforms are interpolated only in
 that owned presentation snapshot. This removes fixed-step judder on high-refresh displays without filtering IMU,
 encoder, contact, replay, headless, or autonomy data.
- No sim-to-real claim is made until a measured RO-1 profile passes held-out logs or hardware tests for wheel speed,
 pose, yaw, stopping distance, scan timing, range, and intensity error envelopes.

## Acceptance matrix

This table is the release source of truth. “Executable” means the contract and a deterministic independent check
exist. RO-1 releases at the sourced `typical` level; measurements in the last column are required only for an
optional future `identified` or calibrated sim-to-real claim.

| Area | Typical reference status | Additional evidence for an identified claim |
|---|---|---|
| Assembly and frames | **Executable**: one rooted link tree; coincident joint pivots; aligned axes; wheel/caster mounts match physical rest transforms; `base_link` height and complete-assembly centre of mass are separate | measured dimensions and centre of mass |
| Mass and inertia | **Executable**: link mass sum, physical inertia inequalities, parallel-axis assembly yaw inertia, and motor/gearbox armature are validated at load | pendulum/CAD or identified inertial measurements |
| Three-point support | **Executable**: two drive wheels and the rolling caster form the support polygon; base and fork remain clear; the time-averaged flat-ground normal-force sum agrees with weight and every support carries positive load | measured static corner loads and long-duration wear/compliance data |
| Tire/contact | **Executable model, assumed parameters**: Bullet owns normal load, compliant contact, anisotropic friction, rolling loss and observed slip | force/slip and load/deflection measurements for each tire/surface pair |
| Drive and encoder | **Executable model, assumed parameters**: wheel target → quantized joint encoder → PID → PWM → torque curve → armature → Bullet joint; coast, braking, reverse and torque-limited slope cases are checked | voltage/current, deadband, gearbox drag, backdrive, continuous/peak and thermal measurements |
| IMU | **Executable model, assumed parameters**: truth is evaluated at `imu_link`, includes gravity and rigid-body lever-arm terms, and receives bias/random-walk/white noise once | mounting calibration, scale/misalignment, temperature and Allan-variance data |
| Planar LiDAR | **Executable typical profile**: sourced 0.2--12 m, 800-ray, 10 Hz spinning profile; 3D Bullet first-hit geometry; explicit invalid near/far returns; fixed counter-clockwise phase; acquisition-end timestamp and per-ray motion; seeded bias/noise/quantization/dropout; occlusion and analytic intensity checks | measured material reflectivity, distance/incidence error curves, latency/jitter and packet behavior |
| Payload | **Partial**: added mass, centre height and ring-equivalent inertia update the plant without double-counting the unloaded assembly | authored payload collision geometry, attachment compliance and measured load placement |
| Determinism | **Executable for focused fixtures**: fresh worlds reproduce platform, encoder and IMU results | long-duration cross-platform envelope and final release/ASan run |
| Mission outcomes | **Frozen while the typical platform gate is open** | re-baseline only after the executable model rows above pass; hardware measurements are not required |

RO-2 through RO-5 are non-reference teaching proxies. Their current mission results do not validate their physical
models and are not included in RO-1 release evidence.

## Implementation sequence

1. **Implemented:** schema v4 separates planning footprint, visual mounts, and an optional articulated link/joint/
 actuator assembly. The generic physics interface contains no RO-1 or caster-specific joint type.
2. **Implemented for RO-1:** an elevated base, powered continuous wheel joints, passive swivel and rolling caster
 joints, 35 mm caster trail, LSRI physical-joint feedback, deployable wheel PID, LSRI normalized-duty output,
 PWM-to-wheel-torque plant response, and deterministic Bullet contact substeps. Other chassis deliberately remain
 non-reference profiles.
3. **Implemented:** `base_footprint`, physical `base_link`, wheel/caster links, and sensor mounts now share one frame
 contract; 3D parts use `base_link` mounts and wheel/caster animation reads Bullet joint position. The virtual IMU is
 owned by `src/sim/hardware/sensors/` and consumes transformed six-axis truth rather than planar command differences.
4. **Implemented for the dynamic gate:** straight/spin/arc, reverse, braking, low-friction slip truth, contact
clearance, closed-loop 20-degree cross-slope pose hold, and a symmetric 20-degree uphill torque probe have executable
checks. The loader now rejects disconnected joint trees, separated pivots, misaligned axes, invalid inertia tensors,
mass totals, yaw inertia totals, mount/link transform disagreement, and a centre of mass outside the support polygon.
Drive-wheel rigid inertia and motor/gearbox armature inertia are separate data. Payload collision geometry and an
identified-hardware envelope remain open.
5. **Implemented:** the single-line LiDAR reference gate fixes the sourced profile, angular/time convention, 3D
 first-return and occlusion behavior, near/far invalid semantics, analytic intensity response, and deterministic
 bias/noise/quantization/dropout fixtures. Beam divergence, weather and multi-return remain separate optional profiles.
6. Freeze mission scoring and outcomes while the platform gate is open. Flat, slope, cross-slope, low-friction and
loose-ground content supplies repeatable inputs to engineering probes; mission success is not a physics oracle.
7. Re-run and re-author RO-1 mission baselines only after the physical, actuator and sensor gates pass. Run release
tests, with ASan/UBSan reserved for milestone and release-candidate gates.

Only after these gates pass does work resume on new Playgrounds, additional chassis, multi-line or solid-state
LiDAR, SLAM, VLA, HIL, or other expansion. The prioritized release and optional fidelity work is maintained in the
[RO-1 post-baseline optimization backlog](product-roadmap.md#ro-1-post-baseline-optimization-backlog); this document
remains the acceptance source of truth for the physical reference model.
