# Sim-to-real: current capability and evidence plan

## Assessment

Lost Signal is **not yet a demonstrated sim-to-real system**. It now has a versioned, middleware-neutral [runtime interface](runtime-interface.md) plus SI units, explicit frames, timestamped observations, chassis capability contracts, actuator telemetry, deterministic replay, and headless comparison. No target robot has completed a calibrated closed-loop mission with simulator-to-hardware error measurements, so transfer remains an engineering hypothesis.

That distinction matters. A realistic renderer or a MuJoCo vehicle does not prove transfer. Evidence begins when the same command and observation contracts run against real hardware and recorded logs can be compared with the simulator.

## What can transfer

| Layer | Current state | Transfer outlook |
|---|---|---|
| Task intent and behavior | deployable Task Executive owns goal ordering, arrival, dwell, pause/resume/cancel and outcomes | external operator/fleet API and a native lifecycle executable remain; game sim/scenario/progression is not deployed |
| Map management | LSRM v1, immutable revisions, semantic places, provider lifecycle, authored-map compiler, and checksummed package I/O are implemented | SLAM, survey, HD-map and remote providers still need concrete bridges and field evidence |
| Global planning | A* and Hybrid A* consume maps and vehicle constraints | reusable when maps, footprint, and costs match the robot |
| Local planning | trajectory evaluation uses obstacles, dynamics, and chassis limits | reusable only after latency, footprint, braking, and prediction calibration |
| Kinematics | differential, tracked, Ackermann, swerve, and mecanum mappings are executable | reusable with measured geometry and sign conventions |
| Motor control | targets, encoder feedback, PID, and PWM-like authority are modeled | gains and feedforward must be identified on each platform |
| Estimation | timestamped encoder, IMU, and localization inputs have stable boundaries | algorithms may transfer; noise and covariance must be measured |
| Sensors | rates, latency, bias, noise, quantization, dropouts, and extrinsics are represented | parameters require bag/log fitting and environmental tests |
| Physics | MuJoCo models contact, slope, payload, and rigid bodies | useful for stress cases; tire/track/terrain contact requires identification |
| Presentation and scoring | consume snapshots and telemetry | directly reusable for operator tools and education |

## Deployment boundary

A native robot build links `lost_signal_robot_stack`, then supplies LSRI platform and map providers. The executable
robot-only build gate configures with `LOST_SIGNAL_BUILD_SIM=OFF`; it does not fetch MuJoCo or compile anything under
`src/sim`. Runtime observation and command contracts are defined outside the simulator and contain
timestamps, frames, measurements, and estimated state only. Simulation truth, authored actors, game goals, scoring,
and rendering snapshots cannot appear in an autonomy interface.

Providers are selected per LSRI endpoint. A hardware-in-the-loop run may bind a native LiDAR to `lidar/front` while
keeping IMU, encoders, actuators, terrain, and vehicle dynamics simulated. The inverse arrangement can feed virtual
sensors to the deployable controller and command a real motor bench. Every binding declares its source, frame,
clock domain, rate, backend, and role; duplicate command ownership is rejected.

The Task Executive sits above autonomy and accepts validated task plans, cancels/pauses/resumes them,
orders semantic goals, and reports progress. It cannot command actuators directly or bypass trajectory and safety
validation. Scenario actors, score thresholds, scripted surprises and story state belong to a separate
simulation-only Scenario/Game Session layer under `src/sim/game/`.

The [Lost Signal Robot Map](robot-map.md) is likewise source-neutral. An editor, SLAM backend, survey compiler, or HD-map
bridge publishes validated revisions through `IMapProvider`; planning and the Task Executive never import the
source format. The persisted LSRM package is a navigation product and does not pretend to preserve a SLAM pose graph,
raw point cloud, Lanelet2 map, or simulation world. A global raster is optional. Map queries and perception derive a
bounded rolling view; dynamic obstacles and inflated costs remain runtime layers.

Conflict-zone behavior follows the same rule. A map may provide a semantic conflict region, while autonomy derives
the stop pose from the current route, footprint, localization margin, actuator delay, and braking capability. A
mission file cannot prescribe a dynamic-world parking coordinate or grant permanent permission to cross.

The simulator currently assembles these neutral inputs in `src/sim/game/scenario/scenario_autonomy.cpp`. The
deployable upper-policy transaction is implemented by
`src/robot/autonomy/pipeline/autonomy_coordinator.*`; the scenario supplies only its typed inputs. The remaining
composition work is to move route-progress, dynamic-interaction and local-candidate assembly behind the same
provider-neutral ROBOT boundary. The algorithm implementations, including route progress and its PID state, live in
the autonomy target; the coordinator also owns the estimator instance and route-tracking state used by that loop. The
global planner instance and localization recovery scheduler are owned there as well. The repository still needs a native
lifecycle/configuration executable before it can run on a physical platform without custom integration. The
`deployable_boundary` test prevents that remaining adapter work from being bypassed by importing game or simulator
types into the reusable targets.

## The reality gap

The largest current gaps are measurable:

- no native hardware driver or optional middleware bridge;
- no target robot URDF or calibrated mass, inertia, center of gravity, wheel radius, steering geometry, or actuator curve;
- no measured command, sensor, network, and clock latency distribution;
- no real LiDAR, IMU, or encoder logs for bias, covariance, dropout, and motion-distortion fitting;
- no tire or track force data across the represented surfaces;
- no time synchronization, emergency-stop integration, or independent safety supervisor;
- no hardware-in-the-loop or physical mission result.

The simulated contact model is intentionally compact. Surface friction and traction create meaningful educational contrasts, but they are not identified Pacejka, terramechanics, or track-soil models. Weather effects also need real measurements before engineering use.

The current LiDAR providers include a mounted single-plane scan and one 16-line spinning point cloud produced by
raycasting the 3D physics collision world. They see terrain, static bodies, and dynamic actors; represent per-ray or
per-point timing; and produce synthetic intensity from surface normal, distance, and a coarse
terrain/static/actor reflectivity class. This supports geometry, occlusion, motion distortion, projection, and
observation-pipeline tests. It is not a calibrated return model and does not use wavelength- and material-specific
optical data, measured packets, jitter, or multi-return behavior.

The staged sensor profiles, multi-line and solid-state extension boundary, Ackermann control cascade, and separation
of physical motor types from actuator command modes are specified in
[platform fidelity and hardware adaptation](platform-fidelity.md).

The articulated RO-2 path exercises the intended transfer boundary end to end: ROBOT owns the Ackermann geometry
and closed loops, LSRI carries normalized-duty and brake channels plus joint
feedback, and SIM owns motor/servo response, mechanisms, contact, and rigid-body integration. Its successful dynamics
gate is simulator evidence only. The mass, inertia, steering load, tire response, latency, and controller gains still
require identification and held-out hardware validation before any transfer claim.

RO-3, RO-4, and RO-5 use the same boundary: ROBOT owns tracked or holonomic kinematics and closed loops; LSRI owns
typed actuator commands and joint feedback; SIM owns motors, mechanisms, terrain contact, gravity, and MuJoCo time.
The mecanum roller law is explicitly a SIM contact-provider mechanism. Replacing it with measured hardware, a
higher-fidelity provider, or a replay source does not change the ROBOT controller contract.

## First hardware milestone: log and compare

The first milestone should not command an unattended robot. It should make simulator and hardware evidence comparable.

1. Choose one low-speed differential-drive platform and a controlled indoor route.
2. Measure wheel radius, track width, mass, center of gravity, encoder CPR, motor deadband, saturation, and braking response.
3. Publish or adapt `map`, `odom`, `base_footprint`, `base_link`, LiDAR, IMU, and wheel frames.
4. Record synchronized commands, encoders, IMU, LiDAR, external ground-truth pose, battery, and safety events.
5. Replay the same command profile in simulation with the measured parameters.
6. Compare pose, yaw, wheel speed, stopping distance, scan timing, and energy envelopes with confidence intervals.
7. Change parameters using a documented identification procedure, then validate on a held-out route.

Passing this milestone means the simulator predicts selected low-speed behavior within declared error bounds. It does not authorize autonomous hardware operation.

## Staged evidence gates

### Gate 1: LSRI parity

The hardware adapter implements LSRI v2 with the same units, frame semantics, timestamps, sequence rules, command limits, lifecycle, watchdog, and state fields used by simulation. Unit, sign, transform, stale-data, and conformance tests pass.

### Gate 2: component identification

Static and dynamic tests identify geometry, actuator response, sensor bias/noise, latency, and stopping behavior. Parameters come from measurements, not mission tuning.

### Gate 3: open-loop replay

Recorded command sequences stay within declared error envelopes for straight motion, turns, braking, ramps, and payload changes.

### Gate 4: supervised closed loop

The autonomy stack runs at low speed inside a protected workspace with an independent stop system. A safety operator can stop motion without relying on Lost Signal.

### Gate 5: scenario transfer

Previously unseen routes and perturbations are evaluated. Reports include failures, distributions, and negative results. Only claims supported by these held-out tests are published.

## Why the current architecture is useful

The project avoids several common dead ends. Algorithms receive sensor-like observations instead of direct actor truth. Sampling time and delivery time are distinct. Transform frames and sensor extrinsics are explicit. Chassis feasibility reaches both planning and actuation. LSRI rejects incompatible and expired commands. Headless runs expose stable JSON for comparison. Physics, observation, estimation, autonomy, and presentation have identifiable boundaries.

Those properties reduce adapter work and make discrepancies observable. They do not remove the reality gap, but they turn it into a list of measurements rather than a vague promise.

## External engineering references

The implementation and roadmap use public conventions and simulator practices as reference points:

- [ROS REP-103](https://www.ros.org/reps/rep-0103.html) standardizes SI units and coordinate conventions.
- [ROS REP-105](https://www.ros.org/reps/rep-0105.html) defines the `map`, `odom`, and `base_link` relationship used by mobile robots.
- [CARLA sensor reference](https://carla.readthedocs.io/en/latest/ref_sensors/) exposes sensor rate, bias, standard deviation, and seed parameters; Lost Signal follows the same principle of explicit observation configuration.
- [Gazebo sensor documentation](https://gazebosim.org/docs/harmonic/sensors/) provides a mature comparison point for simulated sensor systems.
- [Webots tutorials](https://cyberbotics.com/doc/guide/tutorials) demonstrate controller development across simulated robot and sensor interfaces.

These sources guide interface design. They are not validation evidence for this project's models.
