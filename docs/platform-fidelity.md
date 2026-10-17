# Platform fidelity and hardware adaptation

## Decision

Lost Signal declares controller placement as a platform capability. A smart platform accepts a chassis-level
command and owns its low-level loops. A passive plant accepts typed actuator commands from a deployable ROBOT
controller. Sensor and actuator models grow through named, sourced profiles rather than through one
ever-more-expensive default simulation.

This preserves a small teaching and regression runtime while allowing a calibrated robot profile to replace the
simulated platform. It also prevents a device detail such as PWM duty cycle from leaking into planning code.

The design has three fidelity levels:

| Profile | Purpose | Required behavior |
|---|---|---|
| `ideal` | algorithm teaching and focused tests | geometry and timing with deliberately minimal error |
| `typical` | shipped game and controlled physical comparisons | public specifications, documented engineering assumptions, units, ranges, and seeded observation errors |
| `identified` | one measured hardware target | parameters fitted from logs, with a stated calibration source and validation envelope |

The RO-1 release requires the `typical` level. Lost Signal is not planning to build a physical RO-1, so hardware
identification is an optional future contribution rather than a game-release gate. The exact shipped assumptions
and public sources are recorded in [parameter profiles and sources](parameter-sources.md).

`identified` is not a claim that every unmeasured effect is realistic. It names the robot, sensor, firmware,
calibration date, test conditions, and held-out error bounds.

## Two stable boundaries

LSRI supports two explicit control paths:

```text
smart drive:
ROBOT body command -> LSRI -> embedded/provider controller -> plant

passive drive (default RO-1 Bullet composition):
platform joint feedback -> LSRI -> ROBOT wheel controller
ROBOT typed actuator command -> LSRI -> motor and mechanical plant
```

The composition root chooses one path from declared capabilities. Device protocols and feedback conversion remain
inside platform providers. Kinematics and any controller that must transfer with ROBOT remain in `src/robot`;
electrical response, mechanisms, tires, contact, and rigid-body dynamics remain in the provider or physical plant.

Sensor observations travel in the other direction. Simulation and hardware publish the same timestamps, frames,
sequences, units, and measurement fields. A hardware measurement is never passed through the simulation noise
model a second time.

Rendering is outside both directions. The simulated platform publishes a timestamped platform snapshot after its
authoritative step; mission code combines that with goals, environment and autonomy diagnostics into a presentation
snapshot. The graphical client, headless reporter and replay recorder are read-only consumers. In the current
single-process runtime, publication can be a const value handoff. Future threads or processes may transport the same
contract through a bounded queue or shared memory without changing device, physics or rendering ownership.

A GPU scene used by a future camera or depth sensor is still part of sensor simulation. Its pixels become a
timestamped hardware observation before autonomy sees them. The player's 3D camera may share meshes and rendering
libraries, but it cannot be the owner or clock of that sensor.

## Current implementation

The dynamic robot model follows the standard link separation used by URDF and SDFormat: rendered meshes do not
define contact, and the planning footprint does not double as the rigid-body shape. Robot schema v4 authors a
planning radius, visual mounts, and physical collision data independently. Its optional articulation describes
generic links, rest poses, collision shapes, mass, friction, parent/child joints, axes, pivots, and actuator-channel
bindings in data.

RO-1 is the first articulated reference platform. Its data defines an elevated base link, two powered cylindrical
wheel links, a passive caster fork with a vertical swivel joint, and a separate caster wheel with a rolling joint.
The caster has a 35 mm authored trail: the swivel axis is ahead of the wheel axle, while the wheel model origin is
the axle rather than an extra hidden visual offset. Bullet joint position and velocity are the only encoder truth.
The passive caster is absent from motor-controller telemetry: it receives no command, and its visual transform is
copied from physical link state through the platform/presentation snapshot.
The other four chassis still use the compact rigid proxy and are outside the current reference gate.

RO-1 runs a deterministic 20 ms public physics step with four 5 ms Bullet contact substeps. This is a numerical
integration choice, not a second clock domain. It prevents small-wheel contact and chained caster constraints from
injecting the visible pitch impulse produced by one coarse contact solve. Only the two powered wheels and caster
wheel may touch a flat floor during nominal motion; the base and fork have positive clearance.

RO-1 currently uses one compact brushed-DC-style drive profile:

```text
wheel-speed target
 -> PID from quantized physical joint position
 -> signed PWM duty
 -> stall-torque / no-load-speed curve
 -> gearbox-equivalent wheel-joint torque
 -> Bullet contact
```

PWM is the native electronic command and torque is its mechanical output. Bullet owns tire friction, rolling
resistance, slip, and body integration; the motor controller no longer applies a second traction plant. The current
catalog values are typical teaching parameters, not an identified voltage, current, winding, gearbox-efficiency,
deadband, battery-sag, or thermal model.

The wheel-speed PID and encoder quantization are in the deployable ROBOT control target. Physical wheel-joint
position and velocity cross LSRI measurement schema 2; normalized left/right duty crosses the typed actuator
command path. The SIM plant does not own a second wheel-speed loop. Its body-motion entry remains available as the
explicit smart-drive/analytic mode.

The wheel link owns the inertia of its physical wheel geometry. Each powered joint separately declares motor and
gearbox inertia reflected to that axis (`armature_inertia_kgm2`). The Bullet adapter represents the armature as a
coaxial axisymmetric inertial component on the constrained wheel link because Bullet hinges do not expose a native
armature term. This preserves a valid inertia tensor and the intended wheel-speed transient without pretending the
extra inertia is tire mass or friction. The value remains a typical-profile assumption until a coast or torque-step
test identifies it on hardware.

For RO-1, Bullet solves compliant normal contact and two tangent friction constraints in the same 5 ms internal
substeps as the wheel and caster joints. `BulletTireContact` derives the rolling and axle tangent directions from the
physical wheel pose and contact normal; it also refreshes material values already cached in persistent manifolds when
the active surface changes. A static-friction anchor resists slip while the pose is held. It is not a parking brake: an
unbraked differential platform with a passive caster can yaw toward the fall line and roll downhill without violating
static friction. The 20-degree hold gate therefore exercises the encoder, zero-speed motor loop, and heading/position
hold together with tire contact.

Slope behavior is tested in three distinct modes: coast applies zero joint effort and must roll; closed-loop pose
hold uses encoder feedback and may hold only inside its authority; a known sub-gravity shaft torque on the 20-degree
grade must still roll backward. This keeps controller success, motor authority, and tire adhesion as separate
evidence instead of hiding all three in one friction coefficient.

Catalog `rolling_resistance` is dimensionless `Crr`; Bullet's torsional contact limit has torque/load length units.
The adapter converts rolling resistance about the wheel axle to `Crr * wheel_radius`. Articulated wheel content also
declares `rolling_support_moment_arm_m`, a finite-patch support value about the tire's non-rolling axes. Bullet's
rolling anisotropy keeps these two effects separate enough for the compact reference model. `btRaycastVehicle` is
reserved for a possible coarse Ackermann provider and is not used for the collision-wheel, two-joint passive caster.

The motor profile owns available shaft torque (`stall_force_N * wheel_radius_m`). The articulation actuator limit is
a separate joint/transmission safety ceiling and must be at least as large as every supported motor profile; it must
not silently clamp an equipped torque motor to the stock motor's stall torque. RO-1's current 10 N·m joint ceiling
admits its 9 N·m torque profile while each motor model still enforces its own lower curve. The dynamic torque profile
provides 180 N total stall force: a 39 kg slope lesson needs about 131 N for gravity at 20 degrees and about 5 N for
nominal rolling resistance, leaving limited authority for acceleration and differential steering. These are declared
teaching parameters rather than identified hardware values.

In articulated mode, contact saturation is no longer inferred from the analytic motor-force clamp. The platform
compares physical wheel tread speed with rigid-body contact-point speed, applies low-speed and persistence guards,
and publishes the resulting model-side slip state and maximum ratio in the deterministic run evidence.

`ITireContact` is a semantic adapter, not an outer tire-force solver. The current `BulletTireContact` maps common
friction, compliance, wheel-axis, rolling, and spinning semantics into the Bullet body and its internal constraints.
A future `ChronoTireContact` must live inside a complete Chrono platform provider that owns bodies, joints, contact,
and time integration; mixing Chrono tire forces into a Bullet world would create two coupled plants.

The shipped grip tire uses a declared typical-profile friction coefficient of 1.2. This represents a high-grip
teaching tire on the dry slope surface, not a measured material pair; the ordinary tire remains at 0.7. The value
must be replaced by contact-pair measurements before an identified-profile claim.

Normal contact compliance follows the same evidence rule. The current RO-1 wheel values are typical teaching
assumptions chosen to produce millimetre-scale static deflection and a documented damping ratio. They are not
identified rubber properties. Wheel-component and ground-material data will own their respective compliance;
`SimulatedPlatform` resolves one effective pair for the physics backend. Bullet-specific ERP, CFM, or raw rolling
constraint coefficients do not cross that content boundary.

### LiDAR providers

The production LiDAR is already more than a 2D grid query. A named `lidar_link` emits each ray into the Bullet 3D
collision world. Terrain, static geometry, and moving actors participate in occlusion. Each ray receives its own
sample time over the scan period, so vehicle motion distorts a scan; an estimated-motion path produces the deskewed
static comparison. The return model includes first-hit range, a normalized synthetic intensity, fixed bias, white
noise, range quantization, seeded dropout, timestamp, source, sequence, and frame.

Its current limits are explicit:

- one horizontal scan plane;
- a single first return;
- coarse terrain/static/actor reflectivity instead of material and wavelength calibration;
- constant Gaussian range noise and constant dropout probability;
- no beam divergence, intensity saturation, temperature drift, or transport jitter;
- the shipped standard profile uses a 0.2 m invalid near field and 5 mm quantization, while fixed bias and base
 dropout remain zero-valued typical assumptions rather than calibrated device measurements.

The executable typical gate covers the first item below and the deterministic forms of the second and third. An
optional future identified planar-LiDAR profile should add measured evidence in this order:

1. minimum and maximum valid range, angular coverage, scan direction, phase, and per-ray acquisition timing;
2. measured range resolution, fixed bias, and distance-dependent variance;
3. material and incidence-dependent return strength and missed-return probability;
4. delivery latency, jitter, dropped packets, and timestamp/clock behavior at the provider boundary;
5. optional beam divergence, saturation, and multi-return behavior when a real target or lesson needs them.

Every stochastic term remains seeded in deterministic mode. Datasheet values provide initial bounds; recorded scans
against known targets provide identified values.

The 16-line `MultiBeamSpinning` provider uses one rotating three-dimensional scan rather than sixteen unrelated
planar scans. Its sixteen authored beam angles preserve an interleaved firing order. Every return contains sensor
frame XYZ, normalized intensity, ring, return index, and a firing offset relative to the scan-end timestamp. Bullet
raycasts terrain, static geometry, and moving actors at the firing pose, so occlusion and deterministic motion
distortion are present. The static-reference cloud is generated at the estimated sensor pose rather than the truth
pose.

The raw cloud crosses LSRI schema 3. A range-image projection groups returns by azimuth, orders them by measured
elevation, follows adjacent returns whose surface slope is inside the configured ground limit, and excludes that
ground chain before selecting the nearest obstacle return. The authored seed height, maximum ground slope, and
obstacle height interval are explicit teaching-profile values. This compact classifier supports the existing 2D
local obstacle layer; it is not a replacement for a full 3D segmentation or elevation-map pipeline.

The virtual device applies range limits, seeded bias/noise/quantization/dropout, and a deterministic 40 ms delivery
latency after raycasting. Acquisition timestamps remain unchanged while the fixed-step provider releases a scan on
the first tick at or after its availability time. Presentation keeps at most 1200 colored points per scan and replay
frames share the immutable display cloud; autonomy still receives every simulated return.

The profile is a class-level `typical` teaching configuration, not an identified VLP-16. It has one first return,
coarse reflectivity classes, no beam divergence, no weather attenuation, no packet loss or jitter, and no calibrated
material intensity. A future solid-state scanner supplies its own non-rotating pattern and exposure schedule while
reusing the frame, timing, return, and LSRI rules.

Run `python3 tools/benchmark_lidar.py` after a Release build to compare the planar and 16-line providers in the same
20-second fixed-step Bullet scenario. On the development host used for the initial gate, four repeated runs measured
about 1.50--1.59 seconds for the 800-ray planar profile and 4.05--4.11 seconds for the 5760-ray profile, or about
4.9 times real time for the 16-line run. Sharing replay clouds reduced its 20-second peak resident set from roughly
47 MB to 29 MB. These are regression reference measurements, not portable CI thresholds; contributors should report
hardware, build type, duration, median, real-time factor, and peak memory when changing the raycast path.

### Ackermann drive

The current Ackermann chain is:

```text
planner/controller: forward speed + equivalent front-wheel steering angle
  -> drive-wheel speed PID -> normalized PWM -> speed/force curve -> tire contact -> body force
  -> steering position P loop -> normalized PWM -> rate limit + first-order servo response
  -> left/right Ackermann linkage angles -> body yaw response
```

The drive loop uses quantized encoder feedback. Its PWM-like output drives a compact force-speed model, and tire
contact limits the force by available traction before Bullet advances the body. The steering loop controls angle,
then the linkage produces different inner and outer front-wheel angles. It does not yet model motor current,
steering torque, rack inertia, compliance, friction, backlash, thermal limits, or battery voltage sag.

The `standard`, `torque`, and `eco` catalog entries are currently parameter sets for the same plant. They change
maximum wheel speed, stall force, free speed, and power scale. They are not PWM, torque, velocity, and position
command modes.

The current Ackermann follower and local sampler use `0.25 m/s` as a conservative reverse policy default. This is
a tunable typical-profile value, not an identified motor, gearbox, or chassis limit. An `identified` platform must
derive forward and reverse limits from measured hardware behavior and keep the controller and sampler on the same
declared value.

## Motor type, command mode, and controller placement

These concepts remain independent:

| Concept | Examples | Owner |
|---|---|---|
| physical machine | brushed DC, BLDC/PMSM, hub motor, steering servo | identified plant profile |
| transmission and load | gear ratio, efficiency, reflected inertia, wheel or steering rack | platform assembly |
| native command mode | duty cycle, voltage, current/torque, velocity, position | drive capability |
| control loop | current, velocity, and position PID/feedforward | simulator, embedded drive, or provider |
| feedback | position, velocity, effort, current, voltage, temperature, fault | platform provider |

One BLDC motor may expose torque mode through one drive and velocity mode through another. A steering servo commonly
accepts position while closing velocity and current loops internally. Catalog motor choices may therefore describe
different physical motor/gearbox packages without changing the autonomy interface.

When a second real actuator backend is added, its provider should declare a compact capability record rather than
silently reinterpret numbers:

```text
ActuatorCapabilities
  supported command modes: duty_cycle | voltage | torque | velocity | position
  position, velocity, effort/current, voltage, and thermal limits
  update rate and command expiry
  available feedback fields
  controller location and safe-stop behavior
```

Commands are tagged by mode and carry one SI quantity. State may carry position, velocity, effort/torque, current,
voltage, temperature, fault, sample time, and sequence. Missing feedback remains unavailable rather than being
invented.

For a typical Ackermann platform, Lost Signal requests wheel velocity and steering position. A velocity-capable
drive and a position-capable steering servo receive those targets directly. A PWM-only platform enables explicit
provider-side velocity and position controllers. A torque-controlled drive enables an explicit velocity-to-torque
controller. Unsupported modes fail configuration; they are never accepted and reinterpreted as another unit.

## What makes transfer credible

More equations alone do not close the reality gap. A transferable profile needs measurements of:

- motor/drive deadband, no-load speed, force or torque curve, braking, saturation, and command latency;
- steering angle calibration, rate, delay, hysteresis, backlash, and load response;
- wheel radius, gear ratio, encoder resolution, mass, inertia, center of gravity, and payload changes;
- tire force and slip on the surfaces represented by a mission;
- LiDAR range error, dropout and intensity across distance, incidence angle, and representative materials;
- clock offset, sampling phase, delivery latency, jitter, packet loss, and restart behavior.

Parameters are fitted on one dataset and checked on held-out routes and targets. The report compares pose, yaw,
wheel speed, steering angle, stopping distance, scan timing, return rate, and energy with stated tolerances. SIL,
recorded replay, HIL, and low-speed supervised hardware runs then provide progressively stronger evidence.

The desired result is a small and replaceable platform layer plus an honest error envelope. “Seamless sim-to-real”
is not a release claim until the same autonomy build completes those hardware gates.

## Implementation order

1. Finish the RO-1 `typical` reference gates and publish its explicit validation envelope.
2. Introduce the reference planar-LiDAR profile only with measurable fields and deterministic validation cases.
3. Preserve the other chassis as non-reference teaching profiles until each receives its own articulated gate.
4. Add the actuator capability record when a native driver, HIL drive, or second actuator backend can exercise it.
5. Allow an optional future `identified` profile from a low-speed robot and planar LiDAR without blocking release.
6. Extend the implemented 16-line provider or add a solid-state provider only for a concrete learning scenario or
 target device.

The rigid-body acceptance fixture already covers a 30 mm curb and drop with partial support loss, pitch response,
IMU impact, hull clearance, and recovery to three-point support. Add a separate negative-height groove fixture to
bound wheel bridging, temporary contact loss, chassis clearance, and post-impact settling; a visible level seam is
not a substitute for this model test.

This order keeps unused abstraction out of the runtime while preserving the contracts needed for later providers.

## Engineering references

- [SDFormat sensor specification](https://sdformat.org/spec/1.12/sensor/) separates scan samples, angular resolution, range limits, resolution, and noise configuration.
- [Ouster sensor data documentation](https://static.ouster.dev/sensor-docs/image_route1/image_route2/sensor_data/sensor-data.html) illustrates per-measurement timestamps, frame and measurement IDs, range quantization, calibrated reflectivity, and multiple return profiles in a production multi-line sensor.
- [ROS REP-103](https://www.ros.org/reps/rep-0103.html) supplies the SI-unit and coordinate conventions shared by simulation and hardware adapters.

These references inform the contracts. Only measurements from a target platform validate a profile.
