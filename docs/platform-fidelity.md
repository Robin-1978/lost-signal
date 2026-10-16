# Platform fidelity and hardware adaptation

## Decision

Lost Signal keeps autonomy commands at the chassis level and adapts them to native actuator commands inside a
platform provider. Sensor and actuator models grow through measured, named profiles rather than through one
ever-more-expensive default simulation.

This preserves a small teaching and regression runtime while allowing a calibrated robot profile to replace the
simulated platform. It also prevents a device detail such as PWM duty cycle from leaking into planning code.

The design has three fidelity levels:

| Profile | Purpose | Required behavior |
|---|---|---|
| `ideal` | algorithm teaching and focused tests | geometry and timing with deliberately minimal error |
| `dynamic` | shipped game and physical comparisons | current contact, actuator response, scan timing, and seeded observation errors |
| `identified` | one measured hardware target | parameters fitted from logs, with a stated calibration source and validation envelope |

`identified` is not a claim that every unmeasured effect is realistic. It names the robot, sensor, firmware,
calibration date, test conditions, and held-out error bounds.

## Two stable boundaries

LSRI carries intent between autonomy and a platform:

```text
autonomy
  -> forward speed + yaw rate / steering angle / holonomic twist
  -> LSRI validation, expiry, and watchdog
  -> platform adapter
  -> native actuator controllers
  -> motor, steering mechanism, tires, and body
```

The platform adapter owns kinematics, controller placement, device protocols, and feedback conversion. The
autonomy stack therefore does not change when a simulated PWM plant is replaced by a drive that accepts velocity
or torque commands.

Sensor observations travel in the other direction. Simulation and hardware publish the same timestamps, frames,
sequences, units, and measurement fields. A hardware measurement is never passed through the simulation noise
model a second time.

## Current implementation

### Single-plane LiDAR

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
- no beam divergence, minimum-range blind zone, intensity saturation, temperature drift, or transport jitter;
- the shipped component data currently sets bias, quantization, and base dropout to zero even though the model supports those fields.

The next improvement is a calibrated planar-LiDAR profile, not an immediate switch to a large 3D point cloud. In
priority order it should add:

1. minimum and maximum valid range, angular coverage, scan direction, phase, and per-ray acquisition timing;
2. measured range resolution, fixed bias, and distance-dependent variance;
3. material and incidence-dependent return strength and missed-return probability;
4. delivery latency, jitter, dropped packets, and timestamp/clock behavior at the provider boundary;
5. optional beam divergence, saturation, and multi-return behavior when a real target or lesson needs them.

Every stochastic term remains seeded in deterministic mode. Datasheet values provide initial bounds; recorded scans
against known targets provide identified values.

The transport-neutral sensor contract already reserves
`PlanarSpinning`, `MultiBeamSpinning`, and `SolidState` patterns. Its point return contains XYZ, intensity,
per-point time offset, ring, and return index. A future multi-line scanner adds calibrated vertical beam angles and
ring timing. A future solid-state scanner supplies its own non-rotating pattern and exposure schedule. Both reuse the
same physics raycast, frame tree, return model, and LSRI observation rules.

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
a tunable dynamic-profile value, not an identified motor, gearbox, or chassis limit. An `identified` platform must
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

1. Keep the current `dynamic` profile as the deterministic product baseline.
2. Preserve steering-angle semantics from the Ackermann planner through LSRI to the steering position loop.
3. Introduce a planar-LiDAR profile only with measurable fields and deterministic validation cases.
4. Add the actuator capability record when a native driver, HIL drive, or second actuator backend can exercise it.
5. Create the first `identified` profile from one low-speed robot and one planar LiDAR, then publish held-out errors.
6. Add multi-line or solid-state providers only for a concrete learning scenario or target device.

This order keeps unused abstraction out of the runtime while preserving the contracts needed for later providers.

## Engineering references

- [SDFormat sensor specification](https://sdformat.org/spec/1.12/sensor/) separates scan samples, angular resolution, range limits, resolution, and noise configuration.
- [Ouster sensor data documentation](https://static.ouster.dev/sensor-docs/image_route1/image_route2/sensor_data/sensor-data.html) illustrates per-measurement timestamps, frame and measurement IDs, range quantization, calibrated reflectivity, and multiple return profiles in a production multi-line sensor.
- [ros2_control hardware interface types](https://control.ros.org/rolling/doc/ros2_control/hardware_interface/doc/hardware_interface_types_userdoc.html) provide a useful external example of named command and state interfaces with limits. LSRI remains independent and uses a bridge when ROS 2 integration is desired.
- [ROS REP-103](https://www.ros.org/reps/rep-0103.html) supplies the SI-unit and coordinate conventions shared by simulation and hardware adapters.

These references inform the contracts. Only measurements from a target platform validate a profile.
