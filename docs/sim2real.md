# Sim-to-real: current capability and evidence plan

## Assessment

Lost Signal is **not yet a demonstrated sim-to-real system**. It has an architecture that can support sim-to-real work: SI units, explicit frames, timestamped observations, hardware-like sensor packets, chassis capability contracts, actuator telemetry, a physics boundary, deterministic replay, and headless comparison. No target robot has completed a calibrated closed-loop mission with simulator-to-hardware error measurements, so transfer remains an engineering hypothesis.

That distinction matters. A realistic renderer or a Bullet vehicle does not prove transfer. Evidence begins when the same command and observation contracts run against real hardware and recorded logs can be compared with the simulator.

## What can transfer

| Layer | Current state | Transfer outlook |
|---|---|---|
| Mission and behavior | high-level intent, goals, waits, and recovery are explicit | likely reusable after safety review |
| Global planning | A* and Hybrid A* consume maps and vehicle constraints | reusable when maps, footprint, and costs match the robot |
| Local planning | trajectory evaluation uses obstacles, dynamics, and chassis limits | reusable only after latency, footprint, braking, and prediction calibration |
| Kinematics | differential, tracked, Ackermann, swerve, and mecanum mappings are executable | reusable with measured geometry and sign conventions |
| Motor control | targets, encoder feedback, PID, and PWM-like authority are modeled | gains and feedforward must be identified on each platform |
| Estimation | timestamped encoder, IMU, and localization inputs have stable boundaries | algorithms may transfer; noise and covariance must be measured |
| Sensors | rates, latency, bias, noise, quantization, dropouts, and extrinsics are represented | parameters require bag/log fitting and environmental tests |
| Physics | Bullet models contact, slope, payload, and rigid bodies | useful for stress cases; tire/track/terrain contact requires identification |
| Presentation and scoring | consume snapshots and telemetry | directly reusable for operator tools and education |

## The reality gap

The largest current gaps are measurable:

- no hardware transport or ROS 2 adapter;
- no target robot URDF or calibrated mass, inertia, center of gravity, wheel radius, steering geometry, or actuator curve;
- no measured command, sensor, network, and clock latency distribution;
- no real LiDAR, IMU, or encoder logs for bias, covariance, dropout, and motion-distortion fitting;
- no tire or track force data across the represented surfaces;
- no time synchronization, emergency-stop integration, or independent safety supervisor;
- no hardware-in-the-loop or physical mission result.

The simulated contact model is intentionally compact. Surface friction and traction create meaningful educational contrasts, but they are not identified Pacejka, terramechanics, or track-soil models. Weather effects also need real measurements before engineering use.

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

### Gate 1: interface parity

The hardware adapter exposes the same units, frame semantics, timestamps, command limits, and state fields used by simulation. Unit, sign, transform, and stale-data tests pass.

### Gate 2: component identification

Static and dynamic tests identify geometry, actuator response, sensor bias/noise, latency, and stopping behavior. Parameters come from measurements, not mission tuning.

### Gate 3: open-loop replay

Recorded command sequences stay within declared error envelopes for straight motion, turns, braking, ramps, and payload changes.

### Gate 4: supervised closed loop

The autonomy stack runs at low speed inside a protected workspace with an independent stop system. A safety operator can stop motion without relying on Lost Signal.

### Gate 5: scenario transfer

Previously unseen routes and perturbations are evaluated. Reports include failures, distributions, and negative results. Only claims supported by these held-out tests are published.

## Why the current architecture is useful

The project avoids several common dead ends. Algorithms receive sensor-like observations instead of direct actor truth. Sampling time and delivery time are distinct. Transform frames and sensor extrinsics are explicit. Chassis feasibility reaches both planning and actuation. Headless runs expose stable JSON for comparison. Physics, observation, estimation, autonomy, and presentation have identifiable boundaries.

Those properties reduce adapter work and make discrepancies observable. They do not remove the reality gap, but they turn it into a list of measurements rather than a vague promise.

## External engineering references

The implementation and roadmap use public conventions and simulator practices as reference points:

- [ROS REP-103](https://www.ros.org/reps/rep-0103.html) standardizes SI units and coordinate conventions.
- [ROS REP-105](https://www.ros.org/reps/rep-0105.html) defines the `map`, `odom`, and `base_link` relationship used by mobile robots.
- [ros2_control hardware interfaces](https://control.ros.org/rolling/doc/ros2_control/hardware_interface/doc/hardware_interface_types_userdoc.html) separate command and state interfaces, a useful model for a future adapter.
- [CARLA sensor reference](https://carla.readthedocs.io/en/latest/ref_sensors/) exposes sensor rate, bias, standard deviation, and seed parameters; Lost Signal follows the same principle of explicit observation configuration.
- [Gazebo sensor documentation](https://gazebosim.org/docs/harmonic/sensors/) provides a mature comparison point for simulated sensor systems.
- [Webots tutorials](https://cyberbotics.com/doc/guide/tutorials) demonstrate controller development across simulated robot and sensor interfaces.

These sources guide interface design. They are not validation evidence for this project's models.
