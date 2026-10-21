# Native hardware integration guide

Lost Signal uses the same ROBOT autonomy stack for simulation, replay and a real machine. A real robot is integrated by
providing native LSRI endpoints and a measured robot profile. Algorithms do not read a device SDK, a serial packet, a
MuJoCo body, or a simulator truth pose.

## What changes

| Area | Simulation | Native robot |
|---|---|---|
| Platform provider | `ProviderBackend::Simulated` | `ProviderBackend::Native` driver adapter |
| Sensor source | deterministic virtual device | vendor driver, CAN, EtherCAT, serial or network receiver |
| Clock | simulator monotonic clock | hardware timestamp converted to the selected monotonic clock |
| Geometry | authored robot profile | measured calibration profile and transform tree |
| Actuation | MuJoCo plant or smart-drive simulation | passive motor driver or smart-drive controller |
| Truth pose | test-only instrumentation | never published to ROBOT |

The LSRI schema and the ROBOT localization/planning/controller interfaces stay unchanged when the hardware can express
their existing fields. A new device field is added only through a versioned LSRI schema change; it must not become a
planner-specific extension.

## Bring-up sequence

1. **Freeze the robot identity.** Copy the chassis type, wheel/steering/track geometry, mass, inertia, payload range,
   actuator channels and command mode into a versioned native robot profile. Record a calibration ID, date and hash.
2. **Build the transform tree.** Define `map → odom → base_footprint → base_link` and every sensor, wheel and joint link.
   Measure the sensor origins and axes. Do not repeat these transforms in every observation packet; LSRI packets carry the
   calibrated `frame_id`, while the ROBOT transform service owns the versioned tree.
3. **Register endpoints.** Register one observation source or command sink per device in `EndpointRegistry`. Use stable
   endpoint IDs and source IDs, `ProviderBackend::Native`, nominal rates, clock IDs and frame IDs. No two providers may own
   the same command endpoint.
4. **Implement the adapters.** Convert vendor packets to `ObservationFrame` with acquisition timestamp, sequence,
   latency, units, frame and source. Convert `MotionCommand` or `ActuatorCommand` to the device protocol and report
   rejected, late, disconnected and watchdog states through `RuntimeHealth`.
5. **Load physical limits.** Declare chassis command space, maximum speed, acceleration, steering rate, actuator range,
   braking authority and safe values before activation. A missing limit is a degraded configuration, never an unlimited
   default.
6. **Calibrate in place.** Verify LiDAR/IMU extrinsics, IMU axes and bias, wheel radius and signs, encoder resolution,
   steering zero/range, command delay and clock offset. Keep calibration data separate from the algorithm binary.
7. **Run provider conformance.** Run lifecycle, sequence, timestamp, expiry, watchdog, mixed-provider and replay tests with
   the native adapter. First replay recorded native observations through the ROBOT stack; then test with wheels lifted;
   finally test slow motion on a protected surface with an independent emergency stop.
8. **Enable autonomy gradually.** Start with odometry and stationary LiDAR checks, then known-map local correction, then
   controlled mapping, then planner and task execution. Keep simulator truth disabled in all native paths.

## Device-specific acceptance checks

**LiDAR:** verify range units and invalid values, scan period, per-point timing (including 16-line ring order), frame
orientation, minimum/maximum range, occlusion and packet loss. A scan timestamp is the acquisition time, not the arrival
time.

**IMU:** verify right-handed axes, gravity sign, angular-rate units, bias stability, covariance/noise declaration, sample
clock and saturation behavior. Test a stationary robot before using motion data.

**Encoders:** verify positive direction, wheel radius, gear ratio, ticks per revolution, rollover handling, timestamp
ordering and left/right or steering-joint identity. Compare integrated encoder motion against a measured short straight
run and a turn.

**Actuators:** verify every declared channel is written exactly once per command, mode and units match the capability,
limits saturate in the provider, expired commands safe-stop, and a disconnected controller cannot retain the previous
command.

**Power:** publish voltage, signed current, state of charge, remaining energy, temperature and charging/fault state with
freshness. `RobotExecutive` decides reserve and return-to-charge policy; the provider owns battery and charger physics.

## Configuration boundary

The native deployment selects provider IDs and calibration/profile paths at the composition root. It does not change the
planner configuration merely because the provider is native. Planner changes are justified only by measured chassis
capabilities (for example, choosing Hybrid A* for an Ackermann platform), and those capabilities are declared through the
same provider-neutral platform contract.

The minimum production evidence is a recorded native LSRI trace that can be replayed byte-for-byte, a calibration report,
an independent emergency-stop check, and a provider-conformance report. Only after those pass should a native robot be
allowed to publish mission completion or map revisions.
