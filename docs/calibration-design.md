# Calibration and hardware profile design

Calibration is a versioned evidence product. It is not a runtime escape hatch
for a failed route or a way to relax Safety. The ROBOT consumes only the
published calibration artifact after its hash, units, range and robot
description match the active profile.

The first deployable schema is implemented in
`src/robot/platform/calibration.hpp/.cpp`. It provides deterministic JSON
serialization, a median wheel-scale fit plus stationary IMU-bias fit for
offline recorded measurements, and an online wheel-scale update with a
confirmation window, bounded step/range, and explicit checkpoint rollback.
The solver is a profile tool; it is not called from the navigation tick.

The portable CSV bridge uses one ordered acquisition stream. Every row must
have a finite, non-negative timestamp strictly greater than the previous row;
all wheel, reference, accelerometer and gyro fields are finite before the
solver sees them. This rejects truncated, reordered and duplicated field-log
rows instead of changing their effective fit weight. Residuals are likewise
non-negative at the online update boundary. These checks are record-quality
contracts, not scenario-specific thresholds.

## Offline calibration

The input is a recorded measurement set with acquisition timestamps, sensor
source IDs, robot description hash, firmware revision, load/temperature notes
and an immutable digest. A calibration run produces:

- wheel radius, track width, wheel base and per-wheel scale, with covariance;
- IMU gyro/accelerometer bias, scale and axis alignment, with temperature range;
- LiDAR/IMU/encoder extrinsics and measured time offsets;
- motor feedforward, response delay and PID gains, with step-response quality;
- low-grip/slip model coefficients only when the experiment contains measured
  encoder, IMU and scan residual evidence.

Each parameter carries SI units, bounds, covariance, source dataset digest,
solver revision and an applicability range. A result outside the declared
range is rejected at bring-up instead of silently extrapolated. SIM can replay
the same input schema with noise and delivery delay, but a WorldTruth value is
never written into a calibration measurement.

## Online calibration

Online updates run in a separate calibration state from navigation. The
estimator/controller continues using the last accepted snapshot while a new
candidate accumulates a fixed evidence window. An update is accepted only when
the measurement residual improves, the covariance is finite, the parameter
stays inside its profile bounds, and the health monitor has no active safety or
transport fault. Accepted snapshots are journaled with sequence, timestamp,
input digest and rollback parent; a later degradation restores the parent.

Online adaptation is limited to measured hardware behavior such as wheel scale,
IMU bias, low-speed slip factor and execution delay. It cannot change the
footprint, collision inflation, safety polygon, arrival contract, planner
search limits or task objective. Those remain versioned configuration and must
be revalidated offline.

## Deployment contract

Bring-up checks the robot description hash, calibration artifact version,
profile revision, units, ownership and applicability range before enabling
motion. Native adapters can use the platform
`validate_native_hardware_profile()` seam to validate the board's typed
chassis capabilities and calibration artifact together, including profile id,
revision, ownership, joint layout, temperature, load and expiry. The MCU
receives only the accepted actuator profile and typed setpoints; it does not
receive SIM state, map truth or calibration solver internals. A missing,
stale, out-of-range or incompatible artifact enters safe hold.

For a first bench bring-up, `make_nominal_native_hardware_profile()` derives a
deterministic commissioning artifact from the robot description and the board
capability handshake. Its wheel and track scales are the declared values
(1.0), IMU bias and LiDAR time offset start at zero, and its digest is explicitly
prefixed `nominal:`. The default temperature applicability is `-40…85 °C`; a
`0/0` load range means that no payload applicability has been declared. The validator rejects
that artifact unless the adapter sets `allow_nominal_commissioning`; this
keeps a typical starting profile convenient without allowing it to masquerade
as measured calibration in production.

## First-use path

The first run does not require a user to invent wheel, track or IMU numbers.
The adapter performs the typed board handshake, passes the active
`RobotDescription` and returned capabilities to
`make_nominal_native_hardware_profile()`, and validates it with
`allow_nominal_commissioning = true` for a bench-only session. The generated
profile uses the declared geometry as its nominal prior and contains no
simulator pose, actor identity or score. The source tag, `nominal:` digest and
zero sample count are checked together, so the commissioning switch cannot
turn measured evidence into a nominal artifact or the reverse.

After the first safe bench run, the operator records the short straight-line
and stationary IMU CSV described above, runs `calibrate_offline()`, persists
the measured artifact atomically, and removes the commissioning flag. A
production adapter accepts only that measured artifact; a missing, stale or
incompatible file remains in safe hold. This keeps the easy path to three
steps—handshake, nominal bench profile, measured CSV—without weakening the
deployment gate.
