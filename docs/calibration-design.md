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
motion. The MCU receives only the accepted actuator profile and typed
setpoints; it does not receive SIM state, map truth or calibration solver
internals. A missing, stale, out-of-range or incompatible artifact enters safe
hold.
